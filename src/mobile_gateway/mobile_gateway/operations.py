"""Serialized allowlisted operations, exclusive expiring control lease, and jobs."""
import re
import threading
import time
import uuid
from dataclasses import asdict
from pathlib import Path

from .catalog import OPERATIONS, build_argv

ACTIVE = {'running', 'stopping', 'stop_failed'}
TERMINAL = {'succeeded', 'failed', 'stopped'}


class OperationError(Exception):
    def __init__(self, status, code, message):
        super().__init__(message)
        self.status, self.code = status, code


class OperationManager:
    def __init__(self, backend, workspace, *, enabled=False, simulated=False,
                 clock=time.monotonic, lease_seconds=10., external_guard=None):
        self.backend, self.workspace = backend, Path(workspace)
        self.enabled, self.simulated = enabled, simulated
        self.clock, self.lease_seconds = clock, lease_seconds
        self.external_guard = external_guard
        self.lock = threading.RLock()
        self.owner = None
        self.expires = 0.
        self.jobs = {}
        self.requests = {}
        self.stopping_all = False
        self.stop_worker = None
        self.stop_event = threading.Event()
        self.monitor = threading.Thread(target=self._monitor, daemon=True)
        self.monitor.start()

    def _lease(self):
        return {'client_id': self.owner,
                'remaining_seconds': max(0., self.expires - self.clock()) if self.owner else 0.}

    def _enabled(self):
        if not self.enabled:
            raise OperationError(403, 'commands_disabled', 'Commands are disabled on this gateway.')

    @staticmethod
    def _identifier(value):
        if not isinstance(value, str) or not re.fullmatch(r'[A-Za-z0-9_-]{8,64}', value):
            raise OperationError(400, 'invalid_id', 'Identifiers must be 8-64 letters, digits, hyphens or underscores.')
        return value

    def _authorize(self, client_id):
        self._enabled()
        self._identifier(client_id)
        if client_id != self.owner or self.clock() >= self.expires:
            raise OperationError(409, 'lease_required', 'Acquire and maintain the exclusive control lease first.')

    def heartbeat(self, client_id):
        with self.lock:
            self._enabled()
            self._identifier(client_id)
            self._expire()
            if self.stopping_all:
                raise OperationError(409, 'stopping', 'Owned jobs are stopping; wait before acquiring control.')
            if self.owner is not None and self.owner != client_id:
                raise OperationError(409, 'lease_owned', 'Another client owns control.')
            self.owner, self.expires = client_id, self.clock() + self.lease_seconds
            return {'api_version': 1, 'lease': self._lease()}

    def _snapshot(self, job_id):
        if job_id not in self.jobs:
            raise OperationError(404, 'job_not_found', 'Unknown job.')
        details = self.backend.status(job_id)
        if self.jobs[job_id]['operation_id'] in ('recovery_abort', 'recovery_relocalize') and details['state'] == 'succeeded':
            if details.get('reported_failure'):
                details = {**details, 'state': 'failed', 'error': 'ROS service reported success=false; inspect output.'}
        operation_id = self.jobs[job_id]['operation_id']
        if not self.simulated and details['state'] == 'succeeded' and operation_id in ('save_slam', 'save_rtab_map', 'export_rtab_cloud'):
            name = self.jobs[job_id]['parameters']['name']
            family = 'slam_toolbox' if operation_id == 'save_slam' else 'rtabmap'
            extensions = ('.pgm', '.yaml', '.posegraph', '.data') if operation_id == 'save_slam' else (('_cloud.ply',) if operation_id == 'export_rtab_cloud' else ('.pgm', '.yaml'))
            prefix = self.workspace / 'src/mapping_localization_pkg' / family / 'maps' / (family + '_' + name)
            try:
                complete = all(Path(str(prefix) + ext).is_file() and Path(str(prefix) + ext).stat().st_size > 0 for ext in extensions)
            except OSError:
                complete = False
            if not complete:
                details = {**details, 'state': 'failed', 'error': 'CLI exited but expected nonempty output files are missing; inspect logs and map directory.'}
        if details.get('error') is not None:
            details = {**details, 'error': details['error'][:128]}
        return {**details, 'id': job_id, 'operation_id': self.jobs[job_id]['operation_id'],
                'simulated': self.simulated}

    def job(self, job_id):
        with self.lock:
            return {'api_version': 1, 'job': self._snapshot(job_id)}

    def _active(self):
        return {key: self._snapshot(key) for key in self.jobs
                if self.backend.status(key)['state'] in ACTIVE}

    def catalog(self):
        with self.lock:
            definitions = []
            for op in OPERATIONS.values():
                item = asdict(op)
                item['requires_confirmation'] = item.pop('movement_capable')
                definitions.append(item)
            return {'api_version': 1, 'source': 'fixture' if self.simulated else 'ros2',
                    'commands_enabled': self.enabled, 'lease': self._lease(),
                    'operations': definitions, 'jobs': [self._summary(key) for key in self.jobs]}

    def _summary(self, job_id):
        job = self._snapshot(job_id)
        job['output'] = job['output'][-128:]
        return job

    def start(self, operation_id, body):
        with self.lock:
            self._authorize(body.get('client_id'))
            request_id = self._identifier(body.get('request_id'))
            if set(body) - {'client_id', 'request_id', 'parameters', 'confirm'}:
                raise OperationError(400, 'invalid_parameters', 'Unknown request fields.')
            signature = (operation_id, body.get('parameters', {}), body.get('confirm', False))
            previous = self.requests.get(request_id)
            if previous:
                if previous[0] != self.owner or previous[1] != signature:
                    raise OperationError(409, 'request_id_reused', 'Request identifier was used for a different intent.')
                return self.job(previous[2])
            if self.stopping_all:
                raise OperationError(409, 'stopping', 'Wait for stop-all to finish.')
            op = OPERATIONS.get(operation_id)
            if op is None:
                raise OperationError(404, 'operation_not_found', 'Unknown operation.')
            if op.movement_capable and body.get('confirm') is not True:
                raise OperationError(403, 'confirmation_required', 'This operation can enable robot motion; explicit confirmation is required.')
            active = self._active()
            ids = {job['operation_id'] for job in active.values()}
            if operation_id in ids:
                raise OperationError(409, 'already_running', 'This operation already has an active job.')
            if any(job['state'] != 'running' for job in active.values()):
                raise OperationError(409, 'stopping', 'Resolve stopping jobs before starting another operation.')
            if not set(op.requires) <= ids or (op.requires_any and not set(op.requires_any) & ids):
                raise OperationError(409, 'dependencies', 'Start required services first: ' + ', '.join(op.requires + op.requires_any))
            conflicts = set(op.conflicts) & ids
            conflicts.update(key for key in ids if operation_id in OPERATIONS[key].conflicts)
            if conflicts:
                raise OperationError(409, 'conflict', 'Conflicting operation is active: ' + ', '.join(sorted(conflicts)))
            if self.external_guard:
                self.external_guard(ids, operation_id)
            try:
                argv = build_argv(operation_id, body.get('parameters', {}), self.workspace, simulated=self.simulated)
            except (ValueError, TypeError, OSError) as error:
                raise OperationError(400, 'invalid_parameters', str(error)) from error
            if len(self.jobs) >= 128:
                raise OperationError(409, 'history_full', 'Job history is full; stop all jobs and restart gateway before more operations.')
            self._authorize(body.get('client_id'))
            job_id = str(uuid.uuid4())
            self.backend.start(job_id, argv, one_shot=op.kind == 'action', timeout_seconds=op.timeout_seconds)
            self.jobs[job_id] = {'operation_id': operation_id, 'parameters': body.get('parameters', {})}
            self.requests[request_id] = (self.owner, signature, job_id)
            return self.job(job_id)

    def stop(self, job_id, client_id):
        with self.lock:
            self._authorize(client_id)
            target = self._snapshot(job_id)
            if target['operation_id'] in ('recovery_relocalize', 'global_localization'):
                raise OperationError(409, 'action_not_cancelable', 'Stopping the CLI does not undo this ROS service; use recovery_abort or stop-all.')
            for other in self._active().values():
                if other['id'] == job_id:
                    continue
                op = OPERATIONS[other['operation_id']]
                if target['operation_id'] in op.requires + op.requires_any:
                    raise OperationError(409, 'dependents', 'Stop dependent jobs first or use stop-all.')
            self.backend.stop(job_id)
            return self.job(job_id)

    def stop_all(self, client_id):
        with self.lock:
            self._authorize(client_id)
            self._begin_stop_all()
            return {'api_version': 1, 'jobs': [self._summary(key) for key in self.jobs]}

    def _begin_stop_all(self):
        if self.stopping_all:
            return
        self.stopping_all = True
        self.stop_worker = threading.Thread(target=self._stop_all_worker, daemon=True)
        self.stop_worker.start()

    def _stop_all_worker(self):
        # Interrupt motion producers first, then the rest in reverse dependency
        # order. Never let a stuck diagnostic/save prevent stopping motors.
        with self.lock:
            ordered = list(reversed(self.jobs))
            ordered.sort(key=lambda key: 0 if self.jobs[key]['operation_id'] in ('navigation', 'joystick') else 1)
        try:
            for job_id in ordered:
                self.backend.stop(job_id)
            deadline = time.monotonic() + 35
            while time.monotonic() < deadline:
                states = [self.backend.status(key)['state'] for key in ordered]
                if all(state in TERMINAL or state == 'stop_failed' for state in states):
                    return
                time.sleep(.05)
        finally:
            with self.lock:
                self.stopping_all = any(self.backend.status(key)['state'] in ACTIVE for key in ordered)

    def _expire(self):
        if self.owner is not None and self.clock() >= self.expires:
            self.owner = None
            self._begin_stop_all()

    def _monitor(self):
        while not self.stop_event.wait(.1):
            with self.lock:
                if self.stopping_all and self.stop_worker is not None and not self.stop_worker.is_alive() and not self._active():
                    self.stopping_all = False
                self._expire()
                # A failed/exited prerequisite invalidates dependent launch state.
                active = self._active()
                ids = {job['operation_id'] for job in active.values() if job['state'] == 'running'}
                for job in active.values():
                    op = OPERATIONS[job['operation_id']]
                    if not set(op.requires) <= ids or (op.requires_any and not set(op.requires_any) & ids):
                        self._begin_stop_all()
                        break

    def shutdown(self):
        self.enabled = False
        self.stop_event.set()
        self.monitor.join(timeout=1)
        with self.lock:
            self.owner = None
            self._begin_stop_all()

        if self.stop_worker is not None:
            self.stop_worker.join(timeout=36)
        self.backend.shutdown()
