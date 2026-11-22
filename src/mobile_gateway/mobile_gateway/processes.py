"""Asynchronous process ownership; launch status is never a ROS health check.

Only a trusted operation catalog may supply argv. No shell, privilege escalation,
process discovery, or signalling of processes not created by this backend exists.
"""
import os
import signal
import subprocess
import threading
import time


class ProcessBackend:
    """Own a session per launch and interrupt it without destructive escalation.

    A stopped leader can leave descendants behind. We deliberately relinquish
    signalling ownership when the leader is reaped: a subsequent process group
    with the same number must never be signalled. Operators must investigate
    remaining descendants themselves. No automatic SIGKILL is used.
    """

    def __init__(self, cwd=None, *, stop_timeout=15.0, max_output_bytes=1024,
                 popen_factory=subprocess.Popen, killpg=os.killpg,
                 clock=time.monotonic, environ=None):
        self.cwd = cwd
        self.stop_timeout = float(stop_timeout)
        self.max_output_bytes = int(max_output_bytes)
        if self.stop_timeout <= 0 or self.max_output_bytes <= 0:
            raise ValueError('timeouts and output bounds must be positive')
        self._popen = popen_factory
        self._killpg = killpg
        self._clock = clock
        self._env = dict(os.environ if environ is None else environ)
        self._env.pop('HELIOS_GATEWAY_TOKEN', None)
        self._lock = threading.RLock()
        self._entries = {}
        self._closed = False

    def start(self, operation_id, argv, *, one_shot=False, timeout_seconds=30):
        if not isinstance(argv, (list, tuple)) or not argv or any(
                not isinstance(a, str) or '\0' in a for a in argv):
            raise ValueError('argv must be a nonempty sequence of strings')
        if one_shot and not 0 < timeout_seconds <= 3600:
            raise ValueError('one-shot timeout must be between 0 and 3600 seconds')
        with self._lock:
            if self._closed:
                raise RuntimeError('backend is shutting down')
            prior = self._entries.get(operation_id)
            if prior and prior['process'] is not None:
                raise ValueError('operation already owns a process')
            entry = dict(state='running', pid=None, exit_code=None, output='',
                         error=None, started_at=time.time(), process=None,
                         tail=b'', started=self._clock(), stop_started=None,
                         one_shot=one_shot, timeout=timeout_seconds,
                         timed_out=False, orphaned=False, reported_failure=False,
                         scan_tail=b'', output_done=threading.Event(), output_error=False)
            self._entries[operation_id] = entry
            try:
                proc = self._popen(list(argv), cwd=self.cwd, env=self._env.copy(),
                                   shell=False, start_new_session=True,
                                   stdin=subprocess.DEVNULL,
                                   stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
            except (OSError, ValueError) as exc:
                entry.update(state='failed', error='Launch failed: ' + str(exc))
                return self._snapshot(entry)
            entry.update(process=proc, pid=proc.pid)
            threading.Thread(target=self._read_output, args=(entry, proc),
                             daemon=True).start()
            threading.Thread(target=self._watch, args=(entry, proc),
                             daemon=True).start()
            return self._snapshot(entry)

    def _read_output(self, entry, proc):
        try:
            while True:
                chunk = os.read(proc.stdout.fileno(), 1024)
                if not chunk:
                    break
                with self._lock:
                    scanned = entry['scan_tail'] + chunk
                    if b'success=False' in scanned or b'success: false' in scanned:
                        entry['reported_failure'] = True
                    entry['scan_tail'] = scanned[-32:]
                    entry['tail'] = (entry['tail'] + chunk)[-self.max_output_bytes:]
        except (OSError, ValueError):
            with self._lock:
                entry['output_error'] = True
        finally:
            proc.stdout.close()
            entry['output_done'].set()

    def _interrupt(self, entry):
        proc = entry['process']
        if proc is None or entry['stop_started'] is not None:
            return
        # All polling/reaping and signalling uses this lock. Until poll reaps
        # the leader its PID cannot be reused, including while it is a zombie.
        entry.update(state='stopping', stop_started=self._clock())
        try:
            self._killpg(proc.pid, signal.SIGINT)
        except ProcessLookupError:
            pass  # Watcher records the actual leader exit status.
        except OSError as exc:
            entry['error'] = 'Could not interrupt owned process group: ' + str(exc)

    def _watch(self, entry, proc):
        while True:
            with self._lock:
                code = proc.poll()
                if code is not None:
                    was_stopping = entry['stop_started'] is not None
                    entry.update(process=None, exit_code=code)
                    # Read-only existence probe. Ownership ends with the leader;
                    # never signal descendants once that PID has been reaped.
                    try:
                        self._killpg(proc.pid, 0)
                        group_remains = True
                    except ProcessLookupError:
                        group_remains = False
                    except OSError:
                        group_remains = True  # Cannot establish shutdown safely.
                    if group_remains:
                        entry.update(state='stop_failed', orphaned=True,
                                     error='Launch leader exited but process group may remain; ownership relinquished; operator intervention required')
                        return
                    if not entry['one_shot'] and not was_stopping:
                        entry.update(state='failed', error='Launch process exited; ROS health and descendants are unknown')
                        return
                    break  # Release lock before waiting for output EOF.
                now = self._clock()
                if entry['one_shot'] and now - entry['started'] >= entry['timeout'] and entry['stop_started'] is None:
                    entry.update(timed_out=True, error='Operation timed out; graceful interrupt requested')
                    self._interrupt(entry)
                if entry['stop_started'] is not None and now - entry['stop_started'] >= self.stop_timeout:
                    entry['state'] = 'stop_failed'
                    entry['error'] = 'Graceful stop deadline exceeded; process still owned; no force kill performed'
            time.sleep(0.05)

        # The process has already been reaped and ownership relinquished. A
        # pipe may still be held by a detached descendant; never assume success
        # before all output has been inspected, and never wait indefinitely.
        drained = entry['output_done'].wait(1.0)
        with self._lock:
            if not drained or entry['output_error']:
                entry.update(state='failed', error='Launch exited but output was incomplete; result unverified')
            elif entry['timed_out']:
                entry['state'] = 'failed'
            elif was_stopping:
                entry['state'] = 'stopped'
            elif entry['one_shot'] and code == 0:
                entry['state'] = 'succeeded'
            else:
                entry.update(state='failed', error='Launch process exited; ROS health and descendants are unknown')

    @staticmethod
    def _snapshot(entry):
        result = {k: entry[k] for k in ('state', 'pid', 'exit_code', 'error', 'started_at', 'reported_failure')}
        result['output'] = entry['tail'].decode('utf-8', errors='replace')
        return result

    def status(self, operation_id):
        with self._lock:
            entry = self._entries.get(operation_id)
            return self._snapshot(entry) if entry else None

    def statuses(self):
        with self._lock:
            return {key: self._snapshot(entry) for key, entry in self._entries.items()}

    def stop(self, operation_id):
        with self._lock:
            entry = self._entries.get(operation_id)
            if entry is None:
                return None
            self._interrupt(entry)
            return self._snapshot(entry)

    def forget(self, operation_id):
        """Discard terminal history only; a still-owned process is retained."""
        with self._lock:
            entry = self._entries.get(operation_id)
            if entry is not None and (entry['process'] is not None or entry['orphaned'] or entry['state'] in ('running', 'stopping')):
                return False
            self._entries.pop(operation_id, None)
            return True

    def shutdown(self):
        """Request graceful interrupts and return without blocking HTTP shutdown."""
        with self._lock:
            self._closed = True
            for entry in self._entries.values():
                self._interrupt(entry)


class SimulatedProcessBackend:
    """Deterministic fixture backend: never constructs a subprocess."""

    def __init__(self, *args, **kwargs):
        self._entries = {}
        self._lock = threading.RLock()
        self._closed = False

    def start(self, operation_id, argv, *, one_shot=False, timeout_seconds=30):
        with self._lock:
            if self._closed:
                raise RuntimeError('backend is shutting down')
            prior = self._entries.get(operation_id)
            if prior and prior['state'] in ('running', 'stopping'):
                raise ValueError('operation already owns a process')
            self._entries[operation_id] = dict(
                state='running', pid=None,
                exit_code=None, error=None, reported_failure=False,
                started_at=time.time(), output='Simulated operation; no hardware command executed')
            if one_shot:
                timer = threading.Timer(0.1, self._finish, args=(operation_id,))
                timer.daemon = True
                timer.start()
            return dict(self._entries[operation_id])

    def _finish(self, operation_id):
        with self._lock:
            entry = self._entries.get(operation_id)
            if entry and entry['state'] == 'running':
                entry.update(state='succeeded', exit_code=0)

    def status(self, operation_id):
        with self._lock:
            entry = self._entries.get(operation_id)
            return dict(entry) if entry else None

    def statuses(self):
        with self._lock:
            return {key: dict(entry) for key, entry in self._entries.items()}

    def stop(self, operation_id):
        with self._lock:
            entry = self._entries.get(operation_id)
            if entry and entry['state'] in ('running', 'stopping'):
                entry.update(state='stopped', exit_code=0)
            return dict(entry) if entry else None

    def forget(self, operation_id):
        with self._lock:
            entry = self._entries.get(operation_id)
            if entry and entry['state'] in ('running', 'stopping', 'stop_failed'):
                return False
            self._entries.pop(operation_id, None)
            return True

    def shutdown(self):
        with self._lock:
            self._closed = True
            for key in self._entries:
                self.stop(key)
