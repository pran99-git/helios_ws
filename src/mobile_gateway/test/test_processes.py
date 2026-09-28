"""Process ownership tests use fake Popen and fake signalling exclusively."""
import os
import signal
import time
import unittest

from mobile_gateway.processes import ProcessBackend, SimulatedProcessBackend


class FakeProcess:
    def __init__(self, output=b''):
        read_fd, write_fd = os.pipe()
        os.write(write_fd, output)
        os.close(write_fd)
        self.stdout = os.fdopen(read_fd, 'rb')
        self.pid = 987654
        self.code = None

    def poll(self):
        return self.code


def eventually(predicate):
    deadline = time.monotonic() + 2
    while not predicate():
        assert time.monotonic() < deadline, 'Timed out waiting for fake process watcher'
        time.sleep(.01)


def backend_for(proc, **kwargs):
    calls, launches = [], []

    def popen(argv, **options):
        launches.append((argv, options))
        return proc

    def killpg(pid, sig):
        calls.append((pid, sig))
        if sig == 0:
            raise ProcessLookupError()

    backend = ProcessBackend(popen_factory=popen, killpg=killpg, **kwargs)
    return backend, calls, launches


def test_launch_environment_isolation_and_bounded_output():
    proc = FakeProcess(b'x' * 2048)
    backend, calls, launches = backend_for(
        proc, environ={'HELIOS_GATEWAY_TOKEN': 'do-not-inherit', 'ROS_DOMAIN_ID': '7'})
    try:
        backend.start('job', ['ros2', 'launch', 'example'])
        argv, options = launches[0]
        assert argv == ['ros2', 'launch', 'example']
        assert options['shell'] is False
        assert options['start_new_session'] is True
        assert options['env'] == {'ROS_DOMAIN_ID': '7'}
        eventually(lambda: len(backend.status('job')['output']) == 1024)
        assert backend.status('job')['output'] == 'x' * 1024
        assert calls == []
    finally:
        proc.code = 0
        eventually(lambda: backend.status('job')['state'] == 'failed')


def test_stop_deadline_retains_ownership_without_force_kill():
    proc = FakeProcess()
    backend, calls, _ = backend_for(proc, stop_timeout=.03)
    backend.start('job', ['ros2'])
    assert backend.stop('job')['state'] == 'stopping'
    eventually(lambda: backend.status('job')['state'] == 'stop_failed')
    assert not backend.forget('job')
    backend.stop('job')
    assert calls == [(proc.pid, signal.SIGINT)]
    proc.code = 0
    eventually(lambda: backend.status('job')['state'] == 'stopped')
    backend.stop('job')
    assert calls == [(proc.pid, signal.SIGINT), (proc.pid, 0)]
    assert backend.forget('job')


def test_one_shot_timeout_interrupts_and_reports_failure():
    proc = FakeProcess()
    backend, calls, _ = backend_for(proc)
    backend.start('job', ['ros2'], one_shot=True, timeout_seconds=.03)
    eventually(lambda: backend.status('job')['state'] == 'stopping')
    assert calls == [(proc.pid, signal.SIGINT)]
    proc.code = 0
    eventually(lambda: backend.status('job')['state'] == 'failed')
    assert 'timed out' in backend.status('job')['error']


def test_reaped_leader_never_signalled_even_if_group_remains():
    proc = FakeProcess()
    calls = []
    backend = ProcessBackend(popen_factory=lambda *a, **k: proc,
                             killpg=lambda pid, sig: calls.append((pid, sig)))
    backend.start('job', ['ros2'])
    proc.code = 0
    eventually(lambda: backend.status('job')['state'] == 'stop_failed')
    assert 'ownership relinquished' in backend.status('job')['error']
    backend.stop('job')
    backend.shutdown()
    assert calls == [(proc.pid, 0)]  # Read-only probe only, no signal to descendants/reused PGID.
    assert not backend.forget('job')


def test_simulated_backend_actions_and_shutdown():
    backend = SimulatedProcessBackend()
    assert backend.start('service', ['never-executed'])['state'] == 'running'
    assert backend.start('action', ['never-executed'], one_shot=True)['state'] == 'running'
    eventually(lambda: backend.status('action')['state'] == 'succeeded')
    backend.shutdown()
    assert backend.status('service')['state'] == 'stopped'
    assert backend.forget('action')


def load_tests(loader, tests, pattern):
    """Keep mock scenarios directly runnable and discoverable by unittest."""
    return unittest.TestSuite(
        unittest.FunctionTestCase(function)
        for name, function in globals().items() if name.startswith('test_')
    )


def test_streamed_failure_marker_survives_chunk_boundary_and_tail_truncation():
    for marker in (b'success=False', b'success: false'):
        proc = FakeProcess(b'x' * 1020 + marker + b'y' * 1500)
        backend, _, _ = backend_for(proc)
        backend.start('job', ['ros2'], one_shot=True)
        proc.code = 0
        eventually(lambda: backend.status('job')['state'] == 'succeeded')
        result = backend.status('job')
        assert result['reported_failure'] is True
        assert result['output'] == 'y' * 1024


def test_reaped_action_waits_for_reader_without_retaining_signal_ownership():
    proc = FakeProcess()
    proc.stdout.close()
    read_fd, write_fd = os.pipe()
    proc.stdout = os.fdopen(read_fd, 'rb')
    backend, calls, _ = backend_for(proc)
    try:
        backend.start('job', ['ros2'], one_shot=True)
        proc.code = 0
        eventually(lambda: backend.status('job')['exit_code'] == 0)
        assert backend.status('job')['state'] == 'running'
        assert not backend.forget('job')
        backend.stop('job')
        assert calls == [(proc.pid, 0)]
        os.write(write_fd, b'success=False')
    finally:
        os.close(write_fd)
    eventually(lambda: backend.status('job')['state'] == 'succeeded')
    assert backend.status('job')['reported_failure'] is True
    assert calls == [(proc.pid, 0)]


def test_missing_output_eof_never_reports_success():
    proc = FakeProcess()
    proc.stdout.close()
    read_fd, write_fd = os.pipe()
    proc.stdout = os.fdopen(read_fd, 'rb')
    backend, calls, _ = backend_for(proc)
    try:
        backend.start('job', ['ros2'], one_shot=True)
        proc.code = 0
        eventually(lambda: backend.status('job')['state'] == 'failed')
        assert 'output was incomplete' in backend.status('job')['error']
        backend.stop('job')
        assert calls == [(proc.pid, 0)]
    finally:
        os.close(write_fd)


def test_service_exit_fails_before_pipe_eof():
    proc = FakeProcess()
    proc.stdout.close()
    read_fd, write_fd = os.pipe()
    proc.stdout = os.fdopen(read_fd, 'rb')
    backend, _, _ = backend_for(proc)
    try:
        backend.start('job', ['ros2'])
        proc.code = 0
        eventually(lambda: backend.status('job')['exit_code'] == 0)
        assert backend.status('job')['state'] == 'failed'
    finally:
        os.close(write_fd)


if __name__ == '__main__':
    unittest.main()
