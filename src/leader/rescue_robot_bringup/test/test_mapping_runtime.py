"""Process-only regressions; no ROS nodes, devices or containers are started."""
import importlib.util
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

import pytest

ROOT = Path(__file__).resolve().parents[4]
spec = importlib.util.spec_from_file_location('mapping_processes', ROOT/'scripts/mapping_processes.py')
runtime = importlib.util.module_from_spec(spec)
spec.loader.exec_module(runtime)


def wait_until(predicate, timeout=4):
    deadline = time.monotonic()+timeout
    while time.monotonic() < deadline:
        if predicate():
            return
        time.sleep(.03)
    assert predicate()


def test_owned_group_is_stopped_after_launcher_has_exited():
    parent = subprocess.Popen([sys.executable, '-c',
        'import subprocess,sys; p=subprocess.Popen([sys.executable,"-c","import time; time.sleep(60)"]); print(p.pid,flush=True)'],
        stdout=subprocess.PIPE, text=True, start_new_session=True)
    child_pid = int(parent.stdout.readline())
    group = parent.pid
    parent.wait(timeout=4)
    try:
        assert runtime.live_group(group)
        runtime.signal_group(group, 'TERM')
        wait_until(lambda: not runtime.live_group(group))
    finally:
        runtime.signal_group(group, 'KILL')
        parent.stdout.close()


def test_helper_refuses_own_or_system_group():
    for pgid in (0, 1, os.getpgrp()):
        with pytest.raises(ValueError):
            runtime.signal_group(pgid, 'TERM')


def test_session_wrapper_waits_for_child_and_preserves_exit_status():
    started = time.monotonic()
    process = subprocess.Popen(['setsid', '--wait', sys.executable, '-c',
        'import time; time.sleep(.25); raise SystemExit(7)'], start_new_session=True)
    assert process.wait(timeout=4) == 7
    assert time.monotonic()-started >= .2
    assert '"${CONTAINER_NAME}" setsid bash -lc' not in (ROOT/'scripts/run_vslam_mapping.sh').read_text()


def test_preexisting_leader_node_is_detected_without_touching_it(tmp_path):
    fixture = tmp_path/'realsense2_camera_node'
    fixture.write_text('import time; time.sleep(60)\n')
    process = subprocess.Popen([sys.executable, str(fixture), '__ns:=/leader'], start_new_session=True)
    try:
        wait_until(lambda: any(item['pid'] == process.pid for item in runtime.conflicts()))
        assert process.poll() is None
    finally:
        runtime.signal_group(process.pid, 'TERM')
        process.wait(timeout=4)


def launcher_fixture(tmp_path):
    scripts = tmp_path/'scripts'
    scripts.mkdir(exist_ok=True)
    source = (ROOT/'scripts/run_vslam_mapping.sh').read_text()
    # Exercise the launcher's actual lock, ownership guard and signal traps.
    # Stop before any ROS setup, device check or startup command can execute.
    prelude = source.split('if [[ -f "${LAUNCHER_PID_FILE}" ]]', 1)[0]
    path = scripts/'launcher_fixture.sh'
    path.write_text(prelude+'\nprintf "READY\\n"\nwhile true; do sleep 1; done\n')
    env = dict(os.environ, XDG_RUNTIME_DIR=str(tmp_path/'runtime'))
    env.pop('MAPPING_SNAPSHOT', None)
    return path, env


@pytest.mark.parametrize('signum,expected', [(signal.SIGINT, 130), (signal.SIGTERM, 143)])
def test_shutdown_signal_exits_instead_of_continuing_startup(tmp_path, signum, expected):
    script, env = launcher_fixture(tmp_path)
    process = subprocess.Popen(['bash', str(script)], env=env, stdout=subprocess.PIPE,
                               stderr=subprocess.STDOUT, text=True, start_new_session=True)
    try:
        assert process.stdout.readline().strip() == 'READY'
        os.killpg(process.pid, signum)
        assert process.wait(timeout=4) == expected
    finally:
        runtime.signal_group(process.pid, 'KILL')
        process.stdout.close()


def test_second_launcher_cannot_clean_up_the_running_session(tmp_path):
    script, env = launcher_fixture(tmp_path)
    first = subprocess.Popen(['bash', str(script)], env=env, stdout=subprocess.PIPE,
                             stderr=subprocess.STDOUT, text=True, start_new_session=True)
    try:
        assert first.stdout.readline().strip() == 'READY'
        second = subprocess.run(['bash', str(script)], env=env, capture_output=True, text=True, timeout=4)
        assert second.returncode == 1
        assert 'already running' in second.stderr
        assert first.poll() is None
    finally:
        runtime.signal_group(first.pid, 'TERM')
        first.wait(timeout=4)
        first.stdout.close()
