"""Bounded readiness and process ownership for the staged leader mission."""
import os
from pathlib import Path
import signal
import time


def wait_for_active(clients, pump, request_factory, timeout=60., request_timeout=3.):
    deadline=time.monotonic()+timeout
    ready=set();pending={}
    try:
        while time.monotonic()<deadline:
            pump()
            for index,client in enumerate(clients):
                if index in ready:continue
                if index in pending:
                    future,sent=pending[index]
                    if future.done():
                        pending.pop(index)
                        try:
                            if future.result().current_state.id==3:ready.add(index)
                        except Exception:pass
                    elif time.monotonic()-sent>=request_timeout:
                        client.remove_pending_request(future);pending.pop(index)
                if index not in ready and index not in pending and client.service_is_ready():
                    pending[index]=(client.call_async(request_factory()),time.monotonic())
            if len(ready)==len(clients):return
        raise RuntimeError('Reverse delivery Nav2 did not activate')
    finally:
        for index,(future,_) in pending.items():clients[index].remove_pending_request(future)


def signal_owned_group(pid_file, expected, proc_root=Path('/proc')):
    """A stale PID file must never stop an unrelated process or borrowed group."""
    try:
        pid=int(Path(pid_file).read_text().strip())
        if pid<=1:return False
        argv=[a.decode(errors='replace') for a in (proc_root/str(pid)/'cmdline').read_bytes().split(b'\0') if a]
        if not any(Path(a).name==expected for a in argv):return False
        if os.getpgid(pid)!=pid:return False
        os.killpg(pid,signal.SIGINT)
        return True
    except (OSError,ValueError):return False
