"""Phase transitions must recover stale discovery and never signal unrelated PIDs."""
import importlib.util
from pathlib import Path
from types import SimpleNamespace as NS
from concurrent.futures import Future
import pytest

ROOT=Path(__file__).resolve().parents[4]
spec=importlib.util.spec_from_file_location('runtime',ROOT/'scripts/staged_delivery_runtime.py')
runtime=importlib.util.module_from_spec(spec);spec.loader.exec_module(runtime)


class Client:
    def __init__(self,reply_after=2):
        self.requests=[];self.removed=[];self.reply_after=reply_after
    def service_is_ready(self):return True
    def call_async(self,request):
        future=Future();self.requests.append(future)
        if len(self.requests)>=self.reply_after:
            future.set_result(NS(current_state=NS(id=3)))
        return future
    def remove_pending_request(self,future):self.removed.append(future)


def clock(monkeypatch):
    value=[0.]
    monkeypatch.setattr(runtime.time,'monotonic',lambda:value[0])
    def pump():value[0]+=.5
    return pump


def test_old_service_request_is_retried_instead_of_waiting_forever(monkeypatch):
    client=Client()
    runtime.wait_for_active([client],clock(monkeypatch),object,timeout=10,request_timeout=2)
    assert len(client.requests)==2
    assert client.removed==client.requests[:1]


def test_timeout_removes_pending_requests(monkeypatch):
    client=Client(reply_after=100)
    with pytest.raises(RuntimeError,match='did not activate'):
        runtime.wait_for_active([client],clock(monkeypatch),object,timeout=4,request_timeout=2)
    assert client.removed==client.requests


def test_cleanup_rejects_stale_pid_and_borrowed_process_group(tmp_path,monkeypatch):
    pid_file=tmp_path/'owned.pid';pid_file.write_text('42')
    proc=tmp_path/'proc';entry=proc/'42';entry.mkdir(parents=True)
    calls=[];monkeypatch.setattr(runtime.os,'killpg',lambda *args:calls.append(args))
    monkeypatch.setattr(runtime.os,'getpgid',lambda pid:pid)
    (entry/'cmdline').write_bytes(b'python3\0unrelated.py\0')
    assert not runtime.signal_owned_group(pid_file,'delivery_slice.py',proc)
    (entry/'cmdline').write_bytes(b'python3\0/path/delivery_slice.py\0')
    monkeypatch.setattr(runtime.os,'getpgid',lambda pid:41)
    assert not runtime.signal_owned_group(pid_file,'delivery_slice.py',proc)
    assert not calls
    monkeypatch.setattr(runtime.os,'getpgid',lambda pid:pid)
    assert runtime.signal_owned_group(pid_file,'delivery_slice.py',proc)
    assert calls==[(42,runtime.signal.SIGINT)]
