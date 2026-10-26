"""One leader signal starts only when both sides are available; failures cancel both."""
import ast
from concurrent.futures import Future
from pathlib import Path
import time
from types import SimpleNamespace as NS
from unittest.mock import Mock

SCRIPT = Path(__file__).resolve().parents[4]/'scripts/paired_grasp.py'
tree = ast.parse(SCRIPT.read_text())
node = next(n for n in tree.body if isinstance(n, ast.ClassDef))
namespace = {'Node': object, 'time': time, 'Trigger': NS(Request=lambda: NS())}
exec(compile(ast.Module(body=[node], type_ignores=[]), str(SCRIPT), 'exec'), namespace)
Pair = namespace['PairedGrasp']


def pair():
    obj = Pair.__new__(Pair)
    obj.active = False
    obj.phase = 'IDLE'
    obj.started = {}
    obj.done = {}
    obj.last_sequence = {}
    obj.futures = {}
    obj.start_clients = {role: Mock() for role in ('leader', 'follower')}
    obj.cancel_clients = {role: Mock() for role in ('leader', 'follower')}
    for client in (*obj.start_clients.values(), *obj.cancel_clients.values()):
        client.service_is_ready.return_value = True
        client.call_async.return_value = Future()
    obj._status = Mock()
    return obj


def start(obj):
    return obj._start(NS(), NS(success=False, message=''))


def test_missing_follower_never_starts_leader():
    obj = pair()
    obj.start_clients['follower'].service_is_ready.return_value = False
    assert not start(obj).success
    for client in obj.start_clients.values():
        client.call_async.assert_not_called()


def test_one_signal_calls_each_task_once_and_duplicate_start_is_rejected():
    obj = pair()
    assert start(obj).success
    assert not start(obj).success
    for client in obj.start_clients.values():
        client.call_async.assert_called_once()


def test_both_fresh_done_events_required_for_completion():
    obj = pair()
    assert start(obj).success
    for future in obj.futures.values():
        future.set_result(NS(success=True))
    obj._sequence('leader', NS(data='DONE old result'))
    obj._tick()
    assert obj.phase == 'RUNNING'
    for role in ('leader', 'follower'):
        obj._sequence(role, NS(data='STARTED task'))
    obj._sequence('leader', NS(data='DONE motor acknowledged'))
    obj._tick()
    assert obj.active
    obj._sequence('follower', NS(data='DONE motor acknowledged'))
    obj._tick()
    assert obj.phase == 'DONE' and not obj.active
    assert not start(obj).success


def test_one_fault_cancels_both_and_does_not_open_grippers():
    obj = pair()
    start(obj)
    obj._sequence('follower', NS(data='FAULT alignment lost'))
    assert obj.phase == 'FAULT'
    for client in obj.cancel_clients.values():
        client.call_async.assert_called_once()


def test_delayed_acceptance_after_cancel_is_cancelled_again():
    obj = pair()
    start(obj)
    obj._cancel_both('operator')
    obj.futures['follower'].set_result(NS(success=True))
    assert obj.cancel_clients['follower'].call_async.call_count == 2
    assert obj.phase == 'FAULT'


def test_rejected_start_cancels_accepted_peer():
    obj = pair()
    start(obj)
    obj.futures['leader'].set_result(NS(success=True))
    obj.futures['follower'].set_result(NS(success=False, message='busy'))
    obj._tick()
    assert obj.phase == 'FAULT'
    for client in obj.cancel_clients.values():
        client.call_async.assert_called_once()


def test_start_response_timeout_cancels_both():
    obj = pair()
    start(obj)
    obj.requested_at -= 6
    obj._tick()
    assert obj.phase == 'FAULT'
