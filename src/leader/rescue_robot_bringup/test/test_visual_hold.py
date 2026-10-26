"""After carrying handoff, visual covariance adapter must not correct localization."""
import ast
from pathlib import Path
from types import SimpleNamespace as NS
from unittest.mock import Mock

SCRIPT=Path(__file__).parents[1]/'scripts/vslam_covariance_adapter.py'
tree=ast.parse(SCRIPT.read_text())
cls=next(n for n in tree.body if isinstance(n,ast.ClassDef))
namespace={'Node':object,'String':lambda **kwargs:NS(**kwargs)}
exec(compile(ast.Module(body=[cls],type_ignores=[]),str(SCRIPT),'exec'),namespace)
Adapter=namespace['VslamCovarianceAdapter']


def test_visual_hold_acknowledges_and_drops_all_subsequent_visual_updates():
    obj=Adapter.__new__(Adapter)
    obj._visual_held=False
    obj._source_pub=Mock()
    obj._publisher=Mock()
    obj.get_logger=Mock(return_value=Mock())
    response=obj._hold_visual(None,NS())
    assert response.success and obj._visual_held
    obj._callback(NS())
    obj._callback(NS())
    obj._publisher.publish.assert_not_called()
    assert obj._source_pub.publish.call_args.args[0].data.startswith('WHEEL_IMU_ONLY')
