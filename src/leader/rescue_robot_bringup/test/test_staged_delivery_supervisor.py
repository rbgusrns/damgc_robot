"""The staged entry must keep delivery out of mapping and start RGB on the host."""
import importlib.util
import io
import json
from pathlib import Path
import sys
from types import SimpleNamespace as NS

ROOT=Path(__file__).resolve().parents[4]


def test_post_return_components_and_host_rgb_are_started_in_order(tmp_path,monkeypatch):
    monkeypatch.syspath_prepend(str(ROOT/'scripts'))
    spec=importlib.util.spec_from_file_location('staged_supervisor',ROOT/'scripts/run_staged_solo_delivery.py')
    stage=importlib.util.module_from_spec(spec)
    # Importing a supervisor in a test must not replace pytest's signal handlers.
    import signal
    with monkeypatch.context() as patch:
        patch.setattr(signal,'signal',lambda *args:None)
        spec.loader.exec_module(stage)
    log_dir=tmp_path/'vslam_mapping_fake';log_dir.mkdir()
    calls=[];events=[];ticks=[0]
    class Process:
        def __init__(self,args,**kwargs):
            self.pid=100000+len(calls);self.args=args
            calls.append((args,kwargs));self.stdout=io.StringIO('Logs: '+str(log_dir)+'\n')
            if 'solo_grasp_host.py' in ' '.join(args):
                assert (log_dir/'return_context.json').exists();events.append('grasp_host')
            if 'realsense2_camera' in ' '.join(args):
                assert (log_dir/'rgb_camera_requested').exists()
                events.append('host_rgb');kwargs['stdout'].write('RealSense Node Is Up!\n')
        def poll(self):return None
        def wait(self,timeout=None):return 0
    class Thread:
        def __init__(self,target,**kwargs):self.target=target
        def start(self):self.target()
    def sleep(seconds):
        ticks[0]+=1
        if ticks[0]==1:
            assert len(calls)==1
            (log_dir/'return_context.json').write_text(json.dumps({'returned':True}))
        elif ticks[0]==2:
            assert events==['grasp_host']
            (log_dir/'vision_stop_requested').touch();(log_dir/'rgb_camera_requested').touch()
        elif ticks[0]==3:
            assert (log_dir/'rgb_camera_ready').exists()
            (log_dir/'solo_delivery_result.json').write_text('{}');(log_dir/'post_return_finished').touch()
        elif ticks[0]==4:stage.stop.set()
    monkeypatch.setattr(stage.subprocess,'Popen',Process)
    monkeypatch.setattr(stage.subprocess,'run',lambda *a,**k:NS(returncode=0))
    monkeypatch.setattr(stage.threading,'Thread',Thread)
    monkeypatch.setattr(stage.time,'sleep',sleep)
    monkeypatch.setattr(stage.os,'killpg',lambda *args:None)
    stage.main()
    assert events==['grasp_host','host_rgb']
    assert len(calls)==5 # original mapping, grasp host, relay, coordinator, host RGB
    mapping_env=calls[0][1]['env']
    assert mapping_env['MAPPING_CAMERA_FPS']=='30'
    assert mapping_env['MAPPING_POST_RETURN']=='1'
    assert 'MAPPING_SOLO_DELIVERY' not in mapping_env
    coordinator=next(args for args,_ in calls if 'scripts/post_return_delivery.py ' in ' '.join(args))
    assert coordinator[0]=='docker' and '--side-delivery' in coordinator[-1]
    rgb=next(args for args,_ in calls if 'realsense2_camera' in ' '.join(args))
    assert rgb[:2]==['bash','-lc'] # RealSense package exists on host, not mapping container.
