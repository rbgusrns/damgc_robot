#!/usr/bin/env python3
"""Run proven mapping untouched; create every delivery component only after return."""
import argparse
import fcntl
import json
import os
from pathlib import Path
import signal
import shlex
import subprocess
import sys
import threading
import time

from staged_delivery_runtime import signal_owned_group

ROOT=Path(__file__).resolve().parents[1]
CONTAINER=os.environ.get('ISAAC_ROS_CONTAINER',os.environ.get('ISAAC_CONTAINER_NAME','isaac_ros_dev-aarch64-container'))
stop=threading.Event()
for number in (signal.SIGINT,signal.SIGTERM):signal.signal(number,lambda *_:stop.set())


def main(checkpoint_only=False):
    env=dict(os.environ)
    for key in ('MAPPING_SOLO_DELIVERY','DAMGC_SOLO_DELIVERY','MAPPING_START_POSE_FILE','MAPPING_SNAPSHOT','MAPPING_COOPERATIVE_ON_STARTUP','MAPPING_MANUAL_ONLY'):
        env.pop(key,None)
    env['ISAAC_CONTAINER_NAME']=CONTAINER
    env['MAPPING_SOURCE_MODE']='STOP'
    env['MAPPING_POST_RETURN']='1'
    env['MAPPING_CAMERA_FPS']='30'
    env.setdefault('ROS_DOMAIN_ID','0');env['ROS_LOCALHOST_ONLY']='0'
    env['FASTDDS_BUILTIN_TRANSPORTS']='UDPv4'
    state={};children=[];files=[];started=False;vision_stopped=False;rgb_started=False
    mapping=subprocess.Popen(['bash',str(ROOT/'scripts/run_survivor_exploration.sh')],cwd=ROOT,env=env,
        stdin=sys.stdin,stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True,start_new_session=True,bufsize=1)
    def output():
        for line in mapping.stdout:
            print(line,end='',flush=True)
            if line.strip().startswith('Logs: '):state['log_dir']=Path(line.strip()[6:])
    threading.Thread(target=output,daemon=True).start()
    def container_start(command,filename,log_dir):
        output=open(log_dir/filename,'a',buffering=1);files.append(output)
        prefix='source /opt/ros/humble/setup.bash; source /workspaces/isaac_ros-dev/install_docker/setup.bash; '
        proc=subprocess.Popen(['docker','exec','-u','admin','-e','ROS_DOMAIN_ID='+env['ROS_DOMAIN_ID'],
            '-e','ROS_LOCALHOST_ONLY=0','-e','FASTDDS_BUILTIN_TRANSPORTS=UDPv4',CONTAINER,
            'setsid','--wait','bash','-lc',prefix+command],stdout=output,stderr=subprocess.STDOUT)
        children.append(proc)
    try:
        while not stop.is_set():
            if mapping.poll() is not None:
                raise RuntimeError('Mapping launcher exited; no further phase started')
            log_dir=state.get('log_dir')
            if log_dir is not None and (log_dir/'return_context.json').exists() and not started:
                context=json.loads((log_dir/'return_context.json').read_text())
                if not context.get('returned'):raise RuntimeError('Return not completed')
                started=True
                if checkpoint_only:
                    snapshot=ROOT/'data/maps'/log_dir.name
                    state['snapshot']=snapshot
                    target='/workspaces/isaac_ros-dev/data/maps/'+log_dir.name
                    context_path='/workspaces/isaac_ros-dev/log/'+log_dir.name+'/return_context.json'
                    container_start('echo $$ > '+shlex.quote('/workspaces/isaac_ros-dev/log/'+log_dir.name+'/checkpoint_save.pid')+'; exec python3 /workspaces/isaac_ros-dev/scripts/mapping_snapshot.py save '+shlex.quote(target)+' --mission-context '+shlex.quote(context_path),'checkpoint_save.log',log_dir)
                    state['snapshot_process']=children[-1]
                    print('CHECKPOINT: return complete; saving map, home and survivors BEFORE grasp',flush=True)
                    continue
                print('POST_RETURN: original mapping/return complete; starting grasp/delivery components now',flush=True)
                host_log=open(log_dir/'solo_grasp.log','a',buffering=1);files.append(host_log)
                command='source /opt/ros/humble/setup.bash; source install/setup.bash; exec python3 scripts/solo_grasp_host.py'
                children.append(subprocess.Popen(['bash','-lc',command],cwd=ROOT,env=env,
                    stdout=host_log,stderr=subprocess.STDOUT,start_new_session=True))
                prefix='/workspaces/isaac_ros-dev/log/'+log_dir.name
                relay_pid=shlex.quote(prefix+'/relay.pid');delivery_pid=shlex.quote(prefix+'/delivery.pid')
                container_start('echo $$ > '+relay_pid+'; exec python3 /workspaces/isaac_ros-dev/src/leader/rescue_robot_bringup/scripts/delivery_slice.py',
                                'delivery_slice.log',log_dir)
                container_start('echo $$ > '+delivery_pid+'; exec python3 /workspaces/isaac_ros-dev/scripts/post_return_delivery.py '+prefix+' --side-delivery',
                                'post_return.log',log_dir)
            if checkpoint_only and 'snapshot_process' in state and state['snapshot_process'].poll() is not None and 'saved_reported' not in state:
                state['saved_reported']=True
                snapshot=state['snapshot']
                if state['snapshot_process'].returncode!=0 or not (snapshot/'manifest.json').exists():
                    print('CHECKPOINT FAILED: see checkpoint_save.log; robot stays stopped; no grasp',flush=True)
                else:
                    latest=ROOT/'data/maps/latest_mission'
                    if latest.exists() and not latest.is_symlink():raise RuntimeError('latest_mission exists as a real directory; refusing to replace it')
                    temporary=latest.with_name('.latest_mission.tmp')
                    temporary.unlink(missing_ok=True);temporary.symlink_to(snapshot.name);temporary.replace(latest)
                    (log_dir/'vision_stop_requested').write_text('mission saved; stop idle detector\n')
                    print('CHECKPOINT SAVED: '+str(snapshot)+'; STOP; no grasp started',flush=True)
            if started and not vision_stopped and (log_dir/'vision_stop_requested').exists():
                name='damgc-survivor-detector-checkpoint-'+str(mapping.pid)
                subprocess.run(['docker','stop','--time','3',name],stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL)
                vision_stopped=True
                print('POST_RETURN: mapping complete; idle YOLO stopped',flush=True)
            if started and not rgb_started and (log_dir/'rgb_camera_requested').exists():
                rgb_log=open(log_dir/'grasp_rgb_camera_host.log','a',buffering=1);files.append(rgb_log)
                camera_command=('source /opt/ros/humble/setup.bash; source install/setup.bash; exec ros2 launch '
                    'realsense2_camera rs_launch.py camera_namespace:=leader camera_name:=camera '
                    'enable_color:=true enable_depth:=false enable_infra1:=false enable_infra2:=false '
                    'enable_sync:=false align_depth.enable:=false rgb_camera.color_profile:=640x480x30')
                state['camera']=subprocess.Popen(['bash','-lc',camera_command],cwd=ROOT,env=env,
                    stdout=rgb_log,stderr=subprocess.STDOUT,start_new_session=True)
                children.append(state['camera'])
                rgb_started=True
            if rgb_started and not (log_dir/'rgb_camera_ready').exists():
                camera_text=(log_dir/'grasp_rgb_camera_host.log').read_text(errors='replace')
                if state['camera'].poll() is not None:
                    raise RuntimeError('RGB camera launch exited before becoming ready; see grasp_rgb_camera_host.log')
                if 'RealSense Node Is Up' in camera_text:
                    (log_dir/'rgb_camera_ready').write_text('RGB-only RealSense started\n')
            if started and (log_dir/'post_return_finished').exists():
                result_path=log_dir/'solo_delivery_result.json'
                if result_path.exists() and 'reported' not in state:
                    print('POST_RETURN RESULT: '+result_path.read_text(),flush=True);state['reported']=True
            time.sleep(.2)
    except Exception as error:
        print('STAGED MISSION STOPPED: '+str(error),flush=True)
    finally:
        log_dir=state.get('log_dir')
        if log_dir is not None:
            for filename,expected in (('delivery.pid','post_return_delivery.py'),('relay.pid','delivery_slice.py'),
                    ('navigation.pid','reverse_delivery_nav2.launch.py'),('footprint.pid','delivery_footprint.py'),('checkpoint_save.pid','mapping_snapshot.py')):
                signal_owned_group(log_dir/filename,expected)
        for child in children:
            if child.poll() is None:
                try:os.killpg(child.pid,signal.SIGINT)
                except ProcessLookupError:pass
        if mapping.poll() is None:
            try:os.killpg(mapping.pid,signal.SIGINT)
            except ProcessLookupError:pass
            try:mapping.wait(timeout=35)
            except subprocess.TimeoutExpired:os.killpg(mapping.pid,signal.SIGTERM)
        for output in files:output.close()


if __name__=='__main__':
    lock_path=Path(os.environ.get('XDG_RUNTIME_DIR','/tmp'))/f'damgc-solo-delivery-{os.getuid()}.lock'
    with lock_path.open('w') as lock:
        try:fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
        except BlockingIOError:raise SystemExit('A staged solo mission is already running')
        parser=argparse.ArgumentParser();parser.add_argument('--checkpoint-only',action='store_true')
        args=parser.parse_args();main(checkpoint_only=args.checkpoint_only)
