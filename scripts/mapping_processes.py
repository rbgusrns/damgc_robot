#!/usr/bin/env python3
"""Small process-group helpers for the mapping launcher's owned sessions."""
import argparse
import json
import os
from pathlib import Path
import signal


def live_group(pgid):
    for path in Path('/proc').glob('[0-9]*/stat'):
        try:
            fields = path.read_text().rsplit(')', 1)[1].split()
            if fields[0] != 'Z' and int(fields[2]) == pgid:
                return True
        except (OSError, ValueError):
            pass
    return False


def signal_group(pgid, name):
    if pgid <= 1 or pgid == os.getpgrp():
        raise ValueError('Refusing to signal an unowned process group')
    try:
        os.killpg(pgid, getattr(signal, 'SIG'+name))
    except ProcessLookupError:
        pass


def conflicts():
    found = []
    for path in Path('/proc').glob('[0-9]*/cmdline'):
        try:
            argv = [item.decode(errors='replace') for item in path.read_bytes().split(b'\0') if item]
            if not argv:
                continue
            executable = Path(argv[0]).name
            # Python ROS entry points put the executable path in argv[1].
            if executable.startswith('python') and len(argv) > 1:
                executable = Path(argv[1]).name
            leader = '__ns:=/leader' in argv
            if ((leader and executable in ('realsense2_camera_node', 'stm32_bridge_node', 'command_selector_node'))
                    or executable in ('nvblox_node', 'isaac_ros_visual_slam')
                    or (executable == 'ros2' and 'bag' in argv and 'record' in argv
                        and any('/data/vslam_mapping_' in item for item in argv))):
                found.append({'pid': int(path.parent.name), 'node': executable})
        except (OSError, ValueError):
            pass
    return found


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('operation', choices=('signal', 'alive', 'conflicts'))
    parser.add_argument('pgid', nargs='?', type=int)
    parser.add_argument('signal', nargs='?', choices=('INT', 'TERM', 'KILL'))
    args = parser.parse_args()
    if args.operation == 'conflicts':
        found = conflicts()
        if found:
            print('Existing robot nodes detected; refusing duplicate startup: '+json.dumps(found))
            raise SystemExit(1)
    elif args.pgid is None or args.pgid <= 1:
        parser.error('A valid owned process group is required')
    elif args.operation == 'alive':
        raise SystemExit(0 if live_group(args.pgid) else 1)
    elif args.signal is None:
        parser.error('A signal is required')
    else:
        signal_group(args.pgid, args.signal)


if __name__ == '__main__':
    main()
