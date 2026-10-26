"""Validate a complete map/person checkpoint before starting any robot process."""
import json
import math
from pathlib import Path


def load_checkpoint(directory):
    directory=Path(directory).resolve()
    manifest=json.loads((directory/'manifest.json').read_text())
    if manifest.get('mission_checkpoint_version')!=1 or manifest.get('map_frame')!='odom':
        raise ValueError('Not an odom-frame mission checkpoint')
    context=manifest.get('return_context',{})
    if not context.get('returned') or context.get('map_frame')!='odom':raise ValueError('Return was not completed')
    for name,count in (('initial_state',15),):
        values=manifest.get(name,[])
        if len(values)!=count or not all(isinstance(v,(int,float)) and math.isfinite(v) for v in values):raise ValueError('Invalid saved anchor')
    def file(key):
        path=(directory/manifest[key]).resolve()
        if path.parent!=directory or not path.is_file():raise ValueError('Invalid checkpoint file '+key)
        return path
    layers=file('nvblox_file')
    if layers.stat().st_size!=manifest.get('map_bytes') or layers.stat().st_size<=0:raise ValueError('Saved map is incomplete')
    people=json.loads(file('survivors_file').read_text())
    grid=json.loads(file('navigation_slice_file').read_text())
    if people.get('header',{}).get('frame_id')!='odom' or grid.get('header',{}).get('frame_id')!='odom':raise ValueError('Saved frame mismatch')
    tracks=people.get('tracks',[])
    if len(tracks)!=manifest.get('survivor_count') or len({t['id'] for t in tracks})!=len(tracks):raise ValueError('Invalid survivor records')
    for track in tracks:
        for key in ('raw_position','filtered_position'):
            if not all(math.isfinite(track[key][axis]) for axis in ('x','y','z')):raise ValueError('Invalid survivor position')
    if grid['width']<=0 or grid['height']<=0 or not math.isfinite(grid['resolution']) or grid['resolution']<=0 or len(grid['data'])!=grid['width']*grid['height']:
        raise ValueError('Invalid saved navigation slice')
    return manifest,people,grid
