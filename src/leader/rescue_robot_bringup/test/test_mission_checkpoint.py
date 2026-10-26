"""Only complete, consistent map/person bundles may be used to restart a mission."""
import importlib.util
import json
from pathlib import Path
import pytest
ROOT=Path(__file__).resolve().parents[4]
spec=importlib.util.spec_from_file_location('checkpoint',ROOT/'scripts/mission_checkpoint.py')
checkpoint=importlib.util.module_from_spec(spec);spec.loader.exec_module(checkpoint)


def bundle(directory):
    manifest=dict(mission_checkpoint_version=1,map_frame='odom',return_context=dict(returned=True,map_frame='odom'),initial_state=[0.]*15,nvblox_file='map.nvblx',map_bytes=3,survivors_file='survivors.json',navigation_slice_file='navigation_slice.json',survivor_count=1)
    p=dict(x=1.,y=-1.,z=.5)
    people=dict(header=dict(frame_id='odom'),tracks=[dict(id=7,raw_position=p,filtered_position=p)])
    grid=dict(header=dict(frame_id='odom'),width=2,height=2,resolution=.05,data=[1.]*4)
    (directory/'map.nvblx').write_bytes(b'map')
    for filename,value in [('manifest.json',manifest),('survivors.json',people),('navigation_slice.json',grid)]:
        (directory/filename).write_text(json.dumps(value))
    return manifest,people,grid


def test_restart_retains_person_ids_coordinates_and_anchor(tmp_path):
    expected=bundle(tmp_path)
    assert checkpoint.load_checkpoint(tmp_path)==expected


@pytest.mark.parametrize('damage',['map','person_frame','nan_anchor','missing_registry','partial_grid','path_escape'])
def test_rejects_incomplete_or_inconsistent_checkpoint(tmp_path,damage):
    m,p,g=bundle(tmp_path)
    if damage=='map':(tmp_path/'map.nvblx').write_bytes(b'')
    elif damage=='person_frame':p['header']['frame_id']='map'
    elif damage=='nan_anchor':m['initial_state'][0]=float('nan')
    elif damage=='missing_registry':m['survivor_count']=2
    elif damage=='partial_grid':g['data']=[]
    elif damage=='path_escape':m['survivors_file']='../survivors.json'
    for filename,value in [('manifest.json',m),('survivors.json',p),('navigation_slice.json',g)]:
        (tmp_path/filename).write_text(json.dumps(value))
    with pytest.raises((ValueError,FileNotFoundError)):checkpoint.load_checkpoint(tmp_path)
