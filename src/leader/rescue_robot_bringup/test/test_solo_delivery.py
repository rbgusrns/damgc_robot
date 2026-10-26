"""Reverse path, survivor stand-off, unknown-space and pickup-only clearing contracts."""
import ast
import copy
import importlib.util
import math
from pathlib import Path
from types import SimpleNamespace as NS
import numpy as np
import pytest

ROOT=Path(__file__).resolve().parents[4]
spec=importlib.util.spec_from_file_location('delivery_geometry',ROOT/'scripts/delivery_geometry.py')
g=importlib.util.module_from_spec(spec);spec.loader.exec_module(g)


def test_target_stops_short_of_person_and_faces_away_from_travel():
    x,y,yaw=g.delivery_goal((0.,0.),(3.,-1.))
    assert math.isclose(math.hypot(3-x,-1-y),.7)
    assert math.cos(yaw)*x+math.sin(yaw)*y < 0
    assert g.delivery_goal((0.,0.),(.4,0.)) is None


def test_reverse_path_is_short_and_forward_cusp_is_rejected_before_first_step():
    result=g.reverse_prefix([(0.,0.,math.pi),(2.,0.,math.pi)])
    assert math.isclose(result[-1][0],.6)
    with pytest.raises(ValueError):
        g.reverse_prefix([(0.,0.,math.pi),(.5,0.,math.pi),(1.,0.,0.)])
    with pytest.raises(ValueError):
        g.reverse_prefix([(0.,0.,0.),(1.,0.,0.)])


def grid():
    return dict(width=80,height=80,resolution=.05,origin=(-2.,-2.),data=[1.]*(80*80),unknown=-1000.)


def test_loaded_footprint_rejects_unknown_and_obstacles_between_sparse_waypoints():
    m=grid();path=[(0.,0.,math.pi),(1.,0.,math.pi)]
    assert g.footprint_known_clear(m,path)
    index=40*80+50
    m['data'][index]=-1000.
    assert not g.footprint_known_clear(m,path)
    m['data'][index]=0.
    assert not g.footprint_known_clear(m,path)
    assert not g.footprint_known_clear(m,[(2.,0.,0.)])


def test_reverse_velocity_gate_rejects_forward_and_preserves_curvature():
    v,w=g.reverse_command(-.1,.2)
    assert v == -.03 and math.isclose(w/v,-2.)
    for v,w in ((.02,0.),(float('nan'),0.),(0.,float('inf'))):
        with pytest.raises(ValueError):g.reverse_command(v,w)


def test_payload_removal_does_not_clear_unknown_or_cells_outside_pickup():
    script=ROOT/'src/leader/rescue_robot_bringup/scripts/delivery_slice.py'
    tree=ast.parse(script.read_text())
    fn=next(n for n in tree.body if isinstance(n,ast.FunctionDef) and n.name=='remove_attached_payload')
    namespace={'copy':copy,'math':math,'np':np}
    exec(compile(ast.Module(body=[fn],type_ignores=[]),str(script),'exec'),namespace)
    msg=NS(width=20,height=20,resolution=.05,origin=NS(x=-.5,y=-.5),data=[1.]*400,unknown_value=-1000.)
    inside=10*20+14;unknown=10*20+15;outside=10*20+19
    # The box edge intersects a cell whose centre lies just beyond the box.
    boundary=11*20+18
    lateral_wall=13*20+18
    msg.data[boundary]=-.05;msg.data[lateral_wall]=-.05
    msg.data[inside]=0.;msg.data[unknown]=-1000.;msg.data[outside]=0.
    result=namespace['remove_attached_payload'](msg,(0.,0.,0.),.4,.05)
    assert result.data[inside] == 1.
    assert result.data[boundary] == 1.
    assert result.data[lateral_wall] == -.05
    assert result.data[unknown] == -1000.
    assert result.data[outside] == 0.
    assert msg.data[inside] == 0.


def test_side_delivery_puts_box_beside_person_on_opposite_sides():
    person=(3.,-1.)
    goals=g.delivery_side_goals((0.,0.),person)
    offsets=[]
    for x,y,yaw in goals:
        c,s=math.cos(yaw),math.sin(yaw)
        bx,by=x+.35*c,y+.35*s
        dx,dy=bx-person[0],by-person[1]
        assert math.isclose(math.hypot(dx,dy),.55)
        assert math.isclose(dx*c+dy*s,0.,abs_tol=1e-9)
        offsets.append(dx*(-s)+dy*c)
    assert offsets[0]*offsets[1]<0
    with pytest.raises(ValueError):g.delivery_side_goals(person,person)
