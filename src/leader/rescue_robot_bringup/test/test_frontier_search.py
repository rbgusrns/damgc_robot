"""Exercise topology and bounds with synthetic ESDF slices, without ROS."""
import sys
from pathlib import Path
from types import SimpleNamespace

sys.path.insert(0, str(Path(__file__).parents[1] / 'scripts'))
from frontier_search import frontier_clusters


def grid(rows):
    return SimpleNamespace(width=len(rows[0]), height=len(rows), resolution=1.,
                           origin=SimpleNamespace(x=0., y=0.), unknown_value=-999.,
                           data=[v for row in rows for v in row])


def search(msg, robot=(1.5, 2.5), **kwargs):
    return frontier_clusters(msg, robot, (2.5, 2.5), 10., .36, .4, 1, **kwargs)


def test_wall_separates_otherwise_attractive_frontiers():
    msg = grid([[1, 1, 0, 1, -999, -999]] * 6)
    assert search(msg) == []
    msg.data[2*6+2] = 1.  # Open a corridor to the frontier island.
    assert search(msg)


def test_unknown_and_obstacles_are_not_traversable():
    msg = grid([[1, 1, -999, 1, -999, -999]] * 6)
    assert search(msg)  # Left edge of the unknown barrier is a frontier.
    assert all(p[0] < 2 for cluster in search(msg) for p in cluster)


def test_rectangle_excludes_frontiers_and_unknown_beyond_its_boundary():
    msg = grid([[1, 1, 1, 1, -999, -999]] * 6)
    assert search(msg)
    assert search(msg, width=2., height=4.) == []


def test_robot_in_unknown_does_not_jump_to_free_island():
    msg = grid([[1, -999, 1, 1, -999, -999]] * 6)
    assert search(msg) == []


def test_diagonal_contact_does_not_cross_obstacle_corner():
    msg = grid([[0]*6 for _ in range(6)])
    for r, c, v in [(1, 1, 1), (2, 2, 1), (2, 3, -999)]:
        msg.data[r*6+c] = v
    assert search(msg, robot=(1.5, 1.5)) == []


def test_invalid_map_and_nan_never_make_a_goal():
    msg = grid([[float('nan')]*6 for _ in range(6)])
    assert search(msg) == []
    msg.resolution = float('nan')
    assert search(msg) == []


def test_zero_radius_explores_without_an_artificial_boundary():
    msg = grid([[1, 1, 1, 1, -999, -999]] * 6)
    assert frontier_clusters(msg, (1.5, 2.5), (0., 0.), 0., .36, .4, 1)
    assert not frontier_clusters(msg, (1.5, 2.5), (0., 0.), 1., .36, .4, 1)


def test_camera_blind_spot_can_seed_nearby_free_space_but_not_cross_a_wall():
    msg = grid([[1, 1, 1, 1, -999, -999]] * 6)
    msg.resolution = .1
    for r in range(6):
        msg.data[r*6] = -999.
    args = (msg, (.05, .25), (.05, .25), 0., .36, .1, 1)
    assert frontier_clusters(*args)
    for r in range(6):
        msg.data[r*6+1] = 0.
    assert not frontier_clusters(*args)


def test_closed_observed_room_has_observation_points_without_frontiers():
    msg = grid([[0]*8] + [[0]+[1]*6+[0] for _ in range(6)] + [[0]*8])
    args = (msg, (2.5, 3.5), (2.5, 3.5), 0., .36, .6, 1)
    assert not frontier_clusters(*args)
    assert frontier_clusters(*args, include_observed=True)


def test_connectivity_uses_chassis_width_but_goal_keeps_clearance():
    msg = grid([[0]*8] + [[0]+[.25]*3+[.5]*3+[0] for _ in range(6)] + [[0]*8])
    args = (msg, (1.5, 3.5), (1.5, 3.5), 0., .36, .6, 1)
    assert not frontier_clusters(*args, include_observed=True)
    clusters = frontier_clusters(*args, include_observed=True, connection_clearance=.20)
    assert clusters
    assert all(p[2] >= .36 for cluster in clusters for p in cluster)


def test_tiny_unknown_hole_is_not_an_exploration_destination():
    msg = grid([[0]*8] + [[0]+[1]*6+[0] for _ in range(6)] + [[0]*8])
    msg.resolution = .1
    msg.data[4*8+4] = -999.
    args = (msg, (.25, .35), (.25, .35), 0., .36, .1, 1)
    assert frontier_clusters(*args)
    assert not frontier_clusters(*args, minimum_unknown_area=.1)
    assert frontier_clusters(*args, include_observed=True, minimum_unknown_area=.1)


def test_stats_distinguish_blocked_start_from_covered_space():
    msg = grid([[0]*8] + [[0]+[1]*6+[0] for _ in range(6)] + [[0]*8])
    stats = {}
    frontier_clusters(msg, (2.5, 3.5), (2.5, 3.5), 0., .36, .6, 1, stats=stats)
    assert stats['eligible_cells'] == 36
    assert stats['reachable_cells'] == 36
    frontier_clusters(msg, (.5, .5), (.5, .5), 0., .36, .6, 1, stats=stats)
    assert stats['reachable_cells'] == 0
