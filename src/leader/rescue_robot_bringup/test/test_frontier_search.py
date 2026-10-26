"""Exercise topology and bounds with synthetic ESDF slices, without ROS."""
import sys
from pathlib import Path
from types import SimpleNamespace

sys.path.insert(0, str(Path(__file__).parents[1] / 'scripts'))
from frontier_search import frontier_clusters, short_path_goal
import math
import pytest


def test_short_waypoint_follows_bent_path_not_a_straight_line():
    x, y, yaw, final = short_path_goal([(0., 0., 0.), (0., .5, 0.), (.5, .5, 0.)], (0., 0.), .6)
    assert (x, y) == pytest.approx((.1, .5))
    assert not final
    assert math.hypot(x, y) <= .6


def test_short_waypoint_keeps_reverse_path_body_heading():
    x, y, yaw, final = short_path_goal([(0., 0., math.pi), (1., 0., math.pi)], (0., 0.), .6)
    assert x == pytest.approx(.6)
    assert yaw == pytest.approx(math.pi)
    assert not final


def test_short_path_returns_actual_endpoint_and_final_flag():
    assert short_path_goal([(0., 0., 0.), (.2, 0., .3)], (0., 0.), .6) == (.2, 0., .3, True)


def test_short_path_rejects_invalid_inputs():
    for path, distance in (([], .6), ([(float('nan'), 0., 0.)], .6), ([(1., 0., 0.)], 0.)):
        with pytest.raises(ValueError):
            short_path_goal(path, (0., 0.), distance)


def grid(rows):
    return SimpleNamespace(width=len(rows[0]), height=len(rows), resolution=1.,
                           origin=SimpleNamespace(x=0., y=0.), unknown_value=-999.,
                           data=[v for row in rows for v in row])


def test_area_is_not_a_wall_for_reaching_an_interior_frontier():
    rows = [[0.]*7]+[[0.]+[1.]*5+[0.] for _ in range(5)]+[[0.]*7]
    for row in (2, 3, 4):
        rows[row][3] = 0.
    rows[3][5] = -999.
    clusters = frontier_clusters(grid(rows), (2.5, 3.5), (3.5, 3.5), 0., .36, .4, 1,
                                 width=5., height=3.)
    points = [point for cluster in clusters for point in cluster]
    assert any(point[0] > 3.5 for point in points)
    assert all(1. <= point[0] <= 6. and 2. <= point[1] <= 5. for point in points)


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


def test_forward_right_square_rotates_with_heading_and_starts_at_rear_left_corner():
    resolution = .1
    width = height = 80
    origin_x, origin_y = -4., -1.
    yaw = 3.141592653589793 / 2
    rows = []
    for r in range(height):
        row = []
        for c in range(width):
            x = origin_x + (c + .5) * resolution
            y = origin_y + (r + .5) * resolution
            local_x = y
            local_y = -x
            in_area = 0. <= local_x <= 3. and -3. <= local_y <= 0.
            row.append(1. if in_area and local_x < 2.5 else -999.)
        rows.append(row)
    msg = grid(rows)
    msg.resolution = resolution
    msg.origin = SimpleNamespace(x=origin_x, y=origin_y)

    clusters = frontier_clusters(
        msg, (0., 0.), (0., 0.), 0., .36, .4, 1,
        width=3., height=3., offset_x=1.5, offset_y=-1.5, area_yaw=yaw)

    goals = [point for cluster in clusters for point in cluster]
    assert goals
    for x, y, *_ in goals:
        local_x = y
        local_y = -x
        assert 0. <= local_x <= 3.
        assert -3. <= local_y <= 0.
    assert all(point[1] > 2.3 for point in goals)


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
