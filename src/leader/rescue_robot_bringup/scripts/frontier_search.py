"""ROS-independent frontier extraction on an ESDF slice."""
from collections import deque
import math

CARDINAL = ((-1, 0), (1, 0), (0, -1), (0, 1))
NEIGHBORS = tuple((r, c) for r in (-1, 0, 1) for c in (-1, 0, 1) if r or c)


def short_path_goal(poses, robot, maximum_distance):
    """Select a step by path length, retaining the planner's body orientation."""
    poses = list(poses)
    if (not poses or not math.isfinite(maximum_distance) or maximum_distance <= 0
            or not all(math.isfinite(value) for pose in poses for value in pose)):
        raise ValueError('A finite path and positive step distance are required')
    previous = (robot[0], robot[1], poses[0][2])
    remaining = maximum_distance
    for x, y, yaw in poses:
        distance = math.hypot(x-previous[0], y-previous[1])
        if distance > remaining and distance > 1e-9:
            ratio = remaining/distance
            turn = math.atan2(math.sin(yaw-previous[2]), math.cos(yaw-previous[2]))
            heading = previous[2]+ratio*turn
            return (previous[0]+ratio*(x-previous[0]),
                    previous[1]+ratio*(y-previous[1]), heading, False)
        remaining -= distance
        previous = (x, y, yaw)
    return (*previous, True)


def frontier_clusters(msg, robot, center, radius, clearance, minimum_distance,
                      minimum_cluster, width=0., height=0., offset_x=0., offset_y=0., include_observed=False, connection_clearance=None, minimum_unknown_area=0., stats=None, area_yaw=0.):
    """Return reachable free boundary cells; never connect diagonally through walls.

    Rectangle coordinates follow the slice frame, relative to the initial pose.
    Bounds constrain observation candidates. Reachability may pass outside them.
    """
    if stats is not None:
        stats.update(reachable_cells=0, eligible_cells=0)
    w, h, res = int(msg.width), int(msg.height), float(msg.resolution)
    if w < 3 or h < 3 or not math.isfinite(res) or res <= 0 or len(msg.data) != w*h:
        return []
    data = [float(v) for v in msg.data]
    sentinel = float(msg.unknown_value)
    def unknown(v):
        return v == sentinel or not math.isfinite(v)
    def xy(r, c):
        return msg.origin.x+(c+.5)*res, msg.origin.y+(r+.5)*res
    def inside(x, y):
        if radius > 0 and math.hypot(x-center[0], y-center[1]) > radius:
            return False
        if not width:
            return True
        dx, dy = x-center[0], y-center[1]
        local_x = math.cos(area_yaw)*dx + math.sin(area_yaw)*dy
        local_y = -math.sin(area_yaw)*dx + math.cos(area_yaw)*dy
        return (abs(local_x-offset_x) <= width/2 and
                abs(local_y-offset_y) <= height/2)
    traversal_clearance = clearance if connection_clearance is None else connection_clearance
    free = set()
    for r in range(h):
        for c in range(w):
            v = data[r*w+c]
            if not unknown(v) and v >= traversal_clearance:
                free.add((r, c))
    seed = (math.floor((robot[1]-msg.origin.y)/res),
            math.floor((robot[0]-msg.origin.x)/res))
    if seed not in free:
        # D435 cannot observe the chassis underneath itself. Bridge only this
        # short blind spot, never a known obstacle; Nav2 still validates a path.
        def visible_seed(cell):
            x, y = xy(*cell)
            distance = math.hypot(x-robot[0], y-robot[1])
            if distance > 0.40:
                return False
            steps = max(1, math.ceil(distance/(res/2)))
            for i in range(steps+1):
                px = robot[0] + (x-robot[0])*i/steps
                py = robot[1] + (y-robot[1])*i/steps
                rr = math.floor((py-msg.origin.y)/res)
                cc = math.floor((px-msg.origin.x)/res)
                if 0 <= rr < h and 0 <= cc < w:
                    value = data[rr*w+cc]
                    if not unknown(value) and value < traversal_clearance:
                        return False
            return True
        nearby = [cell for cell in free if visible_seed(cell)]
        if not nearby:
            return []
        seed = min(nearby, key=lambda cell: math.hypot(
            xy(*cell)[0]-robot[0], xy(*cell)[1]-robot[1]))
    reached = {seed}
    queue = deque([seed])
    while queue:
        r, c = queue.popleft()
        for dr, dc in CARDINAL:
            cell = r+dr, c+dc
            if cell in free and cell not in reached:
                reached.add(cell)
                queue.append(cell)
    if stats is not None:
        stats.update(reachable_cells=len(reached),
                     eligible_cells=sum(data[r*w+c] >= clearance and inside(*xy(r, c))
                                        for r, c in reached))
    relevant_unknown = None
    if minimum_unknown_area > 0:
        relevant_unknown = set()
        unseen_unknown = {(r, c) for r in range(h) for c in range(w)
                          if unknown(data[r*w+c])}
        while unseen_unknown:
            seed = unseen_unknown.pop()
            cells = [seed]
            queue = deque([seed])
            while queue:
                r, c = queue.popleft()
                for dr, dc in CARDINAL:
                    cell = r+dr, c+dc
                    if cell in unseen_unknown:
                        unseen_unknown.remove(cell)
                        cells.append(cell)
                        queue.append(cell)
            if len(cells)*res*res >= minimum_unknown_area:
                relevant_unknown.update(cells)
    candidates = {}
    for r, c in reached:
        if not (0 < r < h-1 and 0 < c < w-1):
            continue
        x, y = xy(r, c)
        if not inside(x, y):
            continue
        if data[r*w+c] < clearance:
            continue
        if math.hypot(x-robot[0], y-robot[1]) < minimum_distance:
            continue
        points = [xy(r+dr, c+dc) for dr, dc in NEIGHBORS
                  if unknown(data[(r+dr)*w+c+dc]) and inside(*xy(r+dr, c+dc))
                  and (relevant_unknown is None or (r+dr, c+dc) in relevant_unknown)]
        if not points and include_observed:
            points = [(x, y)]
        if points:
            candidates[r, c] = (x, y, data[r*w+c],
                                sum(p[0] for p in points)/len(points),
                                sum(p[1] for p in points)/len(points))
    clusters = []
    unseen = set(candidates)
    while unseen:
        seed = min(unseen)
        unseen.remove(seed)
        queue = deque([seed])
        cluster = [candidates[seed]]
        while queue:
            r, c = queue.popleft()
            for dr, dc in NEIGHBORS:
                cell = r+dr, c+dc
                if cell in unseen:
                    unseen.remove(cell)
                    queue.append(cell)
                    cluster.append(candidates[cell])
        if len(cluster) >= minimum_cluster:
            clusters.append(cluster)
    return clusters
