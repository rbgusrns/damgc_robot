"""Geometry for a solo delivery along a mapped, reverse-only path."""
import math


def delivery_goal(robot, person, stand_off=.7):
    if not all(math.isfinite(v) for v in (*robot[:2], *person[:2], stand_off)) or stand_off <= 0:
        raise ValueError('Invalid delivery target')
    dx, dy = person[0]-robot[0], person[1]-robot[1]
    distance = math.hypot(dx, dy)
    if distance <= stand_off:
        return None
    yaw = math.atan2(dy, dx)
    return (person[0]-stand_off*dx/distance, person[1]-stand_off*dy/distance,
            math.atan2(math.sin(yaw+math.pi), math.cos(yaw+math.pi)))


def delivery_side_goals(home, person, box_center=.35, side_clearance=.55, yaw_offsets=(0.,)):
    """Put the payload centre beside the person on either mapped side of the approach."""
    if not all(math.isfinite(v) for v in (*home[:2],*person[:2],box_center,side_clearance)) or box_center<=0 or side_clearance<=0:
        raise ValueError('Invalid side-delivery geometry')
    if math.hypot(person[0]-home[0],person[1]-home[1])<1e-6:
        raise ValueError('No approach direction')
    yaw=math.atan2(person[1]-home[1],person[0]-home[0])+math.pi
    yaw=math.atan2(math.sin(yaw),math.cos(yaw))
    goals=[]
    for offset in yaw_offsets:
        angle=math.atan2(math.sin(yaw+offset),math.cos(yaw+offset))
        c,s=math.cos(angle),math.sin(angle)
        goals.extend((person[0]-box_center*c-sign*side_clearance*s,
                      person[1]-box_center*s+sign*side_clearance*c,angle) for sign in (-1,1))
    return goals


def reverse_prefix(poses, maximum=.6):
    if len(poses) < 2 or maximum <= 0:
        raise ValueError('No reverse path')
    length = 0.
    output = [poses[0]]
    # Validate the complete planner result before executing even the first step.
    for a, b in zip(poses, poses[1:]):
        if not all(math.isfinite(v) for v in (*a, *b)):
            raise ValueError('Nonfinite path')
        dx, dy = b[0]-a[0], b[1]-a[1]
        distance = math.hypot(dx, dy)
        if distance < 1e-5:
            continue
        delta = math.atan2(math.sin(b[2]-a[2]), math.cos(b[2]-a[2]))
        yaw = a[2]+delta/2
        if dx*math.cos(yaw)+dy*math.sin(yaw) > -.1*distance:
            raise ValueError('Planner path requires forward/sideways movement; stopped')
        if length < maximum:
            fraction = min(1., (maximum-length)/distance)
            output.append((a[0]+fraction*dx, a[1]+fraction*dy, yaw-delta/2+fraction*delta))
        length += distance
    if len(output) < 2:
        raise ValueError('Empty reverse path')
    return output


def footprint_known_clear(grid, poses, front=.40, half=.145, rear=.06, padding=.02, polygon=None):
    """Require every loaded-footprint cell to be known and free, including unknown borders."""
    resolution = grid['resolution']
    if resolution <= 0:
        return False
    ox, oy = grid['origin']
    width, height, data = grid['width'], grid['height'], grid['data']
    dense = []
    for a,b in zip(poses, poses[1:]):
        steps = max(1, math.ceil(math.hypot(b[0]-a[0],b[1]-a[1])/(resolution/2)))
        delta = math.atan2(math.sin(b[2]-a[2]),math.cos(b[2]-a[2]))
        dense.extend((a[0]+(b[0]-a[0])*i/steps,a[1]+(b[1]-a[1])*i/steps,a[2]+delta*i/steps)
                     for i in range(steps))
    dense.extend(poses[-1:])
    for x, y, yaw in dense:
        c, s = math.cos(yaw), math.sin(yaw)
        corners = [(x+c*fx-s*fy, y+s*fx+c*fy)
                   for fx in (-rear-padding, front+padding) for fy in (-half-padding, half+padding)]
        lo_x = math.floor((min(p[0] for p in corners)-ox)/resolution)
        hi_x = math.floor((max(p[0] for p in corners)-ox)/resolution)
        lo_y = math.floor((min(p[1] for p in corners)-oy)/resolution)
        hi_y = math.floor((max(p[1] for p in corners)-oy)/resolution)
        for iy in range(lo_y, hi_y+1):
            for ix in range(lo_x, hi_x+1):
                wx, wy = ox+(ix+.5)*resolution-x, oy+(iy+.5)*resolution-y
                bx, by = c*wx+s*wy, -s*wx+c*wy
                inside = (-rear-padding <= bx <= front+padding and abs(by) <= half+padding)
                if polygon is not None:
                    inside = all((b[0]-a[0])*(by-a[1])-(b[1]-a[1])*(bx-a[0]) >=
                                 -padding*math.hypot(b[0]-a[0],b[1]-a[1])
                                 for a,b in zip(polygon,polygon[1:]+polygon[:1]))
                if inside:
                    if not (0 <= ix < width and 0 <= iy < height):
                        return False
                    distance = data[iy*width+ix]
                    if not math.isfinite(distance) or distance == grid['unknown'] or distance <= 0:
                        return False
    return True


def reverse_command(linear, angular):
    if not math.isfinite(linear) or not math.isfinite(angular) or linear > .001:
        raise ValueError('Forward/nonfinite delivery command rejected')
    scale = min(1., .03/max(abs(linear), 1e-9), .15/max(abs(angular), 1e-9))
    return min(0.,linear)*scale, angular*scale


LOADED_FOOTPRINT = [(-.06,-.145),(.06,-.145),(.40,-.05),
                    (.40,.05),(.06,.145),(-.06,.145)]
