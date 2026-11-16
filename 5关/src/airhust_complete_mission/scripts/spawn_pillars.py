import numpy as np
import random
import math
import matplotlib.pyplot as plt
from matplotlib.patches import Circle

R_outside = 1.6
R_inside = 0.8
p_R = 0.6
p_r = 1.2
p_xy = 0.5
vel_track_max = 0.25
vel_collision_max = 0.30
vel_sp_max = 0.30
dt = 0.05
max_x = 6.0
min_x = 0.0
max_y = 6.0
min_y = 0.0
spawn_x = 3.0
spawn_y = -1.0
uav_r = 0.2
target_x = 3.0
target_y = 6.5
pillar_r = 0.1
pillar = []
pillar_num = 10
route_x = [1.6, 2.3, 3.0, 3.7, 4.4]
wall = [((0.0, 0.0), (0.0, 6.0)), ((6.0, 0.0), (6.0, 6.0))]

def check_pillar(x, y):
    if x < min_x + 0.3 + pillar_r or x > max_x - 0.3 - pillar_r:
        return False
    for px, py in pillar:
        d = math.sqrt((px - x)**2 + (py - y)**2)
        if d < 1.0:
            return False
        if d - 2 * pillar_r < R_inside * 2 and abs(py - y) < 1.2 and any((px - gx) * (x - gx) <= 0 for gx in route_x):
            return False
    for sx in route_x:
        if math.sqrt((sx - x)**2 + (spawn_y - y)**2) < 1.0:
            return False
        if math.sqrt((sx - x)**2 + (target_y - y)**2) < 1.0:
            return False
    return True

def spawn_pillars():
    pillar.clear()
    n = 0
    tries = 0
    while n < pillar_num and tries < 5000:
        tries += 1
        x = random.uniform(min_x, max_x)
        y = random.uniform(min_y, max_y)
        if check_pillar(x, y):
            pillar.append((x, y))
            n += 1
    return n == pillar_num

def get_nearest_collision_distance_and_angle(local_position_x, local_position_y):
    collision_distance = 100.0
    collision_angle = 0.0
    for px, py in pillar:
        d = math.sqrt((local_position_x - px)**2 + (local_position_y - py)**2)
        if d - pillar_r < collision_distance:
            collision_distance = d - pillar_r
            collision_angle = np.arctan2(local_position_y - py, local_position_x - px)
    for (ax, ay), (bx, by) in wall:
        dx = bx - ax
        dy = by - ay
        t = ((local_position_x - ax) * dx + (local_position_y - ay) * dy) / (dx * dx + dy * dy)
        t = max(0.0, min(1.0, t))
        cx = ax + t * dx
        cy = ay + t * dy
        d = math.sqrt((local_position_x - cx)**2 + (local_position_y - cy)**2)
        if d < collision_distance:
            collision_distance = d
            collision_angle = np.arctan2(local_position_y - cy, local_position_x - cx)
    return collision_distance, collision_angle

def crashed(local_position_x, local_position_y):
    collision_distance, collision_angle = get_nearest_collision_distance_and_angle(local_position_x, local_position_y)
    return collision_distance < uav_r

def APF_collision_avoidance(start_x, goal_x):
    local_position_x, local_position_y = start_x, spawn_y
    path = [(local_position_x, local_position_y)]
    stalls = 0
    for _ in range(30000):
        if math.sqrt((local_position_x-goal_x)**2 + (local_position_y-target_y)**2) <= 0.1:
            return path
        if crashed(local_position_x, local_position_y):
            return None
        distance_c, angle_c = get_nearest_collision_distance_and_angle(local_position_x, local_position_y)
        vel_track_x = p_xy * (goal_x - local_position_x)
        vel_track_y = p_xy * (target_y - local_position_y)
        vel_max = max(abs(vel_track_x), abs(vel_track_y))
        if vel_max > vel_track_max:
            vel_track_x = vel_track_x * vel_track_max / vel_max
            vel_track_y = vel_track_y * vel_track_max / vel_max
        vel_collision_x = 0.0
        vel_collision_y = 0.0
        if distance_c < R_outside:
            if distance_c > R_inside:
                F_c = p_R * (R_outside - distance_c)
            else:
                F_c = p_R * (R_outside - R_inside) + p_r * (R_inside - distance_c)
            vel_collision_x = F_c * math.cos(angle_c)
            vel_collision_y = F_c * math.sin(angle_c)
            vel_max = max(abs(vel_collision_x), abs(vel_collision_y))
            if vel_max > vel_collision_max:
                vel_collision_x = vel_collision_x * vel_collision_max / vel_max
                vel_collision_y = vel_collision_y * vel_collision_max / vel_max
            vel_collision_x += random.uniform(-0.05, 0.05)
            vel_collision_y += random.uniform(-0.05, 0.05)
        vel_x = vel_track_x + vel_collision_x
        vel_y = vel_track_y + vel_collision_y
        vel_max = max(abs(vel_x), abs(vel_y))
        if vel_max > vel_sp_max:
            vel_x = vel_x * vel_sp_max / vel_max
            vel_y = vel_y * vel_sp_max / vel_max
        prev_x, prev_y = local_position_x, local_position_y
        local_position_x += vel_x * dt
        local_position_y += vel_y * dt
        if math.sqrt((local_position_x-prev_x)**2 + (local_position_y-prev_y)**2) < 1e-5:
            stalls += 1
            if stalls > 300:
                return None
        else:
            stalls = 0
        path.append((local_position_x, local_position_y))
    return None

def print_map(paths):
    fig, ax = plt.subplots()
    for sx in route_x:
        ax.add_patch(Circle((sx, spawn_y), 0.1, color='r'))
        ax.add_patch(Circle((sx, target_y), 0.1, color='g'))
    for px, py in pillar:
        ax.add_patch(Circle((px, py), pillar_r, color='b'))
    for (ax0, ay0), (ax1, ay1) in wall:
        ax.plot([ax0, ax1], [ay0, ay1], color='k', linewidth=4)
    for path in paths:
        ax.plot([px for px, py in path], [py for px, py in path], linewidth=1)
    ax.set_aspect('equal')
    ax.set_xlim(min_x - 0.5, max_x + 0.5)
    ax.set_ylim(min(spawn_y, min_y) - 0.5, max(target_y, max_y) + 0.5)
    plt.show()


def main():
    paths = None
    for attempt in range(1, 2001):
        if not spawn_pillars():
            continue
        paths = []
        for sx in route_x:
            p = APF_collision_avoidance(sx, sx)
            if p is None:
                paths = None
                break
            paths.append(p)
        if paths is not None:
            print("成功，尝试次数：%d" % attempt)
            break
    if paths is None:
        print("失败，2000 次尝试均未找到可通行阵列")
        return
    print_map(paths)

if __name__ == "__main__":
    main()
