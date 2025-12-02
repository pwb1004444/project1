#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile


# This program requires LEGO EV3 MicroPython v2.0 or higher.
# Click "Open user guide" on the EV3 extension tab for more information.


# Create your objects here.
ev3 = EV3Brick()

arm_motor = Motor(Port.C)
left = Motor(Port.A)
right = Motor(Port.D)
robot = DriveBase(left,right,55.5,104)

left_color = ColorSensor(Port.S3)
right_color = ColorSensor(Port.S4)
object_detector = ColorSensor(Port.S1)
ultra_sensor = UltrasonicSensor(Port.S2)



DRIVE_SPEED = 100
PROPORTIONAL_GAIN = 1.2

# while True:
#     deviation = line_sensor.reflection() - threshold
#     turn_rate = PROPORTIONAL_GAIN * deviation
#     robot.drive(DRIVE_SPEED, turn_rate)
#     wait(10)


threshold = 50
kp =1.3

N, E, S, W = 1, 2, 3, 4


def left_line_following(speed, kp):
    left_reflection = left_color.reflection()
    error = left_reflection - threshold
    turn_rate = kp*error
    robot.drive(speed, turn_rate)

def right_line_following(speed, kp):
    right_reflection = right_color.reflection()
    error = right_reflection - threshold
    turn_rate = kp*error
    robot.drive(speed, turn_rate)

def follow_line_one_cell(direction='left'):
        if direction == 'right':
            while True:
                left_reflection = left_color.reflection()
                right_reflection = right_color.reflection()
                if left_reflection > 40:
                    print('detect')
                    robot.stop()
                    break
                else:
                    error=right_reflection - threshold
                    turn_rate = kp*error
                    robot.drive(DRIVE_SPEED, -turn_rate)
                
            while True:
                left_reflection = left_color.reflection()
                right_reflection = right_color.reflection()
                if left_reflection < 40:
                    print('detect')
                    robot.stop()
                    break
                else:
                    error=right_reflection - threshold
                    turn_rate = kp*error
                    robot.drive(DRIVE_SPEED, -turn_rate)
        
            
        elif direction == 'left':
            while True:
                left_reflection = left_color.reflection()
                right_reflection = right_color.reflection()
                if right_reflection > 40:
                    robot.stop()
                    break
                else:
                    error=left_reflection - threshold
                    turn_rate = kp*error
                    robot.drive(DRIVE_SPEED, turn_rate)
                
            while True:
                left_reflection = left_color.reflection()
                right_reflection = right_color.reflection()
                if right_reflection < 40:
                    robot.stop()
                    break
                else:
                    error=left_reflection - threshold
                    turn_rate = kp*error
                    robot.drive(DRIVE_SPEED, turn_rate)
            
            

def release_object():
    arm_motor.run_until_stalled(-200, then=Stop.COAST, duty_limit=50)

def grab_object():
    arm_motor.run_until_stalled(200, then=Stop.COAST, duty_limit=50)

def turn_min(now_dir, target_dir):
    diff = (target_dir - now_dir) % 4
    angle = [0, 90, -180, -90][diff]
    robot.turn(angle)
    return target_dir

def move_manhattan(start_xy, goal_xy, now_dir):
    x, y = start_xy
    gx, gy = goal_xy
    dx = gx-x
    dy = gy-y
    direction = 'left'

    if dx != 0:
        if dx < 0:
            direction = 'right'
        target_dir = E if dx > 0 else W
        robot.straight(30)
        now_dir = turn_min(now_dir, target_dir)
        wait(100)
        steps = abs(dx)
        for _ in range(steps):
            follow_line_one_cell(direction)
            x += 1 if target_dir == E else -1

    if dy != 0:
        if dy < 0:
            direction = 'left'
        target_dir = N if dy > 0 else S
        robot.straight(30)
        now_dir = turn_min(now_dir, target_dir)
        wait(100)
        steps = abs(dy)
        for _ in range(steps):
            follow_line_one_cell()
            y += 1 if target_dir == N else -1

    return (x, y), now_dir

from collections import deque

# 좌표계: (x,y)
# x: 왼쪽 → 오른쪽 증가
# y: 아래 → 위 증가
#
# 맵 정의 (문자열 상단이 y=H-1)
MAP = [
    ".MM...",   # y = 2
    "..#...",   # y = 1
    "..#...",   # y = 0
]

H = len(MAP)
W = len(MAP[0])

# 문자 맵을 장애물 맵으로 변환 (#=1, 나머지=0)
G = [[1 if MAP[H-1-y][x] == "#" else 0 for x in range(W)] for y in range(H)]

# 필수 지점(M) 찾기 → 첫 번째 M만 경유
mandatory_points = []
for y in range(H):
    for x in range(W):
        if MAP[H-1-y][x] == "M":
            mandatory_points.append((x, y))

if not mandatory_points:
    raise ValueError("MAP 내에 M 지점이 없습니다!")



def bfs(start, goal):
    if G[start[1]][start[0]] == 1 or G[goal[1]][goal[0]] == 1:
        return [], None

    dist = [[None] * W for _ in range(H)]
    prev = [[None] * W for _ in range(H)]
    q = deque([start])
    dist[start[1]][start[0]] = 0

    while q:
        x, y = q.popleft()
        if (x, y) == goal:
            break

        for dx, dy in ((1,0),(-1,0),(0,1),(0,-1)):
            nx, ny = x+dx, y+dy
            if 0 <= nx < W and 0 <= ny < H:
                if G[ny][nx] == 0 and dist[ny][nx] is None:
                    dist[ny][nx] = dist[y][x] + 1
                    prev[ny][nx] = (x, y)
                    q.append((nx, ny))

    if dist[goal[1]][goal[0]] is None:
        return [], dist

    path = []
    cur = goal
    while cur:
        path.append(cur)
        cur = prev[cur[1]][cur[0]]
    path.reverse()
    return path, dist


def show(path, dist):
    grid = [list(r) for r in MAP]

    for x, y in path:
        ry = H-1-y
        if (x, y) not in (S, E, M):
            grid[ry][x] = '*'

    grid[H-1-S[1]][S[0]] = 'S'
    grid[H-1-E[1]][E[0]] = 'G'
    grid[H-1-M[1]][M[0]] = 'M'

    print("=== 경로 맵 ===")
    for row in grid:
        print("".join(row))

    print("\n=== 거리 히트맵 (S→M 기준) ===")
    for y in reversed(range(H)):
        row_str = ""
        for d in dist[y]:
            # None 또는 숫자 1자리일 때 패딩을 직접 적용
            if d is None:
                row_str += " . "
            else:
                s = str(d)
                if len(s) < 2:
                    s = " " + s
                row_str += s + " "
        print(row_str)

    print("\n경로:", path)
    if path:
        print("길이:", len(path)-1)

def bfs_model(S, E):
    M = (1, 2)
    path1, dist1 = bfs(S, M)
    path2, dist2 = bfs(M, E)
    return path1+path2[1:]



while True:

    if any(ev3.buttons.pressed()):
        print(object_detector.color())
        release_object()
        robot.straight(100)
        while right_color.reflection() > 30:
            left_line_following(100, 1.2)
        robot.stop()

        start = (1, 2)
        end = (4, 0)
        path = bfs_model(start, end)  # 올바른 순서 유지
        now_xy = path[0]              # start와 동일
        (fx, fy), now_dir = move_manhattan(start, now_xy, E)                 # 초기 방향
        for next_xy in path[1:]:      # 두 번째부터 목적지까지
            print("이동 →", next_xy)
            (fx, fy), now_dir = move_manhattan((fx, fy), next_xy, now_dir)
        end = (1, 0)
        path = bfs_model((fx, fy), end)  # 올바른 순서 유지
        now_xy = path[0]              # start와 동일
        (fx, fy), now_dir = move_manhattan(start, now_xy, E)                 # 초기 방향
        for next_xy in path[1:]:      # 두 번째부터 목적지까지
            print("이동 →", next_xy)
            (fx, fy), now_dir = move_manhattan((fx, fy), next_xy, now_dir)

        # print(object_detector.color())
        # release_object()
        # robot.straight(100)
        # while right_color.reflection() > 30:
        #     left_line_following(100, 1.2)
        # robot.stop()

        # start = [1, 2]
        # while ultra_sensor.distance() > 40:
        #     left_line_following(100, 1.2)
        #     color = object_detector.color()

        # color = object_detector.color()
        # robot.stop()
        # robot.straight(50)
        # grab_object()
            

        # robot.straight(60)
        # start[0] += 1

        # if color == Color.RED:
        #     target_pos = (0, 1)
        #     (fx, fy), now_dir = move_manhattan(start, (1,1), E)
        #     move_manhattan((fx, fy), target_pos, now_dir)

        # else:
        #     target_pos = (0, 0)
        #     (fx, fy), now_dir = move_manhattan(start, (1,0), E)
        #     print((fx, fy), now_dir)
        #     (fx, fy), now_dir = move_manhattan((fx, fy), target_pos, now_dir)
        #     print((fx, fy), now_dir)

        # release_object()
        # wait(500)