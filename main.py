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



DRIVE_SPEED = 150
PROPORTIONAL_GAIN = 1.2

# while True:
#     deviation = line_sensor.reflection() - threshold
#     turn_rate = PROPORTIONAL_GAIN * deviation
#     robot.drive(DRIVE_SPEED, turn_rate)
#     wait(10)


threshold = 50
kp =1.7

Nor, Eea, Sou, Wes = 1, 2, 3, 4


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
                distance = ultra_sensor.distance()
                if distance < 40:
                    return 'object'
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
                distance = ultra_sensor.distance()
                if distance < 40:
                    return 'object'
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
                distance = ultra_sensor.distance()
                if distance < 40:
                    return 'object'
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
                distance = ultra_sensor.distance()
                if distance < 40:
                    return 'object'
                left_reflection = left_color.reflection()
                right_reflection = right_color.reflection()
                if right_reflection < 40:
                    robot.stop()
                    break
                else:
                    error=left_reflection - threshold
                    turn_rate = kp*error
                    robot.drive(DRIVE_SPEED, turn_rate)
        return 'no'
            
            

def release_object():
    arm_motor.run_until_stalled(-200, then=Stop.COAST, duty_limit=50)

def grab_object():
    arm_motor.run_until_stalled(200, then=Stop.COAST, duty_limit=50)

def turn_min(now_dir, target_dir):
    diff = (target_dir - now_dir) % 4
    angle = [0, 95, -185, -95][diff]
    robot.turn(angle)
    return target_dir

def move_manhattan(start_xy, goal_xy, now_dir):
    x, y = start_xy
    gx, gy = goal_xy
    dx = gx-x
    dy = gy-y
    direction = 'left'

    if dx != 0:
        if y == 2:
            if dx > 0:
                direction='left'
            else:
                direction='right'
        elif y == 0:
            if dx > 0:
                direction='right'
            else:
                direction='left'

        target_dir = Eea if dx > 0 else Wes
        robot.straight(40)
        print('타겟:', target_dir)
        now_dir = turn_min(now_dir, target_dir)
        wait(100)
        steps = abs(dx)
        for _ in range(steps):
            x += 1 if target_dir == Eea else -1
            status = follow_line_one_cell(direction)
            if status == 'object':
                return (x, y), now_dir, status

    if dy != 0:
        if x == 2:
            if dy < 0:
                direction = 'right'
            else:
                direction = 'left'
        elif x == 1 or x == 4:
            if dy > 0:
                direction = 'right'
            else:
                direction = 'left'

        target_dir = Nor if dy > 0 else Sou
        robot.straight(40)
        now_dir = turn_min(now_dir, target_dir)
        wait(100)
        steps = abs(dy)
        for _ in range(steps):
            y += 1 if target_dir == Nor else -1
            status = follow_line_one_cell(direction)
            if status == 'object':
                return (x, y), now_dir, status

    return (x, y), now_dir, 'no'

from collections import deque

MAP = [
    ".........",   # 화면상 y=4
    "##.#.#.#.",   # y=3
    "...#.....",   # y=2
    "##.#.#.#.",   # y=1
    "...#.....",   # y=0  ← 화면의 맨 아래
]

H = len(MAP)
W = len(MAP[0])

# 장애물 맵 (MAP은 안 뒤집음)
# 화면 y → 좌표계 y 변환해서 저장
G = [[1 if MAP[H-1-y][x] == "#" else 0 for x in range(W)] for y in range(H)]


def bfs(s, g):
    dist = [[None] * W for _ in range(H)]
    prev = [[None] * W for _ in range(H)]

    q = deque([s])
    dist[s[1]][s[0]] = 0

    while q:
        x, y = q.popleft()
        if (x, y) == g:
            break

        for dx, dy in ((1,0),(-1,0),(0,1),(0,-1)):
            nx, ny = x + dx, y + dy
            if 0 <= nx < W and 0 <= ny < H:
                if G[ny][nx] == 0 and dist[ny][nx] is None:
                    dist[ny][nx] = dist[y][x] + 1
                    prev[ny][nx] = (x, y)
                    q.append((nx, ny))

    # 끝점 도달 못 하면 빈 경로
    if dist[g[1]][g[0]] is None:
        return [], dist

    # 경로 복원
    path = []
    cur = g
    while cur:
        path.append(cur)
        cur = prev[cur[1]][cur[0]]
    path.reverse()
    return path[::2], dist

def show(path, dist, S, E):
    # MAP 그대로 복사
    grid = [list(row) for row in MAP]  # 화면 기준

    # 경로 표시 (좌표계 → 화면 y로 변환)
    for x, y in path:
        my = (H-1) - y  # 변환
        if (x, y) not in (S, E):
            grid[my][x] = '*'

    # 시작/끝 표시
    grid[(H-1)-S[1]][S[0]] = 'S'
    grid[(H-1)-E[1]][E[0]] = 'G'

    print("=== 경로 맵 ===")
    for row in grid:
        print("".join(row))

    print("\n=== 거리 히트맵 ===")
    for y in range(H-1, -1, -1):
        row = []
        for x in range(W):
            d = dist[y][x]
            if d is None:
                row.append(" .")
            else:
                s = str(d)
                if len(s) == 1:
                    s = " " + s
                row.append(s)
        print(" ".join(row))


    print("\n경로:", path)
    print("길이:", len(path)-1 if path else "없음")

def to_blue(start, now_dir):
        end = (0, 0)
        path, dist = bfs(start, end)
        show(path, dist, start, end)
        path = [(x // 2, y // 2) for (x, y) in path] 
        print(path)
        (fx, fy), now_dir, status = move_manhattan(path[0], path[1], now_dir)
        for next_xy in path[2:]:      # 두 번째부터 목적지까지
            print("이동 →", next_xy)
            (fx, fy), now_dir, status = move_manhattan((fx, fy), next_xy, now_dir)
        release_object()
        robot.straight(150)
        robot.straight(-150)
        to_12((fx*2, fy*2), now_dir)

def to_red(start, now_dir):
        end = (0, 2)
        path, dist = bfs(start, end)
        show(path, dist, start, end)
        path = [(x // 2, y // 2) for (x, y) in path] 
        print(path)
        (fx, fy), now_dir, status = move_manhattan(path[0], path[1], now_dir)
        for next_xy in path[2:]:      # 두 번째부터 목적지까지
            print("이동 →", next_xy)
            (fx, fy), now_dir, status = move_manhattan((fx, fy), next_xy, now_dir)
        release_object()
        robot.straight(150)
        robot.straight(-150)
        to_12((fx*2, fy*2), now_dir)

def to_12(start, now_dir):
    end = (2, 4)
    path, dist = bfs(start, end)
    show(path, dist, start, end)
    path = [(x // 2, y // 2) for (x, y) in path] 
    print(path)
    (fx, fy), now_dir, status = move_manhattan(path[0], path[1], now_dir)
    for next_xy in path[2:]:      # 두 번째부터 목적지까지
        print("이동 →", next_xy)
        (fx, fy), now_dir, status = move_manhattan((fx, fy), next_xy, now_dir)
    robot.straight(50)
    turn_min(now_dir, Eea)

search_l = [(6, 4), (4, 2), (4, 0), (6, 2), (8, 4), (8, 2), (6, 0), (8, 0), (0, 2)][::-1]
block_count = 0
while True:

    if any(ev3.buttons.pressed()):
        release_object()
        robot.straight(100)
        while right_color.reflection() > 30:
            left_line_following(100, 1.5)
        robot.stop()

        start = (2, 4)
        now_dir = Eea
        end = search_l.pop()
        while len(search_l) != 0:
            flag = 0
            print("시작:", start)
            path, dist = bfs(start, end)
            show(path, dist, start, end)
            path = [(x // 2, y // 2) for (x, y) in path][::-1]

            print("direction:", now_dir)
            (fx, fy), now_dir = path.pop(), now_dir
            while len(path) != 0:
                    next_xy = path.pop() 
                    print("이동 →", next_xy)
                    (fx, fy), now_dir, status = move_manhattan((fx, fy), next_xy, now_dir)
                    if status == 'object':
                        flag = 1
                        ev3.speaker.beep()
                        robot.stop()
                        robot.straight(50)
                        grab_object()
                        color = object_detector.color()
                        print(color)
                        if color == Color.RED:    #color[0] < 5 and color[1] < 10 and color[2] > 30
                            if (fx*2, fy*2) == end:
                                ev3.speaker.beep()
                                end = search_l.pop()
                            print('현재위치:', fx, fy)
                            to_red((fx*2, fy*2), now_dir)
                            block_count += 1
                            start = (2, 4)
                            now_dir = Eea
                            break 
                        else:
                            print('현재위치:', fx, fy)
                            if (fx*2, fy*2) == end:
                                end = search_l.pop()
                            to_blue((fx*2, fy*2), now_dir)
                            block_count += 1
                            start = (2, 4)
                            now_dir = Eea
                            break

            if flag == 0:
                start = (fx*2, fy*2)
                end = search_l.pop()
        
        start = (fx*2, fy*2)
        end = (0, 4)
        print("시작:", start)
        path, dist = bfs(start, end)
        show(path, dist, start, end)
        path = [(x // 2, y // 2) for (x, y) in path][::-1]

        print("direction:", now_dir)
        (fx, fy), now_dir = path.pop(), now_dir
        while len(path) != 0:
                    next_xy = path.pop() 
                    print("이동 →", next_xy)
                    (fx, fy), now_dir, status = move_manhattan((fx, fy), next_xy, now_dir)
                