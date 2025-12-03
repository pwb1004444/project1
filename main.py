#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile
from collections import deque

# Create your objects here.
ev3 = EV3Brick()

# 모터 및 센서 설정 (기존과 동일)
arm_motor = Motor(Port.C)
left = Motor(Port.A)
right = Motor(Port.D)
robot = DriveBase(left, right, 55.5, 104)

left_color = ColorSensor(Port.S3)
right_color = ColorSensor(Port.S4)
object_detector = ColorSensor(Port.S1)
ultra_sensor = UltrasonicSensor(Port.S2)

DRIVE_SPEED = 155
threshold = 50
kp = 1.3

# 방향 상수
Nor, Eea, Sou, Wes = 1, 2, 3, 4

# --- 함수 정의 ---

def left_line_following(speed, kp):
    left_reflection = left_color.reflection()
    error = left_reflection - threshold
    turn_rate = kp * error
    robot.drive(speed, turn_rate)

def right_line_following(speed, kp):
    right_reflection = right_color.reflection()
    error = right_reflection - threshold
    turn_rate = kp * error
    robot.drive(speed, turn_rate)

def follow_line_one_cell(direction='left'):
    if direction == 'right':
        while True:
            distance = ultra_sensor.distance()
            if distance < 40:
                return 'object'
            left_reflection = left_color.reflection()
            right_reflection = right_color.reflection()
            if left_reflection > 40: # 교차로 감지 (흰색/검은색 반전 주의: 원본 로직 유지)
                # 원본 코드 로직상 > 40일때 멈추는 것으로 되어있음 (흰색 감지 시 정지?)
                # 보통 검은 선은 < 40 입니다. 원본이 > 40이라면 흰색 교차로 마킹이거나 로직 확인 필요.
                # 여기서는 원본 코드를 존중하여 그대로 둡니다.
                print('detect')
                robot.stop()
                break
            else:
                error = right_reflection - threshold
                turn_rate = kp * error
                robot.drive(DRIVE_SPEED, -turn_rate)
                
        while True:
            distance = ultra_sensor.distance()
            if distance < 40:
                return 'object'
            left_reflection = left_color.reflection()
            right_reflection = right_color.reflection()
            if left_reflection < 40: # 라인 재진입/완료 확인
                print('detect')
                robot.stop()
                break
            else:
                error = right_reflection - threshold
                turn_rate = kp * error
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
                error = left_reflection - threshold
                turn_rate = kp * error
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
                error = left_reflection - threshold
                turn_rate = kp * error
                robot.drive(DRIVE_SPEED, turn_rate)
    return 'no'

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
    dx = gx - x
    dy = gy - y
    direction = 'left'

    if dx != 0:
        if y == 2:
            if dx > 0: direction = 'left'
            else: direction = 'right'
        if y == 0:
            if dx > 0: direction = 'right'
            else: direction = 'left'
        elif dx < 0:
            direction = 'right'
        
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
            if dy < 0: direction = 'right'
            else: direction = 'left'
        elif x == 1:
            if dy > 0: direction = 'right'
            else: direction = 'left'
        elif dy < 0:
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


# --- 맵 및 BFS 설정 ---
MAP = [
    ".........",   # y=4
    "##.#.#.#.",   # y=3
    "...#.....",   # y=2
    "##.#.#.#.",   # y=1
    "...#.....",   # y=0
]
H = len(MAP)
W = len(MAP[0])
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

    if dist[g[1]][g[0]] is None:
        return [], dist

    path = []
    cur = g
    while cur:
        path.append(cur)
        cur = prev[cur[1]][cur[0]]
    path.reverse()
    return path[::2], dist

def show(path, dist, S, E):
    # 디버깅용 출력 함수 (내용 생략 가능하지만 원본 유지)
    print("경로:", path)

# --- 이동 함수들 ---

def to_blue(start, now_dir):
    end = (0, 0)
    path, dist = bfs(start, end)
    show(path, dist, start, end)
    path = [(x // 2, y // 2) for (x, y) in path] 
    print(path)
    (fx, fy), now_dir, status = move_manhattan(path[0], path[1], now_dir)
    for next_xy in path[2:]:
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
    for next_xy in path[2:]:
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
    for next_xy in path[2:]:
        print("이동 →", next_xy)
        (fx, fy), now_dir, status = move_manhattan((fx, fy), next_xy, now_dir)
    
    # [수정 1] 오버슈트 방지를 위해 40에서 20으로 줄임
    robot.straight(20) 
    turn_min(now_dir, Eea)

# --- 메인 실행 루프 ---

search_l = [(6, 4), (4, 2), (6, 2), (8, 4), (8, 0), (8, 2), (6, 0), (4, 0), (0, 0), (0, 0), (0, 0)][::-1]

while True:
    if any(ev3.buttons.pressed()):
        release_object()
        robot.straight(100)
        while right_color.reflection() > 30:
            left_line_following(100, 1.2)
        robot.stop()

        start = (2, 4)
        now_dir = Eea
        end = search_l.pop()
        
        while len(search_l) != 0:
            
            # ==========================================================
            # [수정 2] 자세 재정렬 (Reset) 코드 추가
            # ==========================================================
            # 로봇이 2번째 물체부터는 약간 틀어진 상태로 출발할 수 있으므로,
            # 뒤로 가서 선을 감지해 수평을 맞추고 다시 출발시킵니다.
            print("자세 교정 중...")
            
            # 1. 흰색(>40)인 동안 계속 후진 (검은 선 < 40 만날 때까지)
            robot.drive(-100, 0)
            while left_color.reflection() > 40 and right_color.reflection() > 40:
                pass 
            robot.stop()

            # 2. 바퀴 중심을 선 위에 맞추기 위해 살짝 전진 (20~30mm)
            robot.straight(30)
            wait(200) # 잠시 안정화
            # ==========================================================

            flag = 0
            print("시작:", start)
            path, dist = bfs(start, end)
            show(path, dist, start, end)
            path = [(x // 2, y // 2) for (x, y) in path][::-1]

            print(now_dir)
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
                    
                    # 빨간색 판별 (환경에 따라 RGB 값 조건이 다를 수 있음. 기본 Color.RED 사용)
                    if color == Color.RED: 
                        if (fx*2, fy*2) == end:
                            ev3.speaker.beep()
                            end = search_l.pop()
                        print('현재위치:', fx, fy)
                        to_red((fx*2, fy*2), now_dir)
                        start = (2, 4)
                        break 
                    else:
                        print('현재위치:', fx, fy)
                        if (fx*2, fy*2) == end:
                            end = search_l.pop()
                        to_blue((fx*2, fy*2), now_dir)
                        start = (2, 4)
                        break

            if flag == 0:
                start = (fx*2, fy*2)
                end = search_l.pop()