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
left = Motor(Port.A)
right = Motor(Port.D)
robot = DriveBase(left,right,55.5,104)
left_color = ColorSensor(Port.S3)
right_color = ColorSensor(Port.S4)

DRIVE_SPEED = 150
PROPORTIONAL_GAIN = 1.2

# while True:
#     deviation = line_sensor.reflection() - threshold
#     turn_rate = PROPORTIONAL_GAIN * deviation
#     robot.drive(DRIVE_SPEED, turn_rate)
#     wait(10)


threshold = 45
kp =1.2




def go(n):
        for i in range(n):
            while True:
                left_reflection = left_color.reflection()
                right_reflection = right_color.reflection()
                if right_reflection < 30:
                    robot.stop()
                    break
                else:
                    error=left_reflection - threshold
                    turn_rate = kp*error
                    robot.drive(100,turn_rate)
                wait(10)
            while True:
                left_reflection = left_color.reflection()
                right_reflection = right_color.reflection()
                if right_reflection > 30:
                    robot.stop()
                    break
                else:
                    error=left_reflection - threshold
                    turn_rate = kp*error
                    robot.drive(100,turn_rate)
                wait(10)

def go_F(n):
        for i in range(n):
            while True:
                left_reflection = left_color.reflection()
                right_reflection = right_color.reflection()
                if left_reflection < 30:
                    robot.stop()
                    break
                else:
                    error=right_reflection - threshold
                    turn_rate = kp*error
                    robot.drive(100,turn_rate)
                wait(10)
            while True:
                left_reflection = left_color.reflection()
                right_reflection = right_color.reflection()
                if left_reflection > 30:
                    robot.stop()
                    break
                else:
                    error=right_reflection - threshold
                    turn_rate = kp*error
                    robot.drive(100,turn_rate)
                wait(10)

def turn(nd, td):
        now_dir = nd
        target_dir = td
        ev3.speaker.beep()
        direction = (target_dir - now_dir)%4
        turn_table = [0,90,180,-90]
        angle = turn_table[direction]
        robot.turn(angle)
        return target_dir


def go_target(now_cor, target_cor, flag):
    now_dir = 1
    x = target_cor[0] - now_cor[0]
    y = target_cor[1] - now_cor[1]
    if flag == 1:
        if y > 0:
            go_F(y)
            now_dir = turn(now_dir, 1)
            
        elif y==0:
            go_F(y)
        else:
            go_F(y)
            now_dir = turn(now_dir,3)
            
        if x < 0:
            go_F(x)
            now_dir = turn(now_dir,4)
            
        elif x == 0:
            go_F(x)
        else:
            go_F(x)
            now_dir = turn(now_dir,2)
            

    else:
        if y > 0:
            go(y)
            now_dir = turn(now_dir,1)
            
        elif y==0:
            go(y)
        else:
            go(y)
            now_dir = turn(now_dir,3)
            
        if x < 0:
            go(x)
            now_dir = turn(now_dir,4)
            
        elif x == 0:
            go(x)
        else:
            go(x)
            now_dir = turn(now_dir,2)
            


        

while True:

    if any(ev3.buttons.pressed()):
        go_target((2,0), (2, 1), 1)
        wait(1000)
        robot.turn(-80)
        go_target((2,1), (0, 1), 1)
        wait(1000)
        go_target((0,1), (1, 3), 0)
        wait(1000)
        go_target((1,3), (2, 4), 0)

            