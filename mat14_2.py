#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, ColorSensor, UltrasonicSensor
from pybricks.parameters import Port, Color
from pybricks.robotics import DriveBase
from pybricks.tools import wait
import math

ev3 = EV3Brick()

left_motor = Motor(Port.B)
right_motor = Motor(Port.C)
light_sensor = ColorSensor(Port.S3)
#distance_sensor = UltrasonicSensor(Port.S4)

# Initialize the drive base.
robot = DriveBase(left_motor, right_motor, wheel_diameter=56, axle_track=152)
white_seen = 0

#definable variables
turning_angle = 85
forward_speed = 90
gray_turning = 105
slow_forward_speed = 40
gray_num = 50
black_num = 0

while True:
    while white_seen == 0:
         color = light_sensor.color()
         robot.drive(slow_forward_speed, 0)
         if color == Color.WHITE:
             white_seen += 1
    while white_seen > 0:
            color = light_sensor.color()
            if color == Color.BLACK:
                black_num + 1
            if color == Color.WHITE:
                robot.drive(forward_speed, turning_angle)
            elif color == Color.GREEN:
                robot.drive(forward_speed, (-1)*(turning_angle))
            # elif color == Color.BLACK and black_num == 0:
            #     robot.stop()
            #     robot.turn(-90)
            #     robot.drive(forward_speed, 0)
            # elif color == Color.BLACK and black_num > 0:
            #     robot.drive(forward_speed, 0)
            #elif color == Color.GREY:
            #    robot.drive((forward_speed + 20), gray_turning)
            # elif color == Color.BLACK:
            #     black_seen + 1
            # while black_seen > 0:
            #     if color == Color.WHITE:
            #         robot.drive(slow_forward_speed, (-1)*gray_turning)
            #     elif color == Color.BLACK:
            #         robot.drive(slow_forward_speed, 0)