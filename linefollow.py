#!/usr/bin/env pybricks-micropython
from pybricks.ev3devices import Motor, ColorSensor
from pybricks.parameters import Port
from pybricks.robotics import DriveBase
from pybricks.tools import wait

left_motor = Motor(Port.B)
right_motor = Motor(Port.C)

light_sensor = ColorSensor(Port.S3)

robot = DriveBase(left_motor, right_motor, wheel_diameter=56, axle_track=152)

white_threshold = 70
black_threshold = 25
target_reflection = (white_threshold + black_threshold) / 2

speed = 40
max_turn_rate = 120
kp = 1.95

def line_follow():
    while True:
        reflection = light_sensor.reflection()
        error = reflection - target_reflection
        steering = -kp * error
        steering = max(-max_turn_rate, min(max_turn_rate, steering))
        robot.drive(speed, steering)
        wait(20)
line_follow()
        