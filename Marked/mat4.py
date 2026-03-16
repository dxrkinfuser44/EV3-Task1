#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, ColorSensor
from pybricks.parameters import Port
from pybricks.robotics import DriveBase
from pybricks.tools import wait
import math

ev3 = EV3Brick()
simulator = 0
if simulator == 1:
    left_motor = Motor(Port.A)
    right_motor = Motor(Port.B)
else:
    left_motor = Motor(Port.B)
    right_motor = Motor(Port.C)
robot = DriveBase(left_motor, right_motor, wheel_diameter=56, axle_track=152)
light_sensor = ColorSensor(Port.S3)

#def getBrightness(sensor):
#    r, g, b = sensor.rgb()
#    return (r + g + b) / 3

def followLineSingleSensor(base_speed=90, kp=1.5, ki=0, kd=1.5, target=53):
    integral = 0
    last_error = 0
    max_turn = 500  # safe limit for turn rate
    while True:
        brightness = light_sensor.reflection()
        error = target - brightness
        integral += error
        error_change = error - last_error
        turn_rate = kp * error + ki * integral + kd * error_change
        turn_rate = int(max(-max_turn, min(max_turn, turn_rate)))
        speed = base_speed * math.exp(-0.07 * abs(error))
        robot.drive(speed, turn_rate)
        last_error = error

#sim error
#while simulator == 1:
    #robot.drive (100, 5)
    #wait(2100)
    #break
followLineSingleSensor()