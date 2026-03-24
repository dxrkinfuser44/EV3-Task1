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
def old():
    while True:
        if light_sensor.reflection() < 50:
            robot.drive(60, -120)
        else:
            if light_sensor.reflection() > 50:
                robot.drive(60, 120)
            else:
                robot.drive(70,0)

def main(base_speed=150, kp=1.9, ki=0, kd=1.5, target=45):
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
        speed = base_speed * math.exp(-0.07 * abs(error)) * -1
        robot.drive(speed, turn_rate)
        last_error = error

main()