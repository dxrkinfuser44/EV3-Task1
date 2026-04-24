#!/usr/bin/env pybricks-micropython

# Import necessary libraries
from tracemalloc import start

from pybricks.ev3devices import ColorSensor, Motor, UltrasonicSensor
from pybricks.parameters import Color, Port
from pybricks.robotics import DriveBase
from pybricks.tools import wait

# ---------- STANDARD SETUP CODE ---------- #

# Initialize the motors.
left_motor = Motor(Port.B)
right_motor = Motor(Port.C)

# Initialise the sensors.
light_sensor = ColorSensor(Port.S3)
distance_sensor = UltrasonicSensor(Port.S4)

# Initialize the drive base.
robot = DriveBase(left_motor, right_motor, wheel_diameter=56, axle_track=152)

# ---------- MAINLINE PROGRAM ---------- #


### Here is where your code starts ###
def base():
    robot.drive(200, 0)
    wait(1000)
    robot.stop()
    wait(1000)
    robot.drive(-200, 0)
    wait(1000)
    robot.stop()


# variable definition
green = Color.GREEN
red = Color.RED
white = Color.WHITE
black = Color.BLACK
white_count = 0


def starter():
    while white_count < 1:
        robot.drive(200, 0)
        if light_sensor.color() == white:
            white_count += 1


def color_check():
    while True:
        if light_sensor.color() == white:
            robot.drive(200, 0)
        elif light_sensor.color() == black:
            robot.drive(200, -40)
        elif light_sensor.color() == green:
            robot.straight(50)
            robot.turn(90)
            robot.drive(200, 0)
        elif light_sensor.color() == red:
            robot.drive(200, 0)
            wait(2000)
            robot.turn(90)
            robot.drive(200, 40)


def main():
    starter()
    color_check()
