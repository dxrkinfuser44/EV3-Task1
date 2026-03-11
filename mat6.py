#!/usr/bin/env pybricks-micropython

# Import necessary libraries
from pybricks.ev3devices import Motor, ColorSensor, UltrasonicSensor
from pybricks.parameters import Port
from pybricks.robotics import DriveBase
from pybricks.tools import wait
import linefollow2.py

# ---------- STANDARD SETUP CODE ---------- #

# Initialize the motors.
left_motor = Motor(Port.B)
right_motor = Motor(Port.C)

#Initialise the sensors.
light_sensor = ColorSensor(Port.S3)
#distance_sensor = UltrasonicSensor(Port.S2)

# var def
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

def findline():
    linefound = 0
    while linefound == 0:
        robot.drive(200)
        if light_sensor.reflection < 50:
            linefound + 1
            break
    
    while linefound > 1:
        