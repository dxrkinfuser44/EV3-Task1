#!/usr/bin/env pybricks-micropython
import random
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, ColorSensor, UltrasonicSensor
from pybricks.parameters import Port
from pybricks.robotics import DriveBase
from pybricks.tools import wait

# Initialize the EV3 Brick.
ev3 = EV3Brick()
simulator = 0
# Initialize the motors.
if simulator == 0:
    left_motor = Motor(Port.B)
    right_motor = Motor(Port.C)
else:
    left_motor = Motor(Port.A)
    right_motor = Motor(Port.B)

#Initialise the sensors.
if simulator == 0:
    light_sensor = ColorSensor(Port.S3)
    obstacle_sensor = UltrasonicSensor(Port.S4)
else:
    light_sensor = ColorSensor(Port.S1)
    obstacle_sensor = UltrasonicSensor(Port.S2)


# Initialize the drive base.
robot = DriveBase(left_motor, right_motor, wheel_diameter=56, axle_track=152)
robot.settings(straight_speed=200, straight_acceleration=100, turn_rate=100)

# Obstacle avoidance function
def avoid_obstacle():
    robot.stop()
    wait(100)
    
    # Turn left to go around the obstacle
    robot.turn(45)
    wait(100)
    
    # Move forward while checking if obstacle is clear
    robot.straight(300)
    wait(100)
    
    # Turn right to align back with the line
    robot.turn(-45)
    wait(100)

# Check for obstacles
def check_for_obstacles():
    """Check straight ahead for obstacles"""
    return obstacle_sensor.distance() < 25

# Here is where your code starts
while True:
    if check_for_obstacles():
        avoid_obstacle()
    else: 
        if light_sensor.reflection() < 50:
            robot.drive(60, -70)
        else:
            if light_sensor.reflection() > 50:
                robot.drive(60, 50)
            else:
                robot.drive(70,0)
