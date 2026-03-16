#!/usr/bin/env pybricks-micropython
from pybricks.ev3devices import Motor, ColorSensor, UltrasonicSensor
from pybricks.parameters import Port
from pybricks.robotics import DriveBase
from pybricks.tools import wait
import math
#import linefollow2.py as linefollow

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
    # Drive straight until black line is found
    print("Hello")
    while light_sensor.reflection() >= 30:
        
        robot.drive(200, 0)
    #linefollow.main()
    def main(base_speed=90, kp=1, ki=0, kd=1.5, target=45):
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
    main()
findline()