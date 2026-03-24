#!/usr/bin/env pybricks-micropython
# Import necessary libraries
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, ColorSensor, UltrasonicSensor
from pybricks.parameters import Port, Color
from pybricks.robotics import DriveBase
from pybricks.tools import wait
import math
# ---------- STANDARD SETUP CODE ---------- #

ev3 = EV3Brick()

# Initialize the motors.
left_motor = Motor(Port.B)
right_motor = Motor(Port.C)

#Initialise the sensors.
light_sensor = ColorSensor(Port.S3)
distance_sensor = UltrasonicSensor(Port.S4)

# Initialize the drive base.
robot = DriveBase(left_motor, right_motor, wheel_diameter=56, axle_track=152)

# ---------- MAINLINE PROGRAM ---------- #

### Here is where your code starts ###
def linefollow2(base_speed=90, kp=1, ki=0, kd=1.5, target=45):
    integral = 0
    last_error = 0
    max_turn = 500  # safe limit for turn rate
    sweep_amount = 0
    sweep_direction = 1
    
    while True:
        r, g, b = light_sensor.rgb()
        color = light_sensor.color()
        #ev3.screen.print(color)
        
        # Check for color markers (break out if detected)
        if g > r and g > b and g > 100:
            return "green"
        elif r > g and r > b and r > 100:
            return "red"
        
        # Use brightness for line detection (can be based on RGB sum or specific channel)
        brightness = (r + g + b) // 3
        error = target - brightness
        integral += error
        error_change = error - last_error
        turn_rate = kp * error + ki * integral + kd * error_change
        turn_rate = int(max(-max_turn, min(max_turn, turn_rate)))
        
        # Add gentle sweep motion
        sweep_amount += sweep_direction * 2
        if sweep_amount > 15:
            sweep_direction = -1
        elif sweep_amount < -15:
            sweep_direction = 1
        
        turn_rate += sweep_amount
        
        speed = base_speed * math.exp(-0.07 * abs(error))
        robot.drive(speed, turn_rate)
        last_error = error

def colorsense(base_speed=90, kp=1, ki=0, kd=1.5, target=45):
    while True:
        if light_sensor.color() == ColorSensor.COLOR_GREEN:
            robot.drive(100, -100)
        elif light_sensor.color() == ColorSensor.COLOR_RED:
            robot.drive(100, 100)
        else:
            integral = 0
            last_error = 0
            max_turn = 500  # safe limit for turn rate
            brightness = light_sensor.reflection()
            error = target - brightness
            integral += error
            error_change = error - last_error
            turn_rate = kp * error + ki * integral + kd * error_change
            turn_rate = int(max(-max_turn, min(max_turn, turn_rate)))
            speed = base_speed * math.exp(-0.07 * abs(error))
            robot.drive(speed, turn_rate)
            last_error = error

def linefollow(base_speed=90, kp=1, ki=0, kd=1.5):
    integral = 0
    last_error = 0
    max_turn = 500
    color_error = {
        Color.BLACK: -50,
        Color.WHITE: 50,
    }
    while True:
        color = light_sensor.color()
        if color == Color.GREEN:
            robot.drive(base_speed, -100)
            integral = 0
            last_error = 0
        elif color == Color.RED:
            robot.drive(base_speed, 100)
            integral = 0
            last_error = 0
        else:
            error = color_error.get(color, 0)
            integral += error
            error_change = error - last_error
            turn_rate = kp * error + ki * integral + kd * error_change
            turn_rate = int(max(-max_turn, min(max_turn, turn_rate)))
            robot.drive(base_speed, turn_rate)
            last_error = error

linefollow()
