#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, ColorSensor, UltrasonicSensor
from pybricks.parameters import Port, Color
from pybricks.robotics import DriveBase
from pybricks.tools import wait
import math


# -------- Hardware Setup -------- #
ev3 = EV3Brick()

left_motor = Motor(Port.B)
right_motor = Motor(Port.C)
light_sensor = ColorSensor(Port.S3)
distance_sensor = UltrasonicSensor(Port.S4)

robot = DriveBase(left_motor, right_motor, wheel_diameter=56, axle_track=152)


# -------- Main Tuning -------- #
TARGET_REFLECTION = 45
BASE_SPEED = 90
MIN_SPEED = 35
MAX_TURN = 140

# PID (linefollow2 + mat10 style)
KP = 1.3
KI = 0.0
KD = 5.0
DERIVATIVE_BOOST_THRESHOLD = 5
DERIVATIVE_BOOST_FACTOR = 1.8

# Turn-rate based speed decay (linefollow2 style)
USE_EXP_SPEED_DECAY = True
SPEED_DECAY = 0.04

# Color turn triggers (new3 + mat14_2 ideas)
LEFT_TURN_COLORS = [Color.BLUE, Color.GREEN]
RIGHT_TURN_COLORS = [Color.RED]
TURN_ANGLE = 90
COLOR_COOLDOWN_MS = 500

# Obstacle bypass (mat10/obstacle style)
OBSTACLE_DISTANCE_MM = 130
USE_OBSTACLE_BYPASS = True


def get_brightness(sensor):
    return sensor.reflection()


def clamp(value, low, high):
    return max(low, min(high, value))


def handle_color_turn(color, color_lock):
    if color_lock:
        if color not in LEFT_TURN_COLORS and color not in RIGHT_TURN_COLORS:
            return False
        return True

    if color in LEFT_TURN_COLORS:
        ev3.speaker.beep(1000, 150)
        robot.stop()
        robot.turn(-TURN_ANGLE)
        wait(COLOR_COOLDOWN_MS)
        return True

    if color in RIGHT_TURN_COLORS:
        ev3.speaker.beep(800, 150)
        robot.stop()
        robot.turn(TURN_ANGLE)
        wait(COLOR_COOLDOWN_MS)
        return True

    return False


def bypass_obstacle():
    robot.stop()
    wait(400)

    robot.turn(-55)
    robot.drive(90, 0)
    wait(1800)

    robot.turn(55)
    robot.drive(100, 0)
    wait(1400)

    robot.turn(30)
    robot.drive(90, 0)

    # Drive forward until the line is detected again.
    while get_brightness(light_sensor) > 50:
        wait(20)

    robot.stop()
    wait(200)


def main():
    integral = 0
    last_error = 0
    color_lock = False

    while True:
        brightness = get_brightness(light_sensor)
        color = light_sensor.color()

        if USE_OBSTACLE_BYPASS and distance_sensor.distance() < OBSTACLE_DISTANCE_MM:
            bypass_obstacle()
            integral = 0
            last_error = 0
            continue

        color_lock = handle_color_turn(color, color_lock)
        if color_lock:
            continue

        error = brightness - TARGET_REFLECTION
        integral += error
        derivative = error - last_error

        if abs(derivative) > DERIVATIVE_BOOST_THRESHOLD:
            derivative *= DERIVATIVE_BOOST_FACTOR

        turn_rate = KP * error + KI * integral + KD * derivative
        turn_rate = clamp(turn_rate, -MAX_TURN, MAX_TURN)

        if USE_EXP_SPEED_DECAY:
            speed = BASE_SPEED * math.exp(-SPEED_DECAY * abs(turn_rate))
        else:
            speed = BASE_SPEED - abs(error) * 0.5

        speed = clamp(speed, MIN_SPEED, BASE_SPEED)

        robot.drive(speed, turn_rate)
        last_error = error
        wait(20)


main()