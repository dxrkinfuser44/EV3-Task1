#!/usr/bin/env pybricks-micropython
from pybricks.ev3devices import ColorSensor, Motor, UltrasonicSensor
from pybricks.parameters import Port
from pybricks.robotics import DriveBase
from pybricks.tools import wait

left_motor = Motor(Port.B)
right_motor = Motor(Port.C)
light_sensor = ColorSensor(Port.S3)
distance_sensor = UltrasonicSensor(Port.S4)
robot = DriveBase(left_motor, right_motor, wheel_diameter=56, axle_track=152)

ADJUST = 1.2
THRESHOLD = 40
SPEED = 80
TURN_RATE = 1.3
MAX_DRIVE_SPEED = 120
MAX_TURN_RATE = 85


def clamp(value, lower, upper):
    return max(lower, min(value, upper))


def read_rgb():
    return light_sensor.rgb()


def get_reflection(rgb):
    r, g, b = rgb
    return round((r + g + b) / 3)


def drive_for(duration_ms, drive_speed, drive_turn):
    robot.drive(
        clamp(drive_speed, -MAX_DRIVE_SPEED, MAX_DRIVE_SPEED),
        clamp(drive_turn, -MAX_TURN_RATE, MAX_TURN_RATE),
    )
    wait(duration_ms)
    robot.stop()


def follow_line_step(reflection):
    turn = (reflection - THRESHOLD) * TURN_RATE
    drive_speed = clamp(SPEED - abs(turn * ADJUST), 0, MAX_DRIVE_SPEED)
    robot.drive(drive_speed, clamp(turn, -MAX_TURN_RATE, MAX_TURN_RATE))
    return turn


def run_start_phase():
    while True:
        reflection = get_reflection(read_rgb())
        if reflection > 28:
            follow_line_step(reflection)
            print(reflection)
            continue
        is_black = str(light_sensor.color()) == "Color.BLACK"
        drive_for(400, 100, -90)
        if is_black:
            break


def handle_obstacle(dist):
    if dist >= 140:
        return
    robot.stop()
    drive_for(1500, 80, 35)
    drive_for(4500, 80, -25)


def handle_color_markers(r, g, b):
    if g > (r + b) / 1.2:
        drive_for(200, 50, 0)
        drive_for(300, 50, -85)
    if r > (g + b) / 1.2:
        drive_for(800, 50, 0)
        drive_for(300, 50, 85)


def run_main_phase():
    while True:
        rgb = read_rgb()
        r, g, b = rgb
        reflection = get_reflection(rgb)
        turn = reflection - THRESHOLD
        handle_obstacle(distance_sensor.distance())
        handle_color_markers(r, g, b)
        drive_speed = clamp(SPEED - abs(turn * ADJUST), 0, MAX_DRIVE_SPEED)
        robot.drive(drive_speed, clamp(turn, -MAX_TURN_RATE, MAX_TURN_RATE))
        print(turn)


def main():
    run_start_phase()
    run_main_phase()


main()
