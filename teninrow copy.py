#!/usr/bin/env pybricks-micropython
from pybricks.ev3devices import ColorSensor, Motor, UltrasonicSensor
from pybricks.parameters import Port, Color
from pybricks.robotics import DriveBase
from pybricks.tools import wait

left_motor = Motor(Port.B)
right_motor = Motor(Port.C)
light_sensor = ColorSensor(Port.S3)
distance_sensor = UltrasonicSensor(Port.S4)
robot = DriveBase(left_motor, right_motor, wheel_diameter=56, axle_track=152)

BASE_SPEED = 75
ADJUST = 1.2
THRESHOLD = 40
TURN_RATE = 1.3
EM = 1.5
LINE_FOLLOW = -50
MIN_SPEED = 20
MAX_SPEED = 120
MAX_TURN = 85
OBSTACLE_DISTANCE = 145
CHECK_LOOPS = 3

state = {
    "r": 0,
    "g": 0,
    "b": 0,
    "reflection": 0,
    "distance": 0,
    "color": None,
}


def color_sensing():
    total_r = 0
    total_g = 0
    total_b = 0
    total_distance = 0
    green_hits = 0
    red_hits = 0
    black_hits = 0

    for _ in range(CHECK_LOOPS):
        detected_color = light_sensor.color()
        r, g, b = light_sensor.rgb()
        total_r += r
        total_g += g
        total_b += b
        total_distance += distance_sensor.distance()
        if detected_color == Color.GREEN:
            green_hits += 1
        elif detected_color == Color.RED:
            red_hits += 1
        elif detected_color == Color.BLACK:
            black_hits += 1
        wait(5)

    state["r"] = round(total_r / CHECK_LOOPS)
    state["g"] = round(total_g / CHECK_LOOPS)
    state["b"] = round(total_b / CHECK_LOOPS)
    state["distance"] = round(total_distance / CHECK_LOOPS)
    state["reflection"] = round((state["r"] + state["g"] + state["b"]) / 3)

    state["color"] = None
    if green_hits >= 2:
        state["color"] = Color.GREEN
    elif red_hits >= 2:
        state["color"] = Color.RED
    elif black_hits >= 2:
        state["color"] = Color.BLACK


def linefollow():
    error = -state["reflection"] - LINE_FOLLOW
    turn_rate_nishal = error * EM
    turn_rate_threshold = (state["reflection"] - THRESHOLD) * TURN_RATE
    turn_rate = (turn_rate_nishal + turn_rate_threshold) / 2
    speed = BASE_SPEED - abs(turn_rate) * ADJUST

    if speed < MIN_SPEED:
        speed = MIN_SPEED
    elif speed > MAX_SPEED:
        speed = MAX_SPEED

    if turn_rate > MAX_TURN:
        turn_rate = MAX_TURN
    elif turn_rate < -MAX_TURN:
        turn_rate = -MAX_TURN

    robot.drive(speed, turn_rate)


def handle_green():
    robot.drive(40, 0)
    wait(1500)
    robot.drive(30, 60)
    wait(400)
    robot.stop()
    wait(100)


def handle_red():
    robot.drive(50, 0)
    wait(900)
    robot.drive(0, -30)
    wait(1700)
    robot.stop()


def handle_obstacle():
    robot.stop()
    robot.drive(80, 40)
    wait(1500)
    robot.stop()


def start_loop():
    while True:
        color_sensing()
        if state["color"] == Color.BLACK:
            robot.drive(100, -90)
            wait(400)
            robot.stop()
            break
        linefollow()
        print("START", state["reflection"], state["color"])


def main_loop():
    start_loop()
    while True:
        color_sensing()
        if state["distance"] < OBSTACLE_DISTANCE:
            handle_obstacle()
        elif state["color"] == Color.GREEN:
            handle_green()
        elif state["color"] == Color.RED:
            handle_red()
        else:
            linefollow()
        print("MAIN", state["reflection"], state["distance"], state["color"])


main_loop()
