#!/usr/bin/env pybricks-micropython
from pybricks.ev3devices import ColorSensor, Motor, UltrasonicSensor
from pybricks.parameters import Port, Color
from pybricks.robotics import DriveBase
from pybricks.tools import wait

# Motion tuning merged from teninrow.py, teninrow copy.py, and teninrow-nishal.py.
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
SENSOR_CHECK_LOOPS = 5
MARKER_CONFIRMATION_HITS = 3
MARKER_STREAK_REQUIRED = 2
LOOP_DELAY_MS = 5

WHEEL_DIAMETER = 56
AXLE_TRACK = 152

# Supports both hardware layouts used in the three source files.
PORT_PROFILES = (
    ("BC_S3S4", Port.B, Port.C, Port.S3, Port.S4),
    ("AB_S1S2", Port.A, Port.B, Port.S1, Port.S2),
)


state = {
    "r": 0,
    "g": 0,
    "b": 0,
    "reflection": 0,
    "distance": 0,
    "color": None,
}


marker_memory = {
    "last": None,
    "streak": 0,
}


def init_robot():
    for profile_name, left_port, right_port, color_port, ultrasonic_port in PORT_PROFILES:
        try:
            left_motor = Motor(left_port)
            right_motor = Motor(right_port)
            light_sensor = ColorSensor(color_port)
            distance_sensor = UltrasonicSensor(ultrasonic_port)
            robot = DriveBase(
                left_motor,
                right_motor,
                wheel_diameter=WHEEL_DIAMETER,
                axle_track=AXLE_TRACK,
            )
            print("Profile:", profile_name)
            return robot, light_sensor, distance_sensor
        except Exception:
            continue

    raise OSError("No valid motor/sensor profile found.")


robot, light_sensor, distance_sensor = init_robot()


def clamp(value, lower, upper):
    return max(lower, min(value, upper))


def drive_for(duration_ms, speed, turn):
    robot.drive(
        clamp(speed, -MAX_SPEED, MAX_SPEED),
        clamp(turn, -MAX_TURN, MAX_TURN),
    )
    wait(duration_ms)
    robot.stop()


def read_stable_state():
    total_r = 0
    total_g = 0
    total_b = 0
    total_distance = 0
    green_hits = 0
    red_hits = 0
    black_hits = 0

    for _ in range(SENSOR_CHECK_LOOPS):
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

        wait(LOOP_DELAY_MS)

    state["r"] = round(total_r / SENSOR_CHECK_LOOPS)
    state["g"] = round(total_g / SENSOR_CHECK_LOOPS)
    state["b"] = round(total_b / SENSOR_CHECK_LOOPS)
    state["distance"] = round(total_distance / SENSOR_CHECK_LOOPS)
    state["reflection"] = round((state["r"] + state["g"] + state["b"]) / 3)

    state["color"] = None
    if green_hits >= MARKER_CONFIRMATION_HITS:
        state["color"] = Color.GREEN
    elif red_hits >= MARKER_CONFIRMATION_HITS:
        state["color"] = Color.RED
    elif black_hits >= MARKER_CONFIRMATION_HITS:
        state["color"] = Color.BLACK


def detect_marker():
    rgb_green = state["g"] > (state["r"] + state["b"]) / 1.2
    rgb_red = state["r"] > (state["g"] + state["b"]) / 1.2

    if state["color"] == Color.BLACK and state["reflection"] <= 35:
        marker = Color.BLACK
    elif state["color"] == Color.GREEN or rgb_green:
        marker = Color.GREEN
    elif state["color"] == Color.RED or rgb_red:
        marker = Color.RED
    else:
        marker = None

    if marker is not None and marker == marker_memory["last"]:
        marker_memory["streak"] += 1
    elif marker is not None:
        marker_memory["last"] = marker
        marker_memory["streak"] = 1
    else:
        marker_memory["last"] = None
        marker_memory["streak"] = 0

    if marker_memory["streak"] >= MARKER_STREAK_REQUIRED:
        return marker_memory["last"]
    return None


def linefollow_step():
    error_nishal = -state["reflection"] - LINE_FOLLOW
    turn_nishal = error_nishal * EM
    turn_threshold = (state["reflection"] - THRESHOLD) * TURN_RATE
    turn_rate = (turn_nishal + turn_threshold) / 2

    speed = BASE_SPEED - abs(turn_rate) * ADJUST
    speed = clamp(speed, MIN_SPEED, MAX_SPEED)
    turn_rate = clamp(turn_rate, -MAX_TURN, MAX_TURN)

    robot.drive(speed, turn_rate)


def handle_green():
    drive_for(300, 45, 0)
    drive_for(400, 35, 60)
    wait(100)


def handle_red():
    drive_for(900, 50, 0)
    drive_for(500, 35, -40)


def handle_obstacle():
    robot.stop()
    drive_for(1500, 80, 35)
    drive_for(1800, 80, -25)


def run_start_phase():
    while True:
        read_stable_state()
        marker = detect_marker()

        if marker == Color.BLACK:
            drive_for(400, 100, -90)
            break

        linefollow_step()
        print("START", state["reflection"], state["color"])


def run_main_phase():
    marker_cooldown = 0

    while True:
        read_stable_state()
        marker = detect_marker()

        if marker_cooldown > 0:
            marker_cooldown -= 1
            marker = None

        if state["distance"] < OBSTACLE_DISTANCE:
            handle_obstacle()
            marker_cooldown = 6
        elif marker == Color.GREEN:
            handle_green()
            marker_cooldown = 6
        elif marker == Color.RED:
            handle_red()
            marker_cooldown = 6
        else:
            linefollow_step()

        print("MAIN", state["reflection"], state["distance"], marker)


def main():
    run_start_phase()
    run_main_phase()


main()
