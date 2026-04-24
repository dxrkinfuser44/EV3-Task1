#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, ColorSensor, UltrasonicSensor
from pybricks.parameters import Port, Color
from pybricks.robotics import DriveBase
from pybricks.tools import wait
import math


ev3 = EV3Brick()


# Simulator support preserved from linefollow/new3/linefollow2.
SIMULATOR = 0
if SIMULATOR == 1:
    left_motor = Motor(Port.A)
    right_motor = Motor(Port.B)
    light_sensor = ColorSensor(Port.S1)
else:
    left_motor = Motor(Port.B)
    right_motor = Motor(Port.C)
    light_sensor = ColorSensor(Port.S3)

robot = DriveBase(left_motor, right_motor, wheel_diameter=56, axle_track=152)

distance_sensor = None
try:
    distance_sensor = UltrasonicSensor(Port.S4)
except Exception:
    distance_sensor = None


CONFIG = {
    # Startup behavior from mat14_2 and base.py patterns.
    "wait_for_first_green": False,
    "drive_until_first_white": True,
    "startup_speed": 90,
    "run_initial_recovery": True,
    "allow_drive_test_pattern": False,
    # General loop timing.
    "control_step_ms": 20,
    # Line quality detection and history (mat10 + mat14_2).
    "low_brightness": 20,
    "high_brightness": 75,
    "line_confirm_count": 4,
    "lost_count_limit": 15,
    "history_len": 50,
    "white_straight_threshold": 65,
    # Color turns (new3 + mat14_2).
    "left_turn_colors": [Color.BLUE, Color.GREEN],
    "right_turn_colors": [Color.RED],
    "turn_angle": 90,
    "color_cooldown_ms": 500,
    "color_check_every_n_loops": 2,
    # Obstacle handling (mat10 + obstacle + unified).
    "use_obstacle_bypass": True,
    "obstacle_distance_mm": 130,
    "obstacle_line_search_timeout_ms": 6000,
    # Recovery tuning (linefollow + mat14_2).
    "search_turn_rate": 75,
    "search_sweep_ms": 450,
    "search_step_ms": 20,
    "found_line_threshold": 60,
    # Simple fallback control from obstacle/mat10.
    "fallback_low": 40,
    "fallback_high": 60,
    "fallback_speed": 50,
    "fallback_turn": 20,
    "fallback_turn_sign": 1,
    # Continuous-motion tuning.
    "coast_speed": 35,
    "coast_turn_rate": 0,
    "dynamic_turn_rate": 130,
    "dynamic_turn_speed": 75,
    "arc_ms_per_degree": 8,
}


# PID profiles preserve the styles from mat4, linefollow2, mat10 and unified.py.
PID_PROFILES = {
    "mat4_fast": {
        "target": 53,
        "kp": 2.9,
        "ki": 0.0,
        "kd": 3.0,
        "max_turn": 500,
        "base_speed": 90,
        "min_speed": 35,
        "speed_model": "exp_error",
        "speed_decay": 0.07,
        "error_mode": "target_minus",
        "boost_threshold": 0,
        "boost_factor": 1.0,
    },
    "linefollow_smooth": {
        "target": 50,
        "kp": 0.8,
        "ki": 0.01,
        "kd": 1.2,
        "max_turn": 240,
        "base_speed": 80,
        "min_speed": 30,
        "speed_model": "adaptive_div",
        "speed_decay": 0.02,
        "error_mode": "target_minus",
        "boost_threshold": 0,
        "boost_factor": 1.0,
    },
    "mat10_stable": {
        "target": 45,
        "kp": 1.2,
        "ki": 0.0,
        "kd": 6.0,
        "max_turn": 140,
        "base_speed": 50,
        "min_speed": 25,
        "speed_model": "linear_error",
        "speed_decay": 0.5,
        "error_mode": "brightness_minus",
        "boost_threshold": 5,
        "boost_factor": 1.8,
    },
    "unified_balanced": {
        "target": 45,
        "kp": 1.3,
        "ki": 0.0,
        "kd": 5.0,
        "max_turn": 140,
        "base_speed": 90,
        "min_speed": 35,
        "speed_model": "exp_turn",
        "speed_decay": 0.04,
        "error_mode": "brightness_minus",
        "boost_threshold": 5,
        "boost_factor": 1.8,
    },
}


RECOVERY_PHASES = [
    (40, 0, 500),
    (0, -60, 1000),
    (0, 120, 2000),
]


OBSTACLE_MANEUVERS = {
    "mat10": [(-65, 0, 0), (0, 90, 2000), (65, 0, 0), (0, 100, 3000), (65, 0, 0), (0, 90, 0)],
    "obstacle": [(-45, 0, 0), (0, 90, 2000), (45, 0, 0), (0, 100, 1000), (25, 0, 0), (0, 90, 0)],
    "unified": [(-55, 0, 0), (0, 90, 1800), (55, 0, 0), (0, 100, 1400), (30, 0, 0), (0, 90, 0)],
}


def clamp(value, low, high):
    return max(low, min(high, value))


def getBrightness(sensor):
    return sensor.reflection()


def get_brightness(sensor):
    return getBrightness(sensor)


def checkWall(distanceFromWall=15, waitT=0):
    if distance_sensor is None:
        return False
    if distance_sensor.distance() <= distanceFromWall:
        robot.drive(CONFIG["coast_speed"], CONFIG["coast_turn_rate"])
        wait(waitT)
        return True
    return False


class AdaptiveController:
    def __init__(self):
        self.integral = 0
        self.last_error = 0
        self.last_brightness = 50
        self.lost_count = 0
        self.color_lock = False
        self.loop_count = 0
        self.history = [70] * CONFIG["history_len"]
        self.command_speed = CONFIG["coast_speed"]
        self.command_turn = 0

    def reset_pid(self):
        self.integral = 0
        self.last_error = 0

    def on_plausible_track(self, brightness):
        return CONFIG["low_brightness"] <= brightness <= CONFIG["high_brightness"]

    def has_found_line(self, samples=None):
        if samples is None:
            samples = CONFIG["line_confirm_count"]
        hits = 0
        for _ in range(samples):
            if self.on_plausible_track(get_brightness(light_sensor)):
                hits += 1
            wait(10)
        return hits >= (samples - 1)

    def keep_moving(self, speed=None, turn_rate=None):
        if speed is None:
            speed = CONFIG["coast_speed"]
        if turn_rate is None:
            turn_rate = CONFIG["coast_turn_rate"]
        robot.drive(speed, turn_rate)

    def drive_dynamic(self, speed, turn_rate, max_turn=180):
        turn_ratio = abs(turn_rate) / max(1, max_turn)
        response = 0.25 + min(0.45, 0.55 * turn_ratio)
        self.command_speed += (speed - self.command_speed) * response
        self.command_turn += (turn_rate - self.command_turn) * response
        robot.drive(self.command_speed, self.command_turn)

    def arc_turn(self, angle):
        if angle == 0:
            return
        duration = max(80, int(abs(angle) * CONFIG["arc_ms_per_degree"]))
        turn_rate = CONFIG["dynamic_turn_rate"] if angle > 0 else -CONFIG["dynamic_turn_rate"]
        robot.drive(CONFIG["dynamic_turn_speed"], turn_rate)
        wait(duration)
        robot.drive(CONFIG["dynamic_turn_speed"], 0)

    def drive_test_pattern(self):
        robot.drive(200, 0)
        wait(1000)
        self.keep_moving()
        wait(400)
        robot.drive(-200, 0)
        wait(1000)
        self.keep_moving()

    def wait_for_first_green(self):
        while light_sensor.color() != Color.GREEN:
            robot.drive(max(20, CONFIG["coast_speed"]), 12)
            wait(CONFIG["control_step_ms"])
        self.keep_moving()
        wait(100)

    def drive_until_first_white(self):
        while True:
            robot.drive(CONFIG["startup_speed"], 0)
            if light_sensor.color() == Color.WHITE:
                self.keep_moving(speed=max(CONFIG["coast_speed"], CONFIG["startup_speed"] - 25))
                wait(100)
                return
            wait(CONFIG["control_step_ms"])

    def handle_color_turn(self, color):
        if self.color_lock:
            if color not in CONFIG["left_turn_colors"] and color not in CONFIG["right_turn_colors"]:
                self.color_lock = False
            return self.color_lock

        if color in CONFIG["left_turn_colors"]:
            self.arc_turn(-CONFIG["turn_angle"])
            self.keep_moving(speed=CONFIG["dynamic_turn_speed"])
            wait(CONFIG["color_cooldown_ms"])
            self.color_lock = True
            return True

        if color in CONFIG["right_turn_colors"]:
            self.arc_turn(CONFIG["turn_angle"])
            self.keep_moving(speed=CONFIG["dynamic_turn_speed"])
            wait(CONFIG["color_cooldown_ms"])
            self.color_lock = True
            return True

        return False

    def execute_maneuver(self, steps):
        for turn_angle, speed, duration in steps:
            if turn_angle != 0:
                self.arc_turn(turn_angle)
            if speed != 0:
                robot.drive(speed, 0)
            if duration > 0:
                if speed == 0:
                    self.keep_moving()
                wait(duration)

    def bypass_obstacle(self):
        self.keep_moving(turn_rate=35)
        wait(400)

        distance = 1000
        if distance_sensor is not None:
            distance = distance_sensor.distance()

        if distance < 110:
            selected = OBSTACLE_MANEUVERS["mat10"]
        elif distance < 150:
            selected = OBSTACLE_MANEUVERS["unified"]
        else:
            selected = OBSTACLE_MANEUVERS["obstacle"]

        self.execute_maneuver(selected)

        elapsed = 0
        while elapsed < CONFIG["obstacle_line_search_timeout_ms"]:
            if self.has_found_line(3):
                self.keep_moving()
                wait(200)
                return
            wait(20)
            elapsed += 20

        self.recover_line()

    def recover_line(self):
        try:
            ev3.speaker.beep(900, 80)
        except Exception:
            pass

        for speed, turn, duration in RECOVERY_PHASES:
            elapsed = 0
            while elapsed < duration:
                robot.drive(speed, turn)
                if get_brightness(light_sensor) < CONFIG["found_line_threshold"]:
                    self.keep_moving()
                    wait(100)
                    return
                wait(10)
                elapsed += 10

        sweep_count = 0
        while True:
            if self.has_found_line():
                self.keep_moving()
                wait(100)
                return

            direction = 1 if (sweep_count % 2 == 0) else -1
            sweep_duration = CONFIG["search_sweep_ms"] + (sweep_count // 2) * 200
            elapsed = 0

            while elapsed < sweep_duration:
                if self.has_found_line(2):
                    self.keep_moving()
                    wait(100)
                    return

                robot.drive(0, direction * CONFIG["search_turn_rate"])
                wait(CONFIG["search_step_ms"])
                elapsed += CONFIG["search_step_ms"]

            sweep_count += 1

    def select_pid_profile(self, brightness, error_gradient):
        if self.lost_count > 0 or brightness > CONFIG["white_straight_threshold"]:
            return PID_PROFILES["linefollow_smooth"]
        if abs(error_gradient) > 8:
            return PID_PROFILES["mat10_stable"]
        if abs(self.last_error) < 8:
            return PID_PROFILES["mat4_fast"]
        return PID_PROFILES["unified_balanced"]

    def compute_command(self, profile, brightness):
        if profile["error_mode"] == "brightness_minus":
            error = brightness - profile["target"]
        else:
            error = profile["target"] - brightness

        self.integral += error
        derivative = error - self.last_error

        if profile["boost_threshold"] > 0 and abs(derivative) > profile["boost_threshold"]:
            derivative *= profile["boost_factor"]

        turn_rate = (
            profile["kp"] * error
            + profile["ki"] * self.integral
            + profile["kd"] * derivative
        )
        turn_rate = clamp(turn_rate, -profile["max_turn"], profile["max_turn"])

        if profile["speed_model"] == "exp_error":
            speed = profile["base_speed"] * math.exp(-profile["speed_decay"] * abs(error))
        elif profile["speed_model"] == "exp_turn":
            speed = profile["base_speed"] * math.exp(-profile["speed_decay"] * abs(turn_rate))
        elif profile["speed_model"] == "linear_error":
            speed = profile["base_speed"] - abs(error) * profile["speed_decay"]
        else:
            speed = profile["base_speed"] / (1 + abs(error) * profile["speed_decay"])

        speed = clamp(speed, profile["min_speed"], profile["base_speed"])
        self.last_error = error
        return speed, turn_rate

    def fallback_line_follow(self, brightness):
        low = CONFIG["fallback_low"]
        high = CONFIG["fallback_high"]
        turn = CONFIG["fallback_turn"] * CONFIG["fallback_turn_sign"]

        if brightness < low:
            robot.drive(CONFIG["fallback_speed"] - 20, turn)
            return
        if brightness > high:
            robot.drive(CONFIG["fallback_speed"] - 20, -turn)
            return

        robot.drive(CONFIG["fallback_speed"], 0)

    def step(self):
        brightness = get_brightness(light_sensor)
        self.history.append(brightness)
        if len(self.history) > CONFIG["history_len"]:
            self.history.pop(0)

        if (
            CONFIG["use_obstacle_bypass"]
            and distance_sensor is not None
            and distance_sensor.distance() < CONFIG["obstacle_distance_mm"]
        ):
            self.bypass_obstacle()
            self.reset_pid()
            self.lost_count = 0
            return

        should_check_color = (self.loop_count % CONFIG["color_check_every_n_loops"]) == 0
        self.loop_count += 1

        if should_check_color:
            color = light_sensor.color()
            if color == Color.WHITE or color == Color.BLACK:
                self.color_lock = False
            elif self.handle_color_turn(color):
                self.lost_count = 0
                return

        if brightness > CONFIG["white_straight_threshold"] and min(self.history[-8:]) < 55:
            self.drive_dynamic(60, 0, 180)
            self.last_brightness = brightness
            return

        if not self.on_plausible_track(brightness):
            self.lost_count += 1
        else:
            self.lost_count = 0

        if self.lost_count >= CONFIG["lost_count_limit"]:
            self.recover_line()
            self.reset_pid()
            self.lost_count = 0
            self.last_brightness = get_brightness(light_sensor)
            return

        brightness_gradient = brightness - self.last_brightness
        profile = self.select_pid_profile(brightness, brightness_gradient)
        speed, turn_rate = self.compute_command(profile, brightness)

        if not self.on_plausible_track(brightness) and abs(turn_rate) < 15:
            self.fallback_line_follow(brightness)
        else:
            self.drive_dynamic(speed, turn_rate, profile["max_turn"])

        self.last_brightness = brightness

    def run(self):
        if CONFIG["allow_drive_test_pattern"]:
            self.drive_test_pattern()

        if CONFIG["wait_for_first_green"]:
            self.wait_for_first_green()

        if CONFIG["drive_until_first_white"]:
            self.drive_until_first_white()

        if CONFIG["run_initial_recovery"]:
            self.recover_line()

        while True:
            self.step()
            wait(CONFIG["control_step_ms"])


def followLineSingleSensor():
    AdaptiveController().run()


def followLine(base_speed=50, kp=1.2, kd=6, target=45, turn_speed_decay=0.04):
    PID_PROFILES["mat10_stable"]["base_speed"] = base_speed
    PID_PROFILES["mat10_stable"]["kp"] = kp
    PID_PROFILES["mat10_stable"]["kd"] = kd
    PID_PROFILES["mat10_stable"]["target"] = target
    PID_PROFILES["unified_balanced"]["speed_decay"] = turn_speed_decay
    AdaptiveController().run()


def main():
    AdaptiveController().run()


main()
