#!/usr/bin/env pybricks-micropython
# Import necessary libraries
from pybricks.ev3devices import Motor, ColorSensor, UltrasonicSensor
from pybricks.parameters import Port, Color
from pybricks.robotics import DriveBase
from pybricks.tools import wait
import math


# ---------- STANDARD SETUP CODE ---------- #


# Initialize the motors.
left_motor = Motor(Port.B)
right_motor = Motor(Port.C)


#Initialise the sensors.
light_sensor = ColorSensor(Port.S3)
distance_sensor = UltrasonicSensor(Port.S4)


# Initialize the drive base.
robot = DriveBase(left_motor, right_motor, wheel_diameter=56, axle_track=152)


# ---------- MAINLINE PROGRAM ---------- #
def getBrightness(sensor) -> int:
    # r, g, b = sensor.rgb()
    # return (r + g + b) / 3
    return sensor.reflection() # swap back 2 rgb when needed to


def checkWall(distanceFromWall: int = 15, waitT: int = 0) -> bool:
    if distance_sensor.distance() <= distanceFromWall:
        robot.stop()
        wait(waitT)
        return True
    return False


# we gotta try decrease KP as a higher KP means it will oscillate more; the far sensor makes it more sensitive to turns
# increase derivative as it gives more of an error estimation reducing issues from far sensor
# turn rate decay is the expo decay factor * -abs(turn_rate)
def followLine(base_speed: float = 50, kp: float = 1.2, kd: float = 6, target: float = 45, turn_speed_decay: float = 0.04) -> None:
    # now = datetime.now() # temp
    # cycles = 0


    last_err = 0
    HISTORY_LEN = 50
    history = [70] * HISTORY_LEN
    print(history)
    while True:
        brightness = getBrightness(light_sensor)
        # print(brightness)


        history.append(brightness)
        if len(history) > HISTORY_LEN:
            history.pop(0)


        if history[-1] > 65:
            robot.drive(base_speed, 0)
            continue


        # if brightness > 60:
        #     robot.drive(base_speed, 0)
        #     continue


        print("FOLLOWING LINE AT BRIGHTNESS", brightness)


        # if checkWall():
        #     break


        error = brightness - target
        derivitave = error - last_err


        if abs(derivitave) > 5:
            derivitave *= 1.8
        turn_rate = kp * error + kd * derivitave # removed: nonlinear error to try help prevent the understeering thing
        turn_rate = max(-100, min(turn_rate, 100)) # clamp to help prevent deg/s from going too high causing exception
        speed = base_speed - abs(error) * 0.5
        # speed = base_speed * math.exp(-turn_speed_decay * abs(turn_rate)) # removed: exponentially decrease speed based on turn_rate (expo decay)


        # try:
        robot.drive(speed, turn_rate)
        # except ValueError:
            # robot.drive(0, 0) # temp
        last_err = error
        # print("Speed: " + str(speed) + ", Turning: " + str(turn_rate) + ", Error: " + str(error))
        # last_error = error
        # cycles += 1
        # print(f"Cycles/s: {cycles/(datetime.now()-now).total_seconds()}")


# followLine()


# robot.turn(45)
# robot.drive(90, 0)
# wait(3750)
# robot.turn(45)
# robot.drive(90, 0)
# while lsr > 60:
#     pass


while True:
    lsr = getBrightness(light_sensor)
    if distance_sensor.distance() < 150:
        robot.stop()
        wait(1000)
        robot.turn(-45)
        robot.drive(90, 0)
        wait(2000)
        robot.turn(45)
        robot.drive(100, 0)
        wait(1000)
        robot.turn(25)
        robot.drive(90, 0)
        while getBrightness(light_sensor) > 50:
            print("IN")
            pass
    if lsr < 40:
        robot.drive(30, 20)
    elif lsr > 60:
        robot.drive(30, -20)
    else:
        robot.drive(50, 0)
        wait(40)