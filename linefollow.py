#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, ColorSensor
from pybricks.parameters import Port
from pybricks.robotics import DriveBase
from pybricks.tools import wait

# Setup
simulator = 0
if simulator == 1:
    ev3 = EV3Brick()
    left_motor = Motor(Port.A)
    right_motor = Motor(Port.B)
    robot = DriveBase(left_motor, right_motor, wheel_diameter=56, axle_track=152)
    light_sensor = ColorSensor(Port.S1)
else: 
    ev3 = EV3Brick()
    left_motor = Motor(Port.B)
    right_motor = Motor(Port.C)
    robot = DriveBase(left_motor, right_motor, wheel_diameter=56, axle_track=152)
    light_sensor = ColorSensor(Port.S3)


def getBrightness(sensor):
    r, g, b = sensor.rgb()
    return (r + g + b) / 3

def followLineSingleSensor():
    # --- Configuration ---
    # Target is the "edge" of the line (usually halfway between black and white)
    # On this map, White ~90, Black ~10. Target = 50.
    TARGET = 50 
    BASE_SPEED = 80
    
    # PID Constants - Lowered KP to prevent the "aggressive" twitching
    KP = 0.8
    KI = 0.01
    KD = 1.2
    
    integral = 0
    last_error = 0
    
    # Search variables
    lost_count = 0
    MAX_LOST_CYCLES = 20 # How long to wait before declaring "Lost"

    while True:
        brightness = light_sensor.reflection()

        # Check if we are "Lost" (Seeing pure white for too long)
        if brightness > 85: 
            lost_count += 1
        else:
            lost_count = 0

        # RECOVERY LOGIC: If lost, do a sweeping search instead of a hard spin
        if lost_count > MAX_LOST_CYCLES:
            robot.stop()
            ev3.speaker.beep(100, 50) # Signal search mode
            
            # simple counter-based timer since StopWatch removed
            start = 0
            found = False
            
            # Pattern: Drive forward slowly then sweep left/right
            # Since sensor is on the left, we usually find the line by turning left
            search_phases = [(40, 0, 500), (0, -60, 1000), (0, 120, 2000)]
            
            for speed, turn, duration in search_phases:
                start = 0
                while start < duration:
                    robot.drive(speed, turn)
                    if light_sensor.reflection() < 60: # Found something dark
                        found = True
                        break
                    wait(10)
                    start += 10
                if found: break
            
            # Reset PID values after finding the line to prevent jumps
            integral = 0
            last_error = 0
            lost_count = 0
            continue

        # PID CALCULATION
        error = TARGET - brightness
        integral += error
        derivative = error - last_error
        
        turn_rate = (KP * error) + (KI * integral) + (KD * derivative)
        
        # Adaptive Speed: Slow down significantly during sharp turns/spikes
        # This keeps the sensor from "overshooting" the line
        current_speed = BASE_SPEED / (1 + abs(error) * 0.02)

        robot.drive(current_speed, turn_rate)
        
        last_error = error
        wait(10)

# Start line following
followLineSingleSensor()