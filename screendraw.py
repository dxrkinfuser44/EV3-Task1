#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, ColorSensor, UltrasonicSensor
from pybricks.parameters import Port
from pybricks.robotics import DriveBase

class EV3Robot:
    def __init__(self, simulator=0):
        self.ev3 = EV3Brick()
        self.simulator = simulator
        
        # Initialize motors
        if simulator == 0:
            left_port, right_port = Port.B, Port.C
        else:
            left_port, right_port = Port.A, Port.B
            
        self.left_motor = Motor(left_port)
        self.right_motor = Motor(right_port)
        
        # Initialize sensors
        if simulator == 0:
            self.light_sensor = ColorSensor(Port.S3)
            self.obstacle_sensor = UltrasonicSensor(Port.S4)
        else:
            self.light_sensor = ColorSensor(Port.S1)
            self.obstacle_sensor = UltrasonicSensor(Port.S2)
        
        # Initialize drive base
        self.robot = DriveBase(self.left_motor, self.right_motor, wheel_diameter=56, axle_track=152)
        self.robot.settings(straight_speed=200, straight_acceleration=100, turn_rate=100)
    
    def draw(self):
        if self.obstacle_sensor.distance() < 5:
            self.robot.stop()
            self.robot.drive(100, 0)
        else:
            self.robot.drive(60, 100)
