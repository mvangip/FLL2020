#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile

#This assigns each motor to Port B and Port C
left_motor = Motor(Port.B)
right_motor = Motor(Port.C)

#The wheel diameter is 56 milimeters
#The axle track is 114 millimeters
wheel_diameter = 56
axle_track = 114

#This is the DriveBase object.
robot = DriveBase(left_motor, right_motor, wheel_diameter, axle_track)

#This tells the robot to move forward for 5 seconds. 
robot.drive_time(500, 0, 2000)















