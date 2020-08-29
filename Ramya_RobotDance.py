#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile


# This program requires LEGO EV3 MicroPython v2.0 or higher.
# Click "Open user guide" on the EV3 extension tab for more information.


# Create your objects here.
ev3 = EV3Brick()
left_motor = Motor(Port.B)
right_motor = Motor(Port.C)
front_largeMotor = Motor(Port.D)
medium_motor = Motor(Port.A)

wheel_diameter = 56
axle_track = 115

robot = DriveBase(left_motor, right_motor, wheel_diameter, axle_track)


# Write your code here.

robot.turn(160)
robot.turn(-160)
robot.straight(60)
front_largeMotor.run_target(500, 60)
front_largeMotor.run_target(500, -40)
robot.straight(-60)
robot.turn(260)
robot.turn(-260)
medium_motor.run_angle(500, 60)
medium_motor.run_target(500, -40)
robot.turn(100)
robot.straight(40)
robot.turn(100)
front_largeMotor.run_angle(500, 30)
medium_motor.run_angle(500, -20)
