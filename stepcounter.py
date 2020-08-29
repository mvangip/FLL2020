#!/usr/bin/env pybricks-micropython

"""
Example LEGO® MINDSTORMS® EV3 Robot Educator Driving Base Program
-----------------------------------------------------------------

This program requires LEGO® EV3 MicroPython v2.0.
Download: https://education.lego.com/en-us/support/mindstorms-ev3/python-for-ev3

Building instructions can be found at:
https://education.lego.com/en-us/support/mindstorms-ev3/building-instructions#robot
"""

from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, Stop
from pybricks.parameters import Port
from pybricks.robotics import DriveBase

# Initialize the EV3 Brick.
ev3 = EV3Brick()

# Initialize the motors.
left_motor = Motor(Port.B)
right_motor = Motor(Port.C)

# Initialize the drive base.
robot = DriveBase(left_motor, right_motor, wheel_diameter=55.5, axle_track=104)


robot.straight(700)
ev3.speaker.beep()
Stop.BRAKE

robot.straight(100)
Stop.BRAKE
ev3.speaker.beep()

robot.straight(-50)
robot.straight(10)
Stop.BRAKE
ev3.speaker.beep()

robot.straight(300)
Stop.BRAKE
ev3.speaker.beep()

robot.straight(-1500)
ev3.speaker.beep
Stop.BRAKE
