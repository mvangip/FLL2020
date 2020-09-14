#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile

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

#moves robot forward 700 millimeters towards the mission
robot.straight(700)
#beeps so i know it stopped
ev3.speaker.beep()
Stop.BRAKE

#pushes the step counter forward
robot.straight(100)
Stop.BRAKE
ev3.speaker.beep()

#moves backward and forwards just incase its stuck
robot.straight(-50)
robot.straight(10)
Stop.BRAKE
ev3.speaker.beep()

#moves even more forward since im aiming for blue
robot.straight(130)
Stop.BRAKE
ev3.speaker.beep()


robot.straight(60)
robot.turn(155)

