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

#define your variables
ev3 = EV3Brick()
left_motor = Motor(Port.C)
right_motor = Motor(Port.B)
medium_motor = Motor(Port.A)
large_motor = Motor(Port.D)
wheel_diameter = 56
axle_track = 115  
line_sensor = ColorSensor(Port.S2)
line_sensor1 = ColorSensor(Port.S3)
robot = DriveBase(left_motor, right_motor, wheel_diameter, axle_track)

#moves robot forward 700 millimeters towards the mission
robot.straight(700)
ev3.speaker.beep()
#pushes the step counter slowly
robot.straight(100)
Stop.BRAKE
ev3.speaker.beep()

# goes back for targetting the blue part of the step counter
robot.straight(-50)
robot.straight(10)
Stop.BRAKE
ev3.speaker.beep()

#does its final push
robot.straight(150)
Stop.BRAKE
ev3.speaker.beep()

#goes back so it can be easy to detect the first black line
robot.straight(-150)

#uses the color sensor (2) to allow it to detect the line
while True:
  
    robot.drive(-30,0)
    print(line_sensor.reflection())
    if line_sensor.reflection() <= 12:
        robot.stop(Stop.BRAKE)
        break

#turns towards the pull up bar
left_motor.run_angle(40,500)

#goes underneath the pull-up bar
robot.straight(-400)

