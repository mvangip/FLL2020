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
left_motor = Motor(Port.C)
right_motor = Motor(Port.B)
medium_motor = Motor(Port.A)
front_largeMotor = Motor(Port.D)
wheel_diameter = 56
axle_track = 115

robot = DriveBase(left_motor, right_motor, wheel_diameter, axle_track)

#program

#robot goes straight towards the mission
robot.straight(-100)

#robot turns so turns so it gets the angle straigt
robot.turn(45)

#the robot then goes straught to then adjust itself
robot.straight(-350)

#the robot then turns to push the bench down
robot.turn(-45)

#the robot then moves forward
robot.straight(200)

#robot turns towards the fallen bench to do the hopscotch
robot.turn(155)

#robot moves a little back to get the right angle
robot.straight(125)

#robot then slowly moves down the attatchment to drop the cubes
front_largeMotor.run_angle(85,85)

#robot takes a turn to get ready for the next part
robot.turn(-65)

#robot goes straight to get ready to do the next mission
robot.straight(125)

#robot lifts the attatchment down to lift the rest
front_largeMotor.run_angle(45,10)

#robot goes towards the rest
robot.straight(50)

#robot turns to the left to get ready to lift
robot.turn(120)

#robot goes straight
robot.straight(50)

# the attatchment moves up to lift the rest
front_largeMotor.run_angle(45,-65)

robot.turn(-10)

#robot goes back to base
robot.straight(-1000)







