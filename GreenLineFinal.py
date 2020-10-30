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
axle_track = 114.3 

robot = DriveBase(left_motor, right_motor, wheel_diameter, axle_track)

## Write your code here: 

## The robot goes straight until the Boccia Mission's target. 
robot.straight(1060)

## The robot moves the large motor down to drop the cubes in the target. 
front_largeMotor.run_angle(80, 70, then=Stop.HOLD, wait=True)
front_largeMotor.run_angle(-80, 70, then=Stop.HOLD, wait=True)


## Dance Mission

## The robot moves backwards to reach the Dance Floor so it can Dance as the last mission. 
robot.straight(-185)
robot.turn(-70)
robot.straight(138) 


## The following code is all the dance moves we do for the Dance Mission. 

robot.turn(160)
robot.turn(-160)
robot.straight(60)
front_largeMotor.run_target(500, 60)
front_largeMotor.run_target(500, -40)
robot.straight(-60)
robot.turn(260)
robot.turn(-260)
robot.turn(100)
robot.straight(40)
robot.turn(100)
front_largeMotor.run_angle(500, 30)