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
wheel_diameter = 56
axle_track = 115

robot = DriveBase(left_motor, right_motor, wheel_diameter, axle_track)

Medium_Motor = Motor(Port.A)
Large_Motor = Motor(Port.D)

#Base to Basketball Hoop
robot.straight(300)
robot.turn(70)
robot.straight(453)
robot.turn(-103)
robot.straight(235)
#Drop Cube 
Large_Motor.run_angle(60,50,then = Stop.HOLD, wait = True)
#Large_Motor.run_angle(-100,100,then = Stop.HOLD, wait = True)
#Back Away From Hoop
#robot.straight(-70)
#robot.turn(-50)
#robot.straight(300)
#robot.turn(145)


#Medium_Motor.run_angle(100,300,then = Stop.HOLD, wait = True)
#robot.straight(50)
#robot.turn(80)
#Medium_Motor.run_angle(-100,270,then = Stop.HOLD, wait = True)
#robot.turn(-30)
#Medium_Motor.run_angle(100,400,then = Stop.HOLD, wait = True)

#robot.straight(-40)
#robot.turn(-35)
#Large_Motor.run_angle(60,100,then = Stop.HOLD, wait = True)
#robot.straight(25)
#Large_Motor.run_angle(-60,100,then = Stop.HOLD, wait = True)