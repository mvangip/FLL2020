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
#follows the line underneath the pull up bar until the leftsensor detects black
BLACK = 9
WHITE = 85
threshold = (BLACK + WHITE) / 2
# Set the drive speed at 100 millimeters per second.
DRIVE_SPEED = 100
# Set the gain of the proportional line controller. This means that for every
PROPORTIONAL_GAIN = 1.2
runWhile = True
#goes straight to get ready for line following then resets the distance
robot.straight(250)
robot.reset()
#starts to follow the line towards the replay logo
while True:
    # Calculate the deviation from the threshold.
    deviation = line_sensor.reflection() - threshold
    # Calculate the turn rate.
    turn_rate = PROPORTIONAL_GAIN * deviation
    # Set the drive base speed and turn rate.
    robot.drive(DRIVE_SPEED, turn_rate)
    wait(10)
    print(robot.distance())
    if(robot.distance ()>= 450):
        robot.stop(Stop.BRAKE)
        break
#the robot pushes the phone into the replay logo and moves back to get ready to drop the health units into the replay logo
robot.straight(-75)
robot.stop(Stop.BRAKE)
#the robot then turns so it is going to be perfectly into the replay logo
robot.turn(-35) 
#the robot drops the health units
large_motor.run_angle(100,150) 
#then turns to an angle to go back to base
robot.turn(50) 
robot.straight(-1000)
