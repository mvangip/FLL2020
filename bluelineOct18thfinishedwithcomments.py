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

#go front towards the step counter
robot.straight(650)
robot.stop(Stop.BRAKE)
wait(20)

#makes the robot go slower 
robot.settings(40)

#slowly pushes the step counter by going back and front 2 times
robot.straight(120)
robot.stop(Stop.BRAKE)
robot.straight(-45)
robot.stop(Stop.BRAKE)
robot.straight(120)
robot.stop(Stop.BRAKE)
robot.settings(100)
robot.straight(-30)
robot.stop(Stop.BRAKE)
#the robot then turns and goes backwards
robot.turn(45)
robot.straight(-100)
# the robot then goes back until the right color sensor detects back
while True:
    robot.drive(-30,0)
    if line_sensor1.color() == Color.BLACK:
        robot.stop(Stop.BRAKE)
        break 
#the large motor attatchment comes down at the same time the robot takes a turn towards the black line underneath the pull up bar
large_motor.run_angle(50,200,then=Stop.HOLD, wait=False)
left_motor.run_angle(50,-300,then=Stop.HOLD, wait=True)
#the robot then goes straight towards that line
robot.straight(50)
robot.stop(Stop.BRAKE)
#robot continues to go forwards until the left color sensor detects black
while True:
    robot.drive(30,0)
    if line_sensor.color() == Color.BLACK:
        robot.stop(Stop.BRAKE)
        break 
right_motor.run_angle(50,150,then=Stop.HOLD, wait=True)
#the robot then turns with the right motor until it detects black
while True:
    right_motor.run(85)
    if line_sensor.color() == Color.BLACK:
        robot.stop(Stop.BRAKE)
        break
#follows the line underneath the pull up bar until the leftsensor detects black
BLACK = 9
WHITE = 85
threshold = (BLACK + WHITE) / 2
# Set the drive speed at 100 millimeters per second.
DRIVE_SPEED = 100
# Set the gain of the proportional line controller. This means that for every
PROPORTIONAL_GAIN = 1.2
runWhile = True
robot.reset()
while True:
    # Calculate the deviation from the threshold.
    deviation = line_sensor.reflection() - threshold
    # Calculate the turn rate.
    turn_rate = PROPORTIONAL_GAIN * deviation
    # Set the drive base speed and turn rate.
    robot.drive(DRIVE_SPEED, turn_rate)
    wait(10)
    print(line_sensor1.color())
    if line_sensor1.color() == Color.BLACK:
        robot.stop(Stop.BRAKE)
        break
#the robot then turns towards the boccia aim and moves straight to push it towards the target and finishes the misison 
robot.turn(25)
robot.straight(250)
robot.straight(-50)
large_motor.run_angle(100,-65)
robot.straight(-35)
#the robot then takes a turn (at the same time bringing the attatchment down) towards the slide mission and completes the mission
large_motor.run_angle(50,50,then=Stop.HOLD, wait=False)
robot.turn(-170)
robot.straight(200)
large_motor.run_angle(300, -120, then=Stop.HOLD, wait=True)
robot.straight(-30)
large_motor.run_angle(100, 120, then=Stop.HOLD, wait=True)
## The robot moves straight towards the mission, getting ready to attempt to push the slide figures off once more. (In case it didn't work before.)
robot.straight(30)
large_motor.run_angle(300, -120, then=Stop.HOLD, wait=True)
robot.straight(-50)
#the robot then goes back to base after finishing all of the designated missions
robot.turn(60)
robot.straight(60)
robot.turn(30)
robot.straight(120)
robot.turn(-80)
robot.straight(500)
robot.turn(-70)
robot.straight(600)
