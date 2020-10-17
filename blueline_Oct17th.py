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

robot.settings(40)

robot.straight(120)
robot.stop(Stop.BRAKE)
robot.straight(-45)
robot.stop(Stop.BRAKE)
#robot.straight(130)
robot.straight(120)
robot.stop(Stop.BRAKE)
robot.settings(100)
robot.straight(-30)
robot.stop(Stop.BRAKE)
#robot.turn(50)
robot.turn(45)
robot.straight(-100)

while True:
    robot.drive(-30,0)
    if line_sensor1.color() == Color.BLACK:
        robot.stop(Stop.BRAKE)
        break 
large_motor.run_angle(50,200,then=Stop.HOLD, wait=False)
left_motor.run_angle(50,-300,then=Stop.HOLD, wait=True)
robot.straight(50)
robot.stop(Stop.BRAKE)

while True:
    robot.drive(30,0)
    if line_sensor.color() == Color.BLACK:
        robot.stop(Stop.BRAKE)
        break 
right_motor.run_angle(50,150,then=Stop.HOLD, wait=True)

while True:
    
    right_motor.run(85)
    if line_sensor.color() == Color.BLACK:
        robot.stop(Stop.BRAKE)
        break
BLACK = 9
WHITE = 85
threshold = (BLACK + WHITE) / 2
# Set the drive speed at 100 millimeters per second.
DRIVE_SPEED = 100

# Set the gain of the proportional line controller. This means that for every
# percentage point of light deviating from the threshold, we set the turn
# rate of the drivebase to 1.2 degrees per second.

# For example, if the light value deviates from the threshold by 10, the robot
# steers at 10*1.2 = 12 degrees per second.
PROPORTIONAL_GAIN = 1.2
runWhile = True
# Start following the line endlessly.
robot.reset()
while True:
    # Calculate the deviation from the threshold.
    deviation = line_sensor.reflection() - threshold

    # Calculate the turn rate.
    turn_rate = PROPORTIONAL_GAIN * deviation

    # Set the drive base speed and turn rate.
    robot.drive(DRIVE_SPEED, turn_rate)

    # You can wait for a short time or do other things in this loop.
    wait(10)


    print(line_sensor1.color())
    if line_sensor1.color() == Color.BLACK:
        robot.stop(Stop.BRAKE)
        break
robot.turn(25)
robot.straight(250)
robot.straight(-50)
large_motor.run_angle(100,-65)
robot.straight(-35)
large_motor.run_angle(50,50,then=Stop.HOLD, wait=False)
robot.turn(-170)
robot.straight(190)
large_motor.run_angle(300, -120, then=Stop.HOLD, wait=True)
robot.straight(-30)
large_motor.run_angle(100, 120, then=Stop.HOLD, wait=True)
## The robot moves straight towards the mission, getting ready to attempt to push the slide figures off once more. (In case it didn't work before.)
robot.straight(30)
## The large motor moves up again, and tries to push the figures off. 
large_motor.run_angle(300, -120, then=Stop.HOLD, wait=True)
robot.straight(-50)

## Time to come back to base. 
robot.turn(60)
robot.straight(60)
robot.turn(30)
robot.straight(120)
robot.turn(-80)
robot.straight(500)
robot.turn(-70)
robot.straight(600)
"""
robot.straight(170)
robot.stop(Stop.BRAKE)

robot.turn(-100)
while True:
    robot.drive(30,0)
    if line_sensor.color() == Color.BLACK:
        robot.stop(Stop.BRAKE)
        break 


BLACK = 9
WHITE = 85
threshold = (BLACK + WHITE) / 2
# Set the drive speed at 100 millimeters per second.
DRIVE_SPEED = 100

# Set the gain of the proportional line controller. This means that for every
# percentage point of light deviating from the threshold, we set the turn
# rate of the drivebase to 1.2 degrees per second.

# For example, if the light value deviates from the threshold by 10, the robot
# steers at 10*1.2 = 12 degrees per second.
PROPORTIONAL_GAIN = 1.2
runWhile = True
# Start following the line endlessly.
robot.reset()
while True:
    # Calculate the deviation from the threshold.
    deviation = line_sensor.reflection() - threshold

    # Calculate the turn rate.
    turn_rate = PROPORTIONAL_GAIN * deviation

    # Set the drive base speed and turn rate.
    robot.drive(DRIVE_SPEED, turn_rate)

    # You can wait for a short time or do other things in this loop.
    wait(10)


    print(robot.distance())
    if robot.distance() >= 500:
        break
robot.stop(Stop.BRAKE)
robot.turn(35)
robot.straight(260)
robot.straight(-50)
robot.stop(Stop.BRAKE)
large_motor.run_angle(100,-50)
robot.straight(-30)
large_motor.run_angle(80,-50)
robot.turn(-200)
large_motor.run_angle(50,100,then=Stop.HOLD, wait=False)
robot.straight(100)

"""
