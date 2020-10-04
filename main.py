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
robot.straight(700)
#push the step counter bit by bit
robot.straight(100)
Stop.BRAKE
#robot goes back for more power
robot.straight(-45)
Stop.BRAKE
#robot goes forward
robot.straight(150)
Stop.BRAKE
#turns to get an angle
robot.turn(100)
Stop.BRAKE
#robot goes backwards
robot.straight(-100)
Stop.BRAKE
#robot turns
robot.turn(345)
Stop.BRAKE
print(line_sensor.color())
while True:
    robot.drive(30,10)
    if line_sensor.color() == Color.BLACK:
        robot.stop(Stop.BRAKE)
        break 

#follows the line 
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
    if robot.distance() >= 350:
        break
robot.straight(120)
robot.turn(65)
robot.straight(250)
robot.straight(-25)
large_motor.run_angle(80,-50)
"""
robot.turn(75)
robot.straight(250)
robot.straight(-30)
large_motor.run_angle(80,-50)
"""