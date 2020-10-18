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

line_sensor = ColorSensor(Port.S3)
#line_sensor2 = Color Sensor(Port.S2)

#####

BLACK = 9
WHITE = 85
threshold = (BLACK + WHITE) / 2

######

robot.straight(320)
robot.turn(110)

while True:
    robot.drive(90,0)
    if line_sensor.reflection() <= 10:
        robot.stop(Stop.BRAKE)
        break 

robot.turn(-100)

robot.straight(200)

# Calculate the light threshold. Choose values based on your measurements.
BLACK = 9
WHITE = 85
threshold = (BLACK + WHITE) / 2

# Set the drive speed at 100 millimeters per second.
DRIVE_SPEED = 110

# Set the gain of the proportional line controller. This means that for every
# percentage point of light deviating from the threshold, we set the turn
# rate of the drivebase to 1.2 degrees per second.

# For example, if the light value deviates from the threshold by 10, the robot
# steers at 10*1.2 = 12 degrees per second.
PROPORTIONAL_GAIN = 1.2
runWhile = True
# Start following the line endlessly.
while runWhile:
    # Calculate the deviation from the threshold.
    deviation = line_sensor.reflection() - threshold

    # Calculate the turn rate.
    turn_rate = PROPORTIONAL_GAIN * deviation

    # Set the drive base speed and turn rate.
    robot.drive(DRIVE_SPEED, turn_rate)

    if robot.distance() == 1080:
        runWhile = False

# robot stops after finishing up line following code

robot.stop(Stop.BRAKE)

Medium_Motor

