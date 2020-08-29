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
large_motor = Motor(Port.D)
wheel_diameter = 56
axle_track = 115
 
robot = DriveBase(left_motor, right_motor, wheel_diameter, axle_track)
# Initialize the color sensor.
line_sensor = ColorSensor(Port.S2)
 
robot.straight(110)
 
# Calculate the light threshold. Choose values based on your measurements.
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
while runWhile:
    # Calculate the deviation from the threshold.
    deviation = line_sensor.reflection() - threshold
 
    # Calculate the turn rate.
    turn_rate = PROPORTIONAL_GAIN * deviation
 
    # Set the drive base speed and turn rate.
    robot.drive(DRIVE_SPEED, turn_rate)
 
    if robot.distance() == 780: 
        runWhile = False
 
robot.stop(Stop.BRAKE)
 
# robot turns to finish the line following code at a 103 degree turn
robot.turn(-103)

# robot moves forward towards the mission for 550mm
robot.straight(550)

# robot turns 70 degrees to align the attatchment with the mission
robot.turn(90)

# this moves the large motor attatchment down 200 degrees 
large_motor.run_angle(100, 170, then=Stop.HOLD, wait=True)

# this turns the bot 90 degrees so the attatchment can turn the mission
robot.straight(60)
 
# this moves the bot back, so it can push up the lever
robot.drive_time(-100, 0, 1000)

# this moves the bot forward to approach the mission once again
robot.straight(40)

# this turns the bot, so the attatchment can push the lever
robot.turn(-10)

# this moves the bot forward
robot.straight(40)

# finally, this pushes the lever up, to release the cube
large_motor.run_angle(50, -30, then=Stop.HOLD, wait=True)

# this moves the large motor back down
large_motor.run_angle(100, 30, then=Stop.HOLD, wait=True)

# testing code
#robot.straight(20)

