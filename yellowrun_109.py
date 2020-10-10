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

# Initialize the color sensor.
right_sensor = ColorSensor(Port.S2)
left_sensor = ColorSensor(Port.S3)

robot.straight(110)

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
    deviation = right_sensor.reflection() - threshold

    # Calculate the turn rate.
    turn_rate = PROPORTIONAL_GAIN * deviation

    # Set the drive base speed and turn rate.
    robot.drive(DRIVE_SPEED, turn_rate)

    # set the distance the robot needs to go
    if robot.distance() == 1380:
        runWhile = False

# robot stops after finishing up line following code

robot.stop(Stop.BRAKE)

# the left color sensor senses the perpendicular black line to increase reliability
while True:
  
    robot.drive(100,0)
    print(left_sensor.reflection())
    if left_sensor.reflection() <= 10:
        robot.stop(Stop.BRAKE)
        break

# the robot beeps to confirm the color sensor code worked
ev3.speaker.beep()

# the robot moves backwards just a bit so it has more space to turn
robot.straight(-25)

# robot turns left, forming a right angle
robot.turn(-105)

# robot goes straight as it heads towards the mission
robot.straight(320)

# robot turns right for 150 degrees, with the wheel, and smaller circle in a straight line
robot.turn(150)

# robot goes straight to get closer to the wheel
robot.straight(100)

# the robot turns a little to the left to move the wheel in front of the smaller target
robot.turn(-90)

# the robot moves backwards to give the attachment some space to trap the wheel
robot.straight(-25)

# large motor attachment goes down to trap the wheel in its clutches
front_largeMotor.run_angle(60, 162)

# robot moves backwards to bring wheel outside of the large circle and into the small circle
robot.straight (-100)

# large motor attachment goes back up
front_largeMotor.run_angle(60, -170)

# the robot moves backwards to not ruin any progress that it might have made with the mission
robot.straight(-50)

# robot turns left so that it can go to accomplish the weight machine
robot.turn(-60)

# robot goes straight for a bit before the color sensing code
robot.straight(90)

# right color sensor senses the black line first
while True:
  
    robot.drive(100,0)
    print(left_sensor.reflection())
    if left_sensor.reflection() <= 10:
        robot.stop(Stop.BRAKE)
        break

ev3.speaker.beep()

right_motor.run_angle(5,5)
while True:
  
    right_motor.run(70)
    print(right_sensor.reflection())
    if right_sensor.reflection() <= 13:
        robot.stop(Stop.BRAKE)
        break

ev3.speaker.beep()

robot.turn(-5)
robot.straight(140)

front_largeMotor.run_angle(60,160)

robot.straight(-10)

while True:
  
    robot.drive(-100, 0)
    print(left_sensor.reflection())
    if left_sensor.reflection() <= 13:
        robot.stop(Stop.BRAKE)
        break 

robot.stop(Stop.BRAKE) 
ev3.speaker.beep()

right_motor.run_angle(20,5)

while True:
  
    right_motor.run(70)
    print(right_sensor.reflection())
    if right_sensor.reflection() <= 13:
        robot.stop(Stop.BRAKE)
        break

robot.stop(Stop.BRAKE)

ev3.speaker.beep()

right_motor.run_angle(150, 490)

DRIVE_SPEED = 80

runWhile = True
# Start following the line endlessly.
while runWhile:
    # Calculate the deviation from the threshold.
    deviation = left_sensor.reflection() - threshold

    # Calculate the turn rate.
    turn_rate = PROPORTIONAL_GAIN * deviation

    # Set the drive base speed and turn rate.
    robot.drive(DRIVE_SPEED, turn_rate)

    # set the distance the robot needs to go
    if robot.distance() == 690:
        runWhile = False

# robot stops after finishing up line following code

robot.stop(Stop.BRAKE)

ev3.speaker.beep()