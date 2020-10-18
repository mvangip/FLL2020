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

leftcolorsensor = ColorSensor(Port.S3)
rightcolorsensor = ColorSensor(Port.S2)

#####

BLACK = 9
WHITE = 85
threshold = (BLACK + WHITE) / 2

######

robot.straight(320)
robot.turn(110)

while True:
    robot.drive(90,0)
    if leftcolorsensor.reflection() <= 9:
        robot.stop(Stop.BRAKE)
        break        

robot.turn(-110)

robot.straight(200)

# Calculate the light threshold. Choose values based on your measurements.
BLACK = 6
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
    deviation = rightcolorsensor.reflection() - threshold

    # Calculate the turn rate.
    turn_rate = PROPORTIONAL_GAIN * deviation

    # Set the drive base speed and turn rate.
    robot.drive(DRIVE_SPEED, turn_rate)

    if robot.distance() == 1000:
        runWhile = False

# robot stops after finishing up line following code

robot.stop(Stop.BRAKE)

robot.straight(-40)

robot.turn(-50)
robot.straight(145)
Large_Motor.run_angle(50,90,then = Stop.HOLD, wait = True)

#robot continues run, to do Boccia mission

while True:
    robot.drive(-80,0)
    if leftcolorsensor.reflection() <= 10:
        robot.stop(Stop.BRAKE)
        break 

robot.straight(80)
robot.turn(60)
robot.straight(100)


Large_Motor.run_angle(-50,150,then = Stop.HOLD, wait = True) 
robot.straight(-40)
Large_Motor.run_angle(50,150,then = Stop.HOLD, wait = True) 

robot.straight(-40)

while True:
    robot.drive(-80,0)
    if leftcolorsensor.reflection() <= 9:
        robot.stop(Stop.BRAKE)
        break

robot.straight(40)
robot.turn(-85)

robot.straight(340)
robot.turn(165)

robot.straight(55)
Large_Motor.run_angle(-50,150,then = Stop.HOLD, wait = True)

robot.straight(20)

Medium_Motor.run_angle(150,250,then = Stop.HOLD, wait = True)
robot.turn(70)
Medium_Motor.run_angle(-150,250,then = Stop.HOLD, wait = True)
robot.turn(-20)
robot.straight(-35)
Medium_Motor.run_angle(150,250,then = Stop.HOLD, wait = True)

robot.turn(30)
robot.straight(-130)


