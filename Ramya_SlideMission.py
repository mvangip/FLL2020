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

## This is a line following code. It will follow a line given on the mat, and is coded to stop moving after the robot reaches a certain distance. 
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

    if robot.distance() == 810: 
        runWhile = False


## The line following code has ended. Now, the robot turns for 103 degrees. It is turning into the pull up bar. It must go through. 
robot.turn(-103)
## After the turn, the robot is coded to go straight for 448 millimeters. It goes under the pull up bar, and more towards the mission. 
robot.straight(448)
## The robot turns again for 130 degrees. 
robot.turn(-130)
## The robot moves straight again, for 149.7 millimeters. It is now at the mission. 
robot.straight(149.7)
## Just in case the robot got too close to the mission, it is backing up for two centimeters. 
robot.straight(-20)



## Now that the robot is at the mission, it's time to complete it. The large motor in the front of the bot is coded to move up, and push the slide 
## figures off of the slide. 
front_largeMotor.run_angle(300, -120, then=Stop.HOLD, wait=True)
robot.straight(-30)
front_largeMotor.run_angle(100, 120, then=Stop.HOLD, wait=True)
## The robot moves straight towards the mission, getting ready to attempt to push the slide figures off once more. (In case it didn't work before.)
robot.straight(30)
## The large motor moves up again, and tries to push the figures off. 
front_largeMotor.run_angle(300, -120, then=Stop.HOLD, wait=True)
front_largeMotor.run_angle(300, -35, then=Stop.HOLD, wait=True)
## The robot is now moving backwards for 5 centimeters, away from the mission, to make sure it doesn't ruin any of it's progress. (If it made any.) 
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

