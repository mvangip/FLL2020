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
line_sensor = ColorSensor(Port.S2)
line_sensor2 = ColorSensor(Port.S3)

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

    if robot.distance() == 800:
        runWhile = False

robot.drive(100, 0)
if line_sensor2 and cl.Color() == Color.BLACK:
        robot.stop(Stop.BRAKE)
 
    

# robot stops after finishing up line following code
robot.stop(Stop.BRAKE)
'''
# robot turns after finishing up line following code
robot.turn(-103.5)

# robot goes straight as it heads towards the mission
robot.straight(138)

# robot turns right for 90 degrees
robot.turn(80)

# robot goes straight towards the mission to line the attachment to the wheel
robot.straight(97)

# large motor attachment goes down to trap the wheel in
front_largeMotor.run_angle(60, 162)

# robot moves backwards to bring wheel outside of the large circle
robot.straight (-115)

# large motor releases the trapped tire
front_largeMotor.run_angle(60, -148)

# robot moves straight to get closer the wheel
robot.straight (38)

# robot turns so the wheel can get into the smaller target
robot.turn (-40)

robot.stop (Stop.BRAKE)

# robot goes backwards to leave the target and the wheel inside of it
robot.straight (-110)

# robot turns towards the weight machine
robot.turn (-30)


# going straight from row machine to weight machine
robot.straight(505)

# stopping for accuracy.
robot.stop(Stop.BRAKE)

# turning towards the weight machine.
robot.turn(30)

# robot goes straight to get closer to the weight machine
robot.straight(145)

# large motor going down to complete mission (weight machine).
front_largeMotor.run_angle(120, 130)


# going backwards away from the weight machine
robot.straight(-120)

# large motor goes back up 
# front_largeMotor.run_angle(50, -100)


## The robot is turning away from the Weight Machine and towards the Boccia.  
robot.turn(-127)

## The robot is moving straight towards the Boccia Mission.
robot.straight(290)

# the robot turns right to turn the aim boccia with the yellow axle on the bottom of the bot. 
robot.turn(60)


# robot.straight(-10)

# robot.turn(15)
# front_largeMotor.run_angle(50, 60)

# robot.straight(55)

# the large motor goes up to push the yellow cube down into the target area.
front_largeMotor.run_angle(50, -50)

robot.straight(-100)

robot.turn(-45)
robot.straight(900)

robot.turn(25)

robot.straight(700)
'''