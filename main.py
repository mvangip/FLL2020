#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile

import sleep

# This program requires LEGO EV3 MicroPython v2.0 or higher.
# Click "Open user guide" on the EV3 extension tab for more information.


# Create your objects here.
ev3 = EV3Brick()


# Assign variables to the right and left motor 
right_motor = Motor(Port.B)
left_motor = Motor(Port.C)
medium_motor = Motor(Port.A)
attachment_motor = Motor(Port.D)
wheel_diameter = 56
axle_track = 114

# Create a DriveBase object. The axle_track and wheel_diameter are needed to correct
# the speed when the robot is moving
robot = DriveBase(left_motor, right_motor, wheel_diameter, axle_track)

# HEALTH UNITS MISSION (RETRIEVING FROM DANCE FLOOR AND DUMPING THEM INTO THE REPLAY LOGO)

# robot goes straight for 300 mm
robot.straight(300)

# robot turns left towards the boccia mission
robot.turn(-60)

# robot goes straight to retrieve the health unit
robot.straight(400)

# robot takes a slight right turn to line the attachment up with the health unit
robot.turn(45)

# robot moves straight to get closer to the health unit
robot.straight(130)

# large motor attachment in the front moves downwards so the beam can go inside the loop of the health unit
attachment_motor.run_angle(40, 120)

# robot goes straight just a little big for the attachment to catch the health unit's looped wire
robot.straight(45)

# large motor attachment then moves more downwards to trap the health unit in its clutches
attachment_motor.run_angle(40, 30)

# robot goes backwards all the way into base
robot.straight(-915)

# robot waits for 5 seconds before it attempts the 'dumping' of the health units
robot.sleep(5)

# robot goes straight towards the RePlay logo
robot.straight(640)

# large motor attachment goes down to release the health units into the RePlay logo
attachment_motor.run_angle(100, 140)

# large motor attachment goes back up after releasing health units into the RePlay logo
attachment_motor.run_angle(100, -140)

# robot takes a slight backwards left turn 
robot.turn(-20)

# robot goes backwards as it backs away from the RePlay logo and heads back to base
robot.straight(-620)
