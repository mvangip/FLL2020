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



def waitButtons():
    while True:
        ev3.light.on(Color.GREEN)
        if Button.UP in ev3.buttons.pressed(): 
            RedMission1()
        if Button.LEFT in ev3.buttons.pressed():
            YellowMission()
        if Button.DOWN in ev3.buttons.pressed():
            BlueMission()
        if Button.RIGHT in ev3.buttons.pressed():
            BlackandGreen()

def YellowMission():
    
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

    robot.stop(Stop.BRAKE)

    robot.drive(100, 0)
    if line_sensor2 and cl.Color() == Color.BLACK:
            robot.stop(Stop.BRAKE)
    
        

    # robot stops after finishing up line following code
    robot.stop(Stop.BRAKE)

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

def BlueMission():
    
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
    #robot.settings(200)
    #go front towards the step counter
    robot.straight(650)
    robot.stop(Stop.BRAKE)
    wait(20)

    #makes the robot go slower 
    robot.settings(40)

    #slowly pushes the step counter by going back and front 2 times
    robot.straight(140)
    robot.stop(Stop.BRAKE)
    robot.straight(-45)
    robot.stop(Stop.BRAKE)
    robot.straight(120)
    robot.stop(Stop.BRAKE)
    robot.settings(100)
    robot.straight(-30)
    robot.stop(Stop.BRAKE)
    robot.settings(200)
    #the robot then turns and goes backwards
    robot.turn(45)
    robot.straight(-100)
    # the robot then goes back until the right color sensor detects back
    while True:
        robot.drive(-30,0)
        if line_sensor.color() == Color.BLACK:
            robot.stop(Stop.BRAKE)
            break 
    #the large motor attatchment comes down at the same time the robot takes a turn towards the black line underneath the pull up bar
    large_motor.run_angle(50,200,then=Stop.HOLD, wait=False)
    left_motor.run_angle(50,-300,then=Stop.HOLD, wait=True)
    #the robot then goes straight towards that line
    #robot.straight(120)
    robot.straight(130)
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
    large_motor.run_angle(75,-65)
    robot.straight(-60)
    #the robot then takes a turn (at the same time bringing the attatchment down) towards the slide mission and completes the mission
    large_motor.run_angle(50,80,then=Stop.HOLD, wait=False)
    robot.turn(-165)
    robot.straight(175)
    large_motor.run_angle(300, -120, then=Stop.HOLD, wait=True)
    robot.straight(-15)
    large_motor.run_angle(550, 120, then=Stop.HOLD, wait=True)
    ## The robot moves straight towards the mission, getting ready to attempt to push the slide figures off once more. (In case it didn't work before.)
    robot.straight(35)
    large_motor.run_angle(550, -120, then=Stop.HOLD, wait=True)
    ev3.speaker.beep()
    large_motor.run_angle(300, 30, then=Stop.HOLD, wait=True)
    robot.straight(30)
    #goes to collect the health unit near the basketball (goes back to base)
    robot.straight(-200)
    robot.turn(45)
    ev3.speaker.beep()
    robot.straight(1000)
        
def BlackandGreen():

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
    #follows the line underneath the pull up bar until the leftsensor detects black
    BLACK = 9
    WHITE = 85
    threshold = (BLACK + WHITE) / 2
    # Set the drive speed at 100 millimeters per second.
    DRIVE_SPEED = 100
    # Set the gain of the proportional line controller. This means that for every
    PROPORTIONAL_GAIN = 1.2
    runWhile = True
    #goes straight to get ready for line following then resets the distance
    robot.straight(250)
    robot.reset()
    #starts to follow the line towards the replay logo
    while True:
        # Calculate the deviation from the threshold.
        deviation = line_sensor.reflection() - threshold
        # Calculate the turn rate.
        turn_rate = PROPORTIONAL_GAIN * deviation
        # Set the drive base speed and turn rate.
        robot.drive(DRIVE_SPEED, turn_rate)
        wait(10)
        print(robot.distance())
        if(robot.distance ()>= 450):
            robot.stop(Stop.BRAKE)
            break
    #the robot pushes the phone into the replay logo and moves back to get ready to drop the health units into the replay logo
    robot.straight(-75)
    robot.stop(Stop.BRAKE)
    #the robot then turns so it is going to be perfectly into the replay logo
    robot.turn(-35) 
    #the robot drops the health units
    large_motor.run_angle(100,150) 
    #then turns to an angle to go back to base
    robot.turn(50)
    robot.straight(-1000) 

    wait(50)

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
    axle_track = 114.3 

    robot = DriveBase(left_motor, right_motor, wheel_diameter, axle_track)

    ## Write your code here: 

    ## The robot goes straight until the Boccia Mission's target. 
    robot.straight(1060)

    ## The robot moves the large motor down to drop the cubes in the target. 
    front_largeMotor.run_angle(80, 70, then=Stop.HOLD, wait=True)
    front_largeMotor.run_angle(-80, 70, then=Stop.HOLD, wait=True)


    ## Dance Mission

    ## The robot moves backwards to reach the Dance Floor so it can Dance as the last mission. 
    robot.straight(-185)
    robot.turn(-70)
    robot.straight(138) 


    ## The following code is all the dance moves we do for the Dance Mission. 

    robot.turn(160)
    robot.turn(-160)
    robot.straight(60)
    front_largeMotor.run_target(500, 60)
    front_largeMotor.run_target(500, -40)
    robot.straight(-60)
    robot.turn(260)
    robot.turn(-260)
    robot.turn(100)
    robot.straight(40)
    robot.turn(100)
    front_largeMotor.run_angle(500, 30)

def RedMission1(): 
    
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

    BLACK = 11
    WHITE = 85
    threshold = (BLACK + WHITE) / 2

    ######

    robot.straight(320)
    robot.turn(110)
        
    while True:
        robot.drive(90,0)
        if leftcolorsensor.reflection() <= 12:
            robot.stop(Stop.BRAKE)
            break        

    robot.turn(-110)

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
        if rightcolorsensor.color() == Color.BLACK:
            ev3.speaker.beep()
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

    robot.straight(365)
    robot.turn(155)

    robot.straight(54)
    ##Large_Motor.run_angle(-40,150,then = Stop.HOLD, wait = True)

    ##robot.straight(20)

    Medium_Motor.run_angle(150,250,then = Stop.HOLD, wait = True)
    robot.turn(80)
    Medium_Motor.run_angle(-150,250,then = Stop.HOLD, wait = True)
    robot.turn(-30)
    robot.straight(-40)
    Medium_Motor.run_angle(150,250,then = Stop.HOLD, wait = True)

    robot.turn(30)
    robot.straight(-330)

waitButtons()
