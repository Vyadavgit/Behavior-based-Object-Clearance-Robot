#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile
import time

# This program requires LEGO EV3 MicroPython v2.0 or higher.
# Click "Open user guide" on the EV3 extension tab for more information.
wheelCirc = (5.5*3.14159)/100 # meters
def goToTarget(right, left, target, velocity = 360, wheelCirc = (5.5*3.14159)/100):
    resetAngles(right, left)
    desired_angle = (target/wheelCirc)*360
    right.run_angle(velocity, desired_angle, Stop.HOLD, False)
    left.run_angle(velocity, desired_angle)

def resetAngles(right, left, angle = 0):
    right.reset_angle(angle)
    left.reset_angle(angle)

def rotate(right, left, angle=420, velocity = 150):
    resetAngles(right, left)
    right.run_target(velocity, angle, Stop.HOLD, False)
    left.run_target(velocity, -angle)

def wander():
    # choose some direction
    # Keep moving in direction until 1. move a certain distance 2. find a wall or 3. find the goal space 

    return 0 

# Wall follow will follow a wall for 60cm, if the wall is not cleared, it will return 60cm. 
# Else wall follow will return how far the robot followed the wall. 
# if we move a certain distance, look for object
    # if we cant find the object choose random direction and repeat 
# if we find a wall call wallFollow 
    # if wall follow (ret val ) < .6 turn right 
        # find new direction to move
    # else turn left 
        # look for object 
        # etc
# if we 
def wallFollow(color, sensor):
    dist = 0
    while color == sensor.color():
        goToTarget(rightMotor, leftMotor, .05)
    else: 
        return dist


# Create your objects here.
ev3 = EV3Brick()
rightMotor = Motor(Port.A, Direction.COUNTERCLOCKWISE)
leftMotor = Motor(Port.D, Direction.COUNTERCLOCKWISE)
colorSensor = ColorSensor(Port.S4)
sonicSensor = UltrasonicSensor(Port.S1)

# Write your program here.
ev3.speaker.beep()
tileColor = colorSensor.color()  # Tile color
boolean = True
# while(boolean):
#     if tileColor == colorSensor.color():
#         goToTarget(rightMotor, leftMotor, .05)
#     else: 
#         rotate(rightMotor, leftMotor, 180)
#         wallFollow(colorSensor.color(), colorSensor)
#         boolean = False
        
while(boolean):
    print(colorSensor.color())
    print(sonicSensor.distance()/10)
    time.sleep(1)