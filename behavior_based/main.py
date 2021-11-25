#!/usr/bin/env pybricks-micropython
from sys import builtin_module_names
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


def objectFound(distance, color):
    if distance <= 0.3048 and color=="RED": # i.e., 1 ft
        ev3.speaker.beep()
        return 1
    else: 
        return 0

def wander(distance, sensor, right, left):
    # choose some direction
    # Keep moving in direction until 
    # 1. move a certain distance 
    # 2. find a wall 
    # 3. find the goal space 

    if distance==0.6:
        for i in range(4): # 30, 60, 90, 120, 150 degrees away from wall. 
                           # Not checking for 180 because already checked in that direction while wall following
            rotate(right, left, 30, 150) # 30 degrees left
            if objectFound(sonicSensor.distance(), colorSensor.color()) == 1:
                goToTarget(right, left, sonicSensor.distance()) # move towards the object
        rotate(right, left, -60, 150) # (+150-60=90 degrees away from wall) if object not found while exploring 30, 60, 90, 120, 150 degrees away from wall,
                                      # rotate back so that it is 90 degrees left away form the wall
    while(sensor.color() != "BLUE" or sensor.color() != "RED"):
        goToTarget(right, left, 100) # 100 is arbitrary value to keep it moving forward until it senses a color
    
    if sensor.color() =="BLUE":
        rotate(right, left, 90, 150) # 90 degrees left turn
        wallFollow(sensor.color(), sensor)
    # elif sensor.color()=="RED":
    #     # call 'goal finding'
