#!/usr/bin/env pybricks-micropython
# from sys import builtin_module_names
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile
import time
import random

# This program requires LEGO EV3 MicroPython v2.0 or higher.
# Click "Open user guide" on the EV3 extension tab for more information.
wheelCirc = (5.3975*3.14159)/100 # meters
robotCirc = (12.065*3.14159)/100 # meters

def goToTarget(right, left, target, velocity = 360):
    resetAngles(right, left)
    desired_angle = (target/wheelCirc)*360
    right.run_angle(velocity, desired_angle, Stop.HOLD, False)
    left.run_angle(velocity, desired_angle)

def resetAngles(right, left, angle = 0):
    right.reset_angle(angle)
    left.reset_angle(angle)

def rotate(right, left, angle=90, velocity = 150):
    ratio = angle/360
    dist = robotCirc * ratio
    desiredAngle = ((dist/wheelCirc) * 360 * (14/15))
    resetAngles(right, left)
    right.run_target(velocity, desiredAngle, Stop.HOLD, False)
    left.run_target(velocity, -desiredAngle)

def alignToWall(sensor, right, left):
    goToTarget(right, left, .05) # move forward a small distance
    rotate(right, left, 60)
    while(True):
        if(sensor.color() == Color.RED):
            while(sensor.color() == Color.RED):
                rotate(right, left, 10)
            else:
                rotate(right, left, -15)
                return True
        rotate(right, left, 10) # Rotate to find the wall
    

def wallFollow(sensor, right, left, dist):
    while Color.RED == sensor.color():
        print(dist)
        if dist >= .5:
            return dist
        else:
            goToTarget(right, left, .05)
            dist = dist + .05
    else:
        # do a rotation to check that we aren't at corner 
        # check to make sure we didn't clear the right side of a wall:
        rotate(right, left, 25) 
        if Color.RED == sensor.color():
            rotate(right, left, -5)
            return wallFollow(sensor, right, left, dist)
        # check left side
        rotate(right, left, -50)
        if Color.RED == sensor.color():
            rotate(right, left, 5)
            return wallFollow(sensor, right, left, dist)
        rotate(right, left, 25)
        # check for perpendicular wall: 
        rotate(right, left, 60)
        for i in range(4):
            rotate(right, left, 15) # rotate left
            if Color.RED == sensor.color(): 
                return wallFollow(sensor, right, left, 0)
        else: 
            print("wall ended, travelled for")
            rotate(right, left, -120) # rotate back right 
            return dist

def findObject(sonicSens, right, left):
    """Find the object by moving the vehicle in multiple directions"""
    for i in range(18):
        rotate(right, left, 20) # 45 degree increments
        dist = sonicSens.distance() / 1000
        print(dist)
        if dist < .6:
            rotate(right, left, 10)
            rotate(right, left, -10)
            goToTarget(right, left, .05)
            goToTarget(right, left, -.05)  
            goToTarget(right, left, dist)  
            return True
        # need to avoid walls and remember direction

def goToObject(right, left, color, distance, angle):
    return 0 

def wander(colorS, sonicS, right, left):
    # choose some direction
    # Keep moving in direction until 
    # 1. move a certain distance 
    # 2. find a wall 
    # 3. find the goal space 
    # needs to an ability to go a certain distance and direction but also check for walls on the way there. 
    distance = 0
    boolean = True
    direction = [0, 90, -90, 180]
    while(boolean):
        # choose some random direction to go in. 
        rotate(right, left, direction[random.randint(0, 2)])
        right.run(360), left.run(360)
        while(colorS.color() != Color.RED and colorS.color() != Color.BLUE):
            continue
        else: 
            right.stop()
            left.stop()
            if(alignToWall(colorS, right, left)):
                distance = wallFollow(colorS, right, left, 0)
                if distance == .5: # we've travelled the wall for too long. 
                    rotate(right, left, 90)
                    goToTarget(right, left, .05)
                    if findObject(sonicS, right, left):
                        boolean = False
                else:  # we cleared the wall
                    goToTarget(right, left, .15)
                    rotate(right, left, -90) # turn right past wall
                    goToTarget(right, left, .15)
                    if findObject(sonicS, right, left):
                        boolean = False
            else:
                rotate(right, left, 180)

# Create your objects here.
def main():
    ev3 = EV3Brick()
    rightMotor = Motor(Port.A, Direction.COUNTERCLOCKWISE)
    leftMotor = Motor(Port.D, Direction.COUNTERCLOCKWISE)
    colorSensor = ColorSensor(Port.S4)
    sonicSensor = UltrasonicSensor(Port.S1)
    # Write your program here.
    ev3.speaker.beep()
    wander(colorSensor, sonicSensor, rightMotor, leftMotor)
main()