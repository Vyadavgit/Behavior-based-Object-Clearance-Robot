#!/usr/bin/env pybricks-micropython
# from sys import builtin_module_names
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.nxtdevices import (LightSensor)
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
wallColor = Color.RED
goalColor = Color.BLUE
ev3 = EV3Brick()


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
    rotate(right, left, 25)
    for i in range(10):
        if(sensor.color() == wallColor):
            while(sensor.color() == wallColor):
                rotate(right, left, 10)
            else:
                rotate(right, left, -5)
                return True
        rotate(right, left, 10) # Rotate to find the wall
    else:
        return False
    

def wallFollow(sensor, leftSensor, right, left, dist):
    while wallColor == sensor.color():
        if dist >= 1.5:
            return 1.5
        if leftSensor.color() == wallColor:
            goToTarget(right, left, .05)
            rotate(right, left, 60)
            while sensor.color() != wallColor: 
                rotate(right, left, 5)
            else:
                rotate(right, left, 5)
                return wallFollow(sensor, leftSensor, right, left, 0)
        else:
            goToTarget(right, left, .05)
            dist = dist + .05
    else:
        # check to make sure we didn't clear the right side of a wall:
        rotate(right, left, 15) 
        if wallColor == sensor.color():
            rotate(right, left, -5)
            return wallFollow(sensor, leftSensor, right, left, dist)
        # check left side
        rotate(right, left, -30)
        if wallColor == sensor.color():
            rotate(right, left, 5)
            return wallFollow(sensor, leftSensor, right, left, dist)
        rotate(right, left, 15)
        # do a check that to make sure we aren't at corner 
        goToTarget(right, left, -.05)
        if wallColor == leftSensor.color():
            goToTarget(right, left, .05)
            rotate(right, left, 60)
            while sensor.color() != wallColor: 
                rotate(right, left, 5)
            else:
                rotate(right, left, 5)
                return wallFollow(sensor, leftSensor, right, left, 0)
        goToTarget(right, left, .05)
        return dist

def findObject(sonicSens, right, left):
    """Find the object by moving the vehicle in multiple directions"""
    for i in range(18):
        rotate(right, left, 20) # 20 degree increments
        dist = sonicSens.distance() / 1000
        if dist < .6:
            return dist
    return 1
        # need to avoid walls and remember direction

def goToObject(right, left, rightSensor, leftSensor, sonar, distance):
    # goToTarget(right, left, distance)
    while True:
        if(rightSensor.color() == wallColor or rightSensor.color() == goalColor) or (leftSensor.color() == wallColor or rightSensor.color() == goalColor):
            break
        else:
            goToTarget(right, left, .05)
    else: 
        right.stop()
        left.stop()
        if(rightSensor == wallColor or leftSensor == wallColor):
            #wall follow 
            alignToWall(rightSensor, right, left)
            dist = wallFollow(rightSensor, leftSensor, right, left, 0)
            if (dist == 1.5): 
                return False
            else:  # we cleared the wall look for object again
                goToTarget(right, left, .15)
                rotate(right, left, 90)
                goToObject(right, left, rightSensor, leftSensor, sonar, findObject(sonar, right, left))
        else: 
            # find the target within the goal, and move and knock it off.
            ev3.speaker.play_file(SoundFile.FANFARE)
            while(sonar.distance() > .305):
                rotate(right, left, 5)
            else:
                while(rightSensor.color() == goalColor and leftSensor.color() == goalColor):
                    goToTarget(right, left, .60)
            return True

    return False

def wander(colorS, lColorS, sonicS, right, left):
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
        while(colorS.color() != goalColor and colorS.color() != wallColor):
            continue
        else: 
            right.stop()
            left.stop()
            if(colorS.color() == wallColor):
                if not alignToWall(colorS, right, left):
                    goToTarget(right, left, -.2)
                    continue
                distance = wallFollow(colorS, lColorS, right, left, 0)
                if distance == 1.5: # we've travelled the wall for too long. 
                    rotate(right, left, 90)
                    goToTarget(right, left, .1)
                    distFromObject = findObject(sonicS, right, left)
                    if distFromObject < 1:
                        if goToObject(right, left, colorS, lColorS, sonicS, distFromObject):
                            boolean = False
                        else:
                            continue
                else:  # we cleared the wall
                    goToTarget(right, left, .15)
                    rotate(right, left, -90) # turn right past wall
                    goToTarget(right, left, .15)
                    distFromObject = findObject(sonicS, right, left)
                    if distFromObject < 1:
                        if goToObject(right, left, colorS, lColorS, sonicS, distFromObject):
                            boolean = False
                        else: 
                            continue
            else:
                ev3.speaker.play_file(SoundFile.FANFARE)


# Create your objects here.
def main():
    rightMotor = Motor(Port.A, Direction.COUNTERCLOCKWISE)
    leftMotor = Motor(Port.D, Direction.COUNTERCLOCKWISE)
    colorSensor = ColorSensor(Port.S4)
    leftColorSensor = ColorSensor(Port.S2)
    sonicSensor = UltrasonicSensor(Port.S1)
    # Write your program here.
    ev3.speaker.beep()
    wander(colorSensor, leftColorSensor, sonicSensor, rightMotor, leftMotor)

main()