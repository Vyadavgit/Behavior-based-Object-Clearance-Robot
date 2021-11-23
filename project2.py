#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile
from collections import deque
import math

COLOR_BLUE = 2
COLOR_RED = 5

wheelCirc = (5.5*3.14159)/100 # meters

ev3 = EV3Brick()

# Initialize the color sensor.
line_sensor = ev3.ColorSensor(#port here)

#Initialize the motors
rightMotor = Motor(Port.A, Direction.COUNTERCLOCKWISE)
leftMotor = Motor(Port.B, Direction.COUNTERCLOCKWISE)

def wander()

def follow_line()
