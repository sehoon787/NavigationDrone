#!/usr/bin/env python
# -*- coding: utf-8 -*-

from __future__ import print_function
import math
import time
import RPi.GPIO as GPIO  # RaspberryPi lib

## Raspberry pi setting
# GPIO Mode (BOARD / BCM)
GPIO.setmode(GPIO.BCM)

# set GPIO Pins
GPIO_TRIGGER = 26
GPIO_ECHO = 19

# set GPIO direction (IN / OUT)

GPIO.setup(GPIO_TRIGGER, GPIO.OUT)
GPIO.setup(GPIO_ECHO, GPIO.IN)

# get distance from obstacle to Drone by using sonar sensor
def distance():
    # set Trigger to HIGH
    GPIO.output(GPIO_TRIGGER, True)

    # set Trigger after 0.01ms to LOW
    time.sleep(0.00001)
    GPIO.output(GPIO_TRIGGER, False)

    StartTime = time.time()
    StopTime = time.time()

    # save StartTime
    while GPIO.input(GPIO_ECHO) == 0:
        StartTime = time.time()

    # save time of arrival
    while GPIO.input(GPIO_ECHO) == 1:
        StopTime = time.time()

    # time difference between start and arrival
    TimeElapsed = StopTime - StartTime
    # multiply with the sonic speed (34300 cm/s)
    # and divide by 2, because there and back
    distance = (TimeElapsed * 34300) / 2

    return distance


def drone_fly():
    print("Take off!")


    print("Set default/target airspeed to 1")


    i = 4  # start altitude

    print("Angle Positioning and move toward")  # move to next point

    dist = 1000

    starttime=time.time()
    flytime=0
    while flytime<=40:

        if dist<=300 and dist>=100:  # 3M
            print("Detect Obstacle")

            i = i + 1
            print("Up to :", i)

            print(" Altitude : ", i)
            i+1

            time.sleep(1)
        else:
            print("Go Forward")
            # Send a new target every two seconds
            # For a complete implementation of follow me you'd want adjust this delay
            time.sleep(1)

        dist = distance()
        if dist >= 100:
            print("Vehicle from Obstacle : " + str(dist))
        flytime=time.time()-starttime


    print("Setting LAND mode...")
    time.sleep(1)

    print("Close vehicle object")
    print("Ready to leave to next Landing Point")

drone_fly()

drone_fly()

drone_fly()

GPIO.cleanup()
