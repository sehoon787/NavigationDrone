#!/usr/bin/env python
# -*- coding: utf-8 -*-

from dronekit import connect, VehicleMode, LocationGlobal, LocationGlobalRelative
import time
import math
import serial
# import RPi.GPIO as GPIO     # RaspberryPi lib
# import sys


vehicle = connect('/dev/ttyACM0', wait_ready=True, baud=57600)
print("Connect Drone!")

## Lidar
ser = serial.Serial("/dev/ttyS0", 115200)
# connect lidar to raspberry pi 4
if ser.isOpen() == False:
    ser.open()

## Raspberry pi setting
# # GPIO Mode (BOARD / BCM)
# GPIO.setmode(GPIO.BCM)
#
# # set GPIO Pins
# GPIO_TRIGGER = 26
# GPIO_ECHO = 19
#
# # set GPIO direction (IN / OUT)
# GPIO.setup(GPIO_TRIGGER, GPIO.OUT)
# GPIO.setup(GPIO_ECHO, GPIO.IN)

# get distance from obstacle to Drone by using sonar sensor
# def distance():
#     # set Trigger to HIGH
#     GPIO.output(GPIO_TRIGGER, True)
#
#     # set Trigger after 0.01ms to LOW
#     time.sleep(0.00001)
#     GPIO.output(GPIO_TRIGGER, False)
#
#     StartTime = time.time()
#     StopTime = time.time()
#
#     # save StartTime
#     while GPIO.input(GPIO_ECHO) == 0:
#         StartTime = time.time()
#
#     # save time of arrival
#     while GPIO.input(GPIO_ECHO) == 1:
#         StopTime = time.time()
#
#     # time difference between start and arrival
#     TimeElapsed = StopTime - StartTime
#     # multiply with the sonic speed (34300 cm/s)
#     # and divide by 2, because there and back
#     distance = (TimeElapsed * 34300) / 2
#
#     return distance
def distance():
    try:
        while True:
            counter = ser.in_waiting
            if counter > 8:
                bytes_serial = ser.read(9)
                ser.reset_input_buffer()

                if bytes_serial[0] == 0x59 and bytes_serial[1] == 0x59:
                    dist = bytes_serial[2] + bytes_serial[3] * 256
                    ser.reset_input_buffer()
                    return dist
    except Exception as e:
        print("Ignore this Error!!" + str(e))
        return 0

def arm_and_takeoff(aTargetAltitude):
    """
    Arms vehicle and fly to aTargetAltitude.
    """

    print("Basic pre-arm checks")
    # Don't try to arm until autopilot is ready
    while not vehicle.is_armable:
        print(" Waiting for vehicle to initialise...")
        time.sleep(1)

    print("Arming motors")
    # Copter should arm in GUIDED mode
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True

    # Confirm vehicle armed before attempting to take off
    while not vehicle.armed:
        print(" Waiting for arming...")
        time.sleep(1)

    print("Taking off!")
    vehicle.simple_takeoff(aTargetAltitude)  # Take off to target altitude

    while True:
        print(" Altitude: ", vehicle.location.global_relative_frame.alt)
        # Break and return from function just below target altitude.
        if vehicle.location.global_relative_frame.alt >= aTargetAltitude * 0.95:
            print("Reached target altitude")
            break
        time.sleep(1)


def send_attitude_target(roll_angle=0.0, pitch_angle=0.0,
                         yaw_angle=None, yaw_rate=0.0, use_yaw_rate=False,
                         thrust=0.5):
    if yaw_angle is None:
        # this value may be unused by the vehicle, depending on use_yaw_rate
        yaw_angle = vehicle.attitude.yaw
    # Thrust >  0.5: Ascend
    # Thrust == 0.5: Hold the altitude
    # Thrust <  0.5: Descend
    msg = vehicle.message_factory.set_attitude_target_encode(
        0,  # time_boot_ms
        1,  # Target system
        1,  # Target component
        0b00000000 if use_yaw_rate else 0b00000100,
        to_quaternion(roll_angle, pitch_angle, yaw_angle),  # Quaternion
        0,  # Body roll rate in radian
        0,  # Body pitch rate in radian
        math.radians(yaw_rate),  # Body yaw rate in radian/second
        thrust  # Thrust
    )
    vehicle.send_mavlink(msg)


def set_attitude(roll_angle=0.0, pitch_angle=0.0,
                 yaw_angle=None, yaw_rate=0.0, use_yaw_rate=False,
                 thrust=0.5, duration=0):
    send_attitude_target(roll_angle, pitch_angle,
                         yaw_angle, yaw_rate, False,
                         thrust)
    start = time.time()
    while time.time() - start < duration:
        send_attitude_target(roll_angle, pitch_angle,
                             yaw_angle, yaw_rate, False,
                             thrust)
        time.sleep(0.1)
    # Reset attitude, or it will persist for 1s more due to the timeout
    send_attitude_target(0, 0,
                         0, 0, True,
                         thrust)


def to_quaternion(roll=0.0, pitch=0.0, yaw=0.0):
    """
    Convert degrees to quaternions
    """
    t0 = math.cos(math.radians(yaw * 0.5))
    t1 = math.sin(math.radians(yaw * 0.5))
    t2 = math.cos(math.radians(roll * 0.5))
    t3 = math.sin(math.radians(roll * 0.5))
    t4 = math.cos(math.radians(pitch * 0.5))
    t5 = math.sin(math.radians(pitch * 0.5))

    w = t0 * t2 * t4 + t1 * t3 * t5
    x = t0 * t3 * t4 - t1 * t2 * t5
    y = t0 * t2 * t5 + t1 * t3 * t4
    z = t1 * t2 * t4 - t0 * t3 * t5

    return [w, x, y, z]


def drone_fly(lati, longi):
    global dist
    try:
        print("Take off!")
        arm_and_takeoff(2)

        print("Set default/target airspeed to 1")
        vehicle.airspeed = 1

        i = 3  # start altitude

        print("Angle Positioning and move toward")  # move to next point

        starttime = time.time()
        flytime = 0
        while flytime <= 30:

            # dist = distance()
            dist = distance()
            if 150 <= dist:
                print("Vehicle from Obstacle : " + str(dist))

            if 150 <= dist <= 400:  # 3M
                print("Detect Obstacle")

                i = i + 1
                print("Up to :", i)

                while True:
                    print(" Altitude :", vehicle.location.global_relative_frame.alt)
                    # Break and return from function just below target altitude.
                    send_attitude_target(roll_angle=0.0, pitch_angle=0.0,
                                         yaw_angle=None, yaw_rate=0.0, use_yaw_rate=False,
                                         thrust=0.7)
                    if vehicle.location.global_relative_frame.alt >= i * 0.95:
                        print("Reached target altitude")
                        break
                    time.sleep(1)
                    dist = 0
            else:
                print("Go Forward")
                loc_point = LocationGlobalRelative(lati, longi, i)
                vehicle.simple_goto(loc_point, groundspeed=1)
                # For a complete implementation of follow me you'd want adjust this delay
                time.sleep(1)

            flytime = time.time() - starttime

        print("Setting LAND mode...")
        vehicle.mode = VehicleMode("LAND")
        time.sleep(1)

        print("Close vehicle object")
        vehicle.close()
        print("Ready to leave to next Landing Point")
    except KeyboardInterrupt:
        print("Emergency Land")
        vehicle.mode = VehicleMode("LAND")
        time.sleep(5)



print("Going towards first point for 40 seconds ...")
drone_fly(35.133168, 129.106331)
#35.133168, 129.106331---target 4

print("sleep 5 sec")
time.sleep(5)

#return
vehicle = connect('/dev/ttyACM0', wait_ready=True, baud=57600)

print("Going towards home point for 20 seconds ...")
drone_fly(35.132833, 129.106215)
#35.132833, 129.106215---homepoint

print("Close vehicle object")
vehicle.close()
