# echo_client.py
# -*- coding:utf-8 -*-

import socket
from dronekit import connect, VehicleMode, LocationGlobal, LocationGlobalRelative
from pymavlink import mavutil  # Needed for command message definitions
import time
import math
import cv2
import numpy
import threading
from socket import *
import RPi.GPIO as GPIO     # RaspberryPi lib
import sys

client_index = 11  # the number of client. Add 1 to use path information(for Home base and to return)
locationsTo_Web = ""    # to send TSP path to Web server
land_point = "Center"

latitude = []
longitude = []

## Raspberry pi setting
# GPIO Mode (BOARD / BCM)
GPIO.setmode(GPIO.BCM)
# set GPIO Pins
GPIO_TRIGGER = 19
GPIO_ECHO = 6
# set GPIO direction (IN / OUT)
GPIO.setup(GPIO_TRIGGER, GPIO.OUT)
GPIO.setup(GPIO_ECHO, GPIO.IN)

vehicle = connect('/dev/ttyAMA0', wait_ready=True, baud=57600)
print("Vehicle Connect")


# get TSP path from TSP HCP server
## not thread
def get_TSP_path():
    global locationsTo_Web
    #   To get shortest visiting path by using HPC TSP algorithm and point
    #   Client socket connection to HPC TSP Server
    msg = tsp_client_socket.recv(256)  # get message from server

    print("Connect Drone to HPC Server!")

    msg = str(msg)
    locations = msg.split('\'')
    locations = locations[1]
    locations = locations.split('\\')
    locations = locations[0]
    locationsTo_Web = locations  # Shortest path for delivery drone
    locations = locations.split('/')
    locations.pop()
    locations = list(map(float, locations))
    print("Path from server : {}".format(locations))  # print message from server

    # to make path
    i = 0
    for i in range(len(locations)):
        if i % 2 == 0:
            latitude.append(locations[i])
        else:
            longitude.append(locations[i])

    for i in range(len(latitude)):
        print('latitude[', i, '] : ', latitude[i], '\tlongitude[', i, '] : ', longitude[i])

    tsp_client_socket.close()

# get distance from obstacle to Drone by using sonar sensor
## not thread
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


## Drone Control function
def arm_and_takeoff(aTargetAltitude):
    """
    Arms vehicle and fly to aTargetAltitude.
    """

    msgTo_webserver("Basic pre-arm checks")
    # Don't try to arm until autopilot is ready
    while not vehicle.is_armable:
        print(" Waiting for vehicle to initialise...")
        time.sleep(1)

    msgTo_webserver("Arming motors")
    # Copter should arm in GUIDED mode
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True

    # Confirm vehicle armed before attempting to take off
    while not vehicle.armed:
        msgTo_webserver(" Waiting for arming...")
        time.sleep(1)

    msgTo_webserver("Taking off!")
    vehicle.simple_takeoff(aTargetAltitude)  # Take off to target altitude

    while True:
        msgTo_webserver(" Altitude: ", vehicle.location.global_relative_frame.alt)
        # Break and return from function just below target altitude.
        if vehicle.location.global_relative_frame.alt >= aTargetAltitude * 0.95:
            msgTo_webserver("Reached target altitude")
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
    global land_point
    try:
        msgTo_webserver("(Go)Take off!")
        arm_and_takeoff(2)  # take off altitude 2M

        i = 4  # start altitude to move 4M

        msgTo_webserver("(Go)Set default/target airspeed to 3")
        vehicle.airspeed = 3

        msgTo_webserver("(Go)Angle Positioning and move toward")  # move to next point

        dist = 1000

        starttime=time.time()
        flytime=0
        while flytime <= 40:

            if dist <= 300 and dist >= 100:  # 3M from obstacle
                msgTo_webserver("(Go)Detect Obstacle")

                i = i + 1
                msgTo_webserver("(Go)Up to :", i)
                while True:
                    msgTo_webserver("(Go)Altitude :", vehicle.location.global_relative_frame.alt)
                    # Break and return from function just below target altitude.
                    send_attitude_target(roll_angle=0.0, pitch_angle=0.0,
                                         yaw_angle=None, yaw_rate=0.0, use_yaw_rate=False,
                                         thrust=0.6)
                    if vehicle.location.global_relative_frame.alt >= i * 0.95:
                        msgTo_webserver("(Go)Reached target altitude")
                        break
                    time.sleep(1)
            else:
                msgTo_webserver("(Go)Go Forward")
                loc_point = LocationGlobalRelative(lati, longi, i)
                vehicle.simple_goto(loc_point, groundspeed=1)
                # Send a new target every two seconds
                # For a complete implementation of follow me you'd want adjust this delay
                time.sleep(1)

            dist = distance()
            if dist >= 100:
                msgTo_webserver("(Go)Vehicle from Obstacle : " + str(dist))
            flytime = time.time() - starttime

        #msgTo_web("(L)Set General Landing Mode")
        #vehicle.mode = VehicleMode("LAND")
        #time.sleep(1)

        time.sleep(3)
        drone_land(lati, longi, land_point)     # image processing landing

        msgTo_webserver("(Go)Close vehicle object")
        vehicle.close()
        msgTo_webserver("(Go)Ready to leave to next Landing Point")
    except KeyboardInterrupt:
        msgTo_webserver("Emergency Land")
        vehicle.mode = VehicleMode("LAND")
        time.sleep(1)
def drone_land(lati, longi, land_point):
        msgTo_webserver("(L)Setting Landing Mode!")

        msgTo_webserver("(L)Set airspeed 1m/s")
        vehicle.airspeed = 1

        print("Target Detect : ", land_point)
        find_point = str(land_point)

        i = vehicle.location.global_relative_frame.alt  # current altitude

        while True:

            i = i - 1

            if find_point == "Center":  # i M from Landing point
                msgTo_webserver("(L)Set Precision Landing Mode(Center)")

                while True:
                    msgTo_webserver("(L)Altitude :", vehicle.location.global_relative_frame.alt)
                    # Break and return from function just below target altitude.
                    send_attitude_target(roll_angle=0.0, pitch_angle=0.0,
                                         yaw_angle=None, yaw_rate=0.0, use_yaw_rate=False,
                                         thrust=0.4)
                    if vehicle.location.global_relative_frame.alt >= i * 0.95:
                        msgTo_webserver("(L)Reached target altitude")
                        break
                    time.sleep(1)
            elif vehicle.location.global_relative_frame.alt<=1:
                msgTo_webserver("(L)Set General Landing Mode")
                vehicle.mode = VehicleMode("LAND")
                time.sleep(1)
                break
            else:
                msgTo_webserver("(L)Finding Landing Point Target")
                msgTo_webserver("(L)Altitude :", vehicle.location.global_relative_frame.alt)
                loc_point = LocationGlobalRelative(lati, longi, i)
                vehicle.simple_goto(loc_point, groundspeed=1)
                # Send a new target every two seconds
                # For a complete implementation of follow me you'd want adjust this delay
                time.sleep(3)


# Using thread to connect HPC image processing server and Web server
## Thread 1
def send_To_HPCimg_server(sock):
    print("Connect to Image Processing Server")
    try:
        # to send drone cam image to HPC image processing server
        # PI camera image capture
        cam = cv2.VideoCapture(0)
        # Frame size 3 = width, 4 = height
        cam.set(3, 1280);
        cam.set(4, 960);
        # image quality range : 0~100, set 90 (default = 95)
        encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 90]

        while True:
            # get 1 frame
            # Success ret = True, Fail ret = False, frame = read frame

            ret, frame = cam.read()

            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

            # cv2. imencode(ext, img [, params])
            # encode_param format, frame to png image encode
            result, frame = cv2.imencode('.png', frame, encode_param)
            # convert frame to String type
            data = numpy.array(frame)
            stringData = data.tobytes()

            # send data to HPC image processing server
            # (str(len(stringData))).encode().ljust(16)
            HPC_clientSocket.sendall((str(len(stringData))).encode().ljust(16) + stringData)

    except:  # when socket connection failed
        print("Socket Close!!")
        cam.release()
        HPC_clientSocket.close()
    finally:
        HPC_clientSocket.close()
## Thread 2
def recv_from_HPCimg_server(sock):
    global land_point
    while True:
        data = HPC_clientSocket.recv(1024)
        land_point = data.decode("utf-8")
        time.sleep(1)


def msgTo_webserver(msg_to_web):  # make message to HPC image processing server
    Web_clientSocket.sendall(str(msg_to_web).encode("utf-8"))
    print(str(msg_to_web))

    data = Web_clientSocket.recv(1024)
    data = str(data).split("b'", 1)[1].rsplit("'", 1)[0]
    print(data)
## Thread 3
# Move drone for TSP path and send log data to Web
def send_Logdata_toWebserver(sock):
    #   To send Drone log, video and other informations to Web Server
    #   Client socket connection to Web Server
    try:
        print("Connect Drone to Web Server!")
        msgTo_webserver(locationsTo_Web)

        num = 0  # Current Target point to send Server

        msgTo_webserver("Start to move")  # convert num to string type     send 1 to server

        # 1  start Drone delivery.    The number of point(including Home base) : 12
        while num < client_index + 1:  # loop 12 times, manipulate it when you test this system
            num = num + 1     # to move first(1) point
            drone_fly(latitude[num], longitude[num])
            point = (latitude[num] + '/' + longitude[num])
            msgTo_webserver(point)
            time.sleep(1)
            if num < client_index:
                vehicle = connect("/dev/ttyAMA0", wait_ready=True, baud=57600)
                msgTo_webserver("Vehicle Reconnect!")
            else:
                msgTo_webserver("Return To Base")

        # 2(Finish Drone delivery)
        msgTo_webserver("Completed to Delivery")
        msgTo_webserver("Finish")

        msgTo_webserver("arrive")
        Web_clientSocket.close()  # close socket connection

        ### End Drone Delivery System

    except:  # when socket connection failed
        print("EMERGENCY LAND!!")
        vehicle.mode = VehicleMode("LAND")
        time.sleep(1)
        print("Close vehicle object")
        vehicle.close()
        Web_clientSocket.close()
    finally:
        Web_clientSocket.close()



if __name__=="__main__":

    # socket connection address and port for HPC TSP server
    # get shortest path data from HPC TSP server
    TSP_SERVER_IP = "116.89.189.55"  # HPC TSP server IP
    TSP_SERVER_PORT = 22042
    SIZE = 512
    tsp_client_socket = socket(AF_INET, SOCK_STREAM)
    tsp_client_socket.connect((TSP_SERVER_IP, TSP_SERVER_PORT))
    # to get TSP path from HPC TSP server
    get_TSP_path()
    time.sleep(5)


    ######## Start flying Drone ########

    ## HPC Image processing Server(Image)
    # send drone cam image to HPC image processing server and get landing data from HPC server
    IMG_SERVER_IP = "192.168.0.6"   # Koren VM Image Processing server IP
    IMG_SERVER_PORT = 22043    # HPC external port 22043
    HPC_clientSocket = socket(AF_INET, SOCK_STREAM)
    HPC_clientSocket.connect((IMG_SERVER_IP, IMG_SERVER_PORT))


    try:
        ## Web Server(Log)
        # send drone log(altitude, arrive point point etc..) to Web server
        Web_SERVER_IP = "192.168.0.6"  # koren SDI VM IP
        Web_SERVER_PORT = 22044
        Web_clientSocket = socket(AF_INET, SOCK_STREAM)
        Web_clientSocket.connect((Web_SERVER_IP, Web_SERVER_PORT))

        try:
            ##   Declare Thread
            # Web Server Thread
            sendLog = threading.Thread(target=send_Logdata_toWebserver, args=(Web_clientSocket,))
            # HPC Image Processing Server Thread
            sendImg = threading.Thread(target=send_To_HPCimg_server, args=(HPC_clientSocket,))
            receiver = threading.Thread(target=recv_from_HPCimg_server, args=(HPC_clientSocket,))

            ##  Start Thread
            # HPC Image Processing Server Thread
            sendImg.start()
            receiver.start()
            # Web Server Thread
            sendLog.start()


            while True:
                time.sleep(1)   # thread 간의 우선순위 관계 없이 다른 thread에게 cpu를 넘겨줌(1 일때)
                pass            # sleep(0)은 cpu 선점권을 풀지 않음
        except:
            HPC_clientSocket.close()
    except:
        Web_clientSocket.close()
