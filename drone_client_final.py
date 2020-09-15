# echo_client.py
# -*- coding:utf-8 -*-

import socket
import numpy
from dronekit import connect, VehicleMode, LocationGlobal, LocationGlobalRelative
from pymavlink import mavutil  # Needed for command message definitions
import time
import math
import cv2
import numpy
import threading

client_index = 11  # the number of client. Add 1 to use path information(for Home base and to return)

# socket connection address and port for HPC TSP server
# get shortest path data from HPC TSP server
SERVER_IP = '10.100.201.132'  # HPC TSP server IP
SERVER_PORT = 10002
SIZE = 512
SERVER_ADDR = (SERVER_IP, SERVER_PORT)

# socket connection address and port for Web server
# send drone log(altitude, arrive point point etc..) to Web server
SERVER_IP2 = '116.89.189.55'    # koren SDI VM IP
SERVER_PORT2 = 22044
SIZE = 512
SERVER_ADDR2 = (SERVER_IP2, SERVER_PORT2)

# to send drone cam image to HPC image processing server
SERVER_IP3 = '10.100.201.132'   # HPC Image Processing server IP
SERVER_PORT3 = 10003

num = 0  # To make path
locations = []
locationsTo_Web = []
msgTo_web = []

#   To get shortest visiting path by using HPC TSP algorithm and point
#   Client socket connection to HPC TSP Server
with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as client_socket:
    client_socket.connect(SERVER_ADDR)  # connect to server
    msg = client_socket.recv(256)  # get message from server

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
    longitude = []
    latitude = []
    for i in range(len(locations)):
        if i % 2 == 0:
            latitude.append(locations[i])
        else:
            longitude.append(locations[i])

    for i in range(len(latitude)):
        print('latitude[', i, '] : ', latitude[i], '\tlongitude[', i, '] : ', longitude[i])


# Using thread to connect HPC image processing server and Web server
# Thread 1
def sendImage_toHPC(port):
    try:
        ## Connect to HPC image processing server
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        ## server ip, port
        s.connect((SERVER_IP3, SERVER_PORT3))

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
            s.sendall((str(len(stringData))).encode().ljust(16) + stringData)

    except socket.error:  # when socket connection failed
        print("Socket Close!!")
        cam.release()
        s.close()
    finally:
        s.close()

# Thread 2
def sendLogdata_toWebserver(port):
    #   To send Drone log, video and other informations to Web Server
    #   Client socket connection to Web Server
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as client_socket:
            client_socket.connect(SERVER_ADDR2)  # connect to HPC image processing server

            def msgTo_webserver(msg_to_web):  # make message to HPC image processing server
                msgTo_web = msg_to_web
                client_socket.send(msgTo_web.encode('utf-8'))
                print(msgTo_web)

            msgTo_webserver("Connect Drone to Web Server!\n\n")
            msgTo_webserver(locationsTo_Web)

            vehicle = connect('/dev/ttyAMA0', wait_ready=True, baud=57600)
            msgTo_webserver("Connect Drone!")


            def arm_and_takeoff_nogps(aTargetAltitude):
                ##### CONSTANTS #####
                DEFAULT_TAKEOFF_THRUST = 0.6
                SMOOTH_TAKEOFF_THRUST = 0.6

                msgTo_webserver("Basic pre-arm checks")
                while not vehicle.is_armable:
                    msgTo_webserver(" Waiting for vehicle to initialise...")
                    time.sleep(1)

                msgTo_webserver("Arming motors")
                vehicle.mode = VehicleMode("GUIDED_NOGPS")
                vehicle.armed = True

                while not vehicle.armed:
                    msgTo_webserver(" Waiting for arming...")
                    vehicle.armed = True
                    time.sleep(1)

                msgTo_webserver("Taking off!")

                thrust = DEFAULT_TAKEOFF_THRUST
                while True:
                    current_altitude = vehicle.location.global_relative_frame.alt
                    msgTo_webserver(" Altitude: %f  Desired: %f" %
                                    (current_altitude, aTargetAltitude))
                    if current_altitude >= aTargetAltitude * 0.95:
                        msgTo_webserver("Reached target altitude")
                        break
                    elif current_altitude >= aTargetAltitude * 0.6:
                        thrust = SMOOTH_TAKEOFF_THRUST
                    set_attitude(thrust=thrust)
                    time.sleep(0.2)


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
                vehicle = connect('/dev/ttyAMA0', wait_ready=True, baud=57600)
                msgTo_webserver("Take off!")
                arm_and_takeoff_nogps(2)

                msgTo_webserver("Angle Positioning and move toward")  # move to next point
                loc_point = LocationGlobalRelative(lati, longi, 4)
                vehicle.simple_goto(loc_point, groundspeed=3)
                time.sleep(40)

                msgTo_webserver("Setting LAND mode...")
                vehicle.mode = VehicleMode("LAND")
                time.sleep(1)

                msgTo_webserver("Close vehicle object")
                vehicle.close()
                msgTo_webserver("Ready to leave to next Landing Point")


            num2 = 0  # Current Target point to send Server

            msgTo_webserver('Start to move')  # convert num to string type     send 1 to server

            # 1  start Drone delivery.    The number of point(including Home base) : 12
            while num2 < client_index + 1:  # loop 12 times, manipulate it when you test this system
                num2 = num2 + 1     # to move first(1) point
                drone_fly(latitude[num2], longitude[num2])
                point = (latitude[num2] + '/' + longitude[num2])
                point = str(point)
                msgTo_webserver(point)
                time.sleep(3)

            # 2(Finish Drone delivery)
            vehicle.mode = VehicleMode("RTL")
            msgTo_webserver("Completed to Base")
            msgTo_webserver("Finish")

            point = 'arrive'
            client_socket.send(point.encode('utf-8'))  # Socket conection finish
            client_socket.close()  # close socket connection


    except socket.error:  # when socket connection failed
        msgTo_webserver("EMERGENCY LAND!!")
        vehicle.mode = VehicleMode("LAND")
        time.sleep(1)
        msgTo_webserver("Close vehicle object")
        vehicle.close()
        client_socket.close()
    finally:
        client_socket.close()


if __name__=="__main__":
    thraed_sendLog = threading.Thread(target=sendLogdata_toWebserver, args=(SERVER_PORT2,))
    thraed_sendLog.start()

    thraed_sendImage = threading.Thread(target=sendImage_toHPC, args=(SERVER_PORT3,))
    thraed_sendImage.start()

