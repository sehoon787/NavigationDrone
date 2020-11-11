# echo_client.py
# -*- coding:utf-8 -*-

from dronekit import connect, VehicleMode, LocationGlobal, LocationGlobalRelative
from pymavlink import mavutil  # Needed for command message definitions
import time
import math
import cv2
import numpy
import threading
from socket import *
import serial       # for Lidar
import RPi.GPIO as GPIO     # RaspberryPi lib
import Adafruit_PCA9685

pi = 3.1415926535897932384626433832795028841971693993751

client_index = 6  # the number of client. Add 1 to use path information(for Home base and to return)
locationsTo_Web = ""    # to send TSP path to Web server
distanceTo_Web = ""   # to send point to point fly time to Web server
# land_point = "Land"

clat = 0
clong = 0
calt = 0

latitude = []
longitude = []
flydistance = []    # point to point distances
visitOrder = 0

## Raspberry pi setting
ser = serial.Serial("/dev/ttyS0", 115200)

vehicle = connect('/dev/ttyACM0', wait_ready=True, baud=57600)
print("Vehicle Connect")

# get Radian
def toRad(degree):
    return degree / 180 * pi
# Calculate point to point distances by using formula
def calculateDistance(lat1, long1, lat2, long2):
    flydist = math.sin(toRad(lat1)) * math.sin(toRad(lat2)) + \
              math.cos(toRad(lat1)) * math.cos(toRad(lat2)) * math.cos(toRad(long2 - long1))
    flydist = math.acos(flydist)
    #  distance = (6371 * pi * dist) / 180;
    #  got dist in radian, no need to change back to degree and convert to rad again.
    flydist = 6371000 * flydist
    return flydist

# get TSP path from TSP HCP server
## not thread
def get_TSP_path():
    global locationsTo_Web, distanceTo_Web, flydistance
    #   To get shortest visiting path by using HPC TSP algorithm and point
    #   Client socket connection to HPC TSP Server
    msg = tsp_client_socket.recv(256)  # get message from server

    print("Connect Drone to HPC Server!")

    msg = str(msg)
    locations = msg.split('\'')
    locations = locations[1]
    locations = locations.split('\\')
    locations = locations[0]
    locationsTo_Web = locationsTo_Web + locations  # Shortest path for delivery drone
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

    for i in range(client_index+1):
        print('latitude[', i, '] : ', latitude[i], '\tlongitude[', i, '] : ', longitude[i])
        # Calculate distance point to point
        if i < client_index:
            temp = calculateDistance(latitude[i], longitude[i], latitude[i + 1], longitude[i + 1])
            flydistance.append(temp)

    flydistance = list(map(str, flydistance))
    distanceTo_Web = distanceTo_Web + "/".join(flydistance)

    tsp_client_socket.close()
    time.sleep(1)

# get distance from obstacle to Drone by using sonar sensor
## not thread
def distance():
    try:
        while True:
            counter = ser.in_waiting
            if counter > 8:
                bytes_serial = ser.read(9)
                ser.reset_input_buffer()

                if bytes_serial[0] == 0x59 and bytes_serial[1] == 0x59:
                    dist = bytes_serial[2] + bytes_serial[3] * 256
                    if 150 <= dist <= 400:
                        print("Vehicle to Obstacle : " + str(dist))
                    ser.reset_input_buffer()
                    return dist
    except Exception as e:
        print("Ignore this Error!!" + str(e))
        return 0
## Drone Control function
def arm_and_takeoff(aTargetAltitude):
    """
    Arms vehicle and fly to aTargetAltitude.
    """

    msgTo_log_server("Basic pre-arm checks")
    # Don't try to arm until autopilot is ready
    while not vehicle.is_armable:
        msgTo_log_server(" Waiting for vehicle to initialise...")
        time.sleep(1)

    msgTo_log_server("Arming motors")
    # Copter should arm in GUIDED mode
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True

    # Confirm vehicle armed before attempting to take off
    while not vehicle.armed:
        msgTo_log_server(" Waiting for arming...")
        time.sleep(1)

    msgTo_log_server("Take off!")
    vehicle.simple_takeoff(aTargetAltitude)  # Take off to target altitude

    while True:
        print(" Altitude: ", vehicle.location.global_relative_frame.alt)
        # Break and return from function just below target altitude.
        if vehicle.location.global_relative_frame.alt >= aTargetAltitude * 0.95:
            msgTo_log_server("Reached target altitude")
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
    global clat, clong, calt, visitOrder
    try:
        arm_and_takeoff(2)  # take off altitude 2M

        i = 3  # start altitude to move 3M

        msgTo_log_server("(Go)Set default/target airspeed to 1")
        airspeed = 1
        vehicle.airspeed = airspeed

        msgTo_log_server("(Go)Angle Positioning and move toward")  # move to next point

        starttime=time.time()
        flytime=0
        endtime = int(flydistance[visitOrder])/int(airspeed) + 5
        visitOrder = visitOrder + 1
        msgTo_log_server("(Go)Flying time : " + str(endtime-5))

        while flytime <= endtime:

            dist = distance()
            if 150 <= dist:
                temp_lat = vehicle.location.global_relative_frame.lat
                temp_long = vehicle.location.global_relative_frame.lon

            if 150 <= dist <= 600:  # 6M from obstacle
                msgTo_log_server("(Go)Detect Obstacle to " + str(dist) + "M")

                i = i + 1
                msgTo_log_server("(Go)Up to : " + str(i))
                while True:
                    msgTo_log_server("(Go)Altitude : " + str(vehicle.location.global_relative_frame.alt))
                    # Break and return from function just below target altitude.
                    # send_attitude_target(roll_angle=0.0, pitch_angle=0.0,
                    #                      yaw_angle=None, yaw_rate=0.0, use_yaw_rate=False,
                    #                      thrust=0.6)
                    loc_point = LocationGlobalRelative(temp_lat, temp_long, i)
                    vehicle.simple_goto(loc_point, groundspeed=1)

                    clat = vehicle.location.global_relative_frame.lat
                    clong = vehicle.location.global_relative_frame.lon
                    calt = vehicle.location.global_relative_frame.alt

                    if vehicle.location.global_relative_frame.alt >= i * 0.95:
                        msgTo_log_server("(Go)Reached target altitude")
                        dist = 0
                        break
                    time.sleep(1)

            else:
                msgTo_log_server("(Go)Go Forward")
                loc_point = LocationGlobalRelative(lati, longi, i)
                vehicle.simple_goto(loc_point, groundspeed=1)
                clat = vehicle.location.global_relative_frame.lat
                clong = vehicle.location.global_relative_frame.lon
                calt = vehicle.location.global_relative_frame.alt
                time.sleep(1)

            flytime = time.time() - starttime
            # For a complete implementation of follow me you'd want adjust this delay

        #msgTo_web("(L)Set General Landing Mode")
        #vehicle.mode = VehicleMode("LAND")
        #time.sleep(1)

        drone_land(lati, longi)     # image processing landing

        msgTo_log_server("(Go)Close vehicle object")
        vehicle.close()

        # put mini cargo on landing point
        put_cargo(visitOrder)

        msgTo_log_server("(Go)Ready to leave to next Landing Point")
    except Exception as e:  # when socket connection failed
        print(e)
        print("EMERGENCY LAND!!")
        vehicle.mode = VehicleMode("LAND")
        time.sleep(1)
        print("Close vehicle object")
        vehicle.close()
        GPIO.cleanup()
    except KeyboardInterrupt:
        msgTo_log_server("EMERGENCY LAND!!")
        vehicle.mode = VehicleMode("LAND")
        time.sleep(1)
        msgTo_log_server("Close vehicle object")
        vehicle.close()
        GPIO.cleanup()
def drone_land(lati, longi):
    global clat, clong, calt
    try:
        msgTo_log_server("(L)Setting Landing Mode!")

        msgTo_log_server("(L)Set airspeed 1m/s")
        vehicle.airspeed = 1

        # msgTo_log_server("(L)Target Panel Detect : " + str(land_point))
        # find_point = str(land_point)

        i = vehicle.location.global_relative_frame.alt  # current altitude

        while True:

            i = i - 0.5
            # print(find_point)       # to print center or not

            if vehicle.location.global_relative_frame.alt <= 2:     # if altitude is less than 1m

                time.sleep(1)

                msgTo_log_server("(L)Set General Landing Mode")
                vehicle.mode = VehicleMode("LAND")

                clat = vehicle.location.global_relative_frame.lat
                clong = vehicle.location.global_relative_frame.lon
                calt = vehicle.location.global_relative_frame.alt

                time.sleep(1)
                break
            # down to i-1 M from Landing point, Drone on right landing point
            elif lati == vehicle.location.global_relative_frame.lat and longi == vehicle.location.global_relative_frame.lon:
                msgTo_log_server("(L)Simple descending Landing Mode(Center)")

                while True:
                    msgTo_log_server("(L)Altitude : " + str(vehicle.location.global_relative_frame.alt))
                    # Break and return from function just below target altitude.
                    send_attitude_target(roll_angle=0.0, pitch_angle=0.0,
                                         yaw_angle=None, yaw_rate=0.0, use_yaw_rate=False,
                                         thrust=0.4)
                    if vehicle.location.global_relative_frame.alt >= i * 0.95:
                        msgTo_log_server("(L)Reached target altitude")
                        break

                    clat = vehicle.location.global_relative_frame.lat
                    clong = vehicle.location.global_relative_frame.lon
                    calt = vehicle.location.global_relative_frame.alt

                    time.sleep(1)

            else:       # if Drone is not on right landing point, then move to right point
                msgTo_log_server("(L)Precision Landing Mode(Out of Target)")

                msgTo_log_server("(L)Altitude : " + str(vehicle.location.global_relative_frame.alt))
                loc_point = LocationGlobalRelative(lati, longi, i)
                vehicle.simple_goto(loc_point, groundspeed=1)

                clat = vehicle.location.global_relative_frame.lat
                clong = vehicle.location.global_relative_frame.lon
                calt = vehicle.location.global_relative_frame.alt

                # Send a new target every two seconds
                # For a complete implementation of follow me you'd want adjust this delay
                time.sleep(1)

    except Exception as e:  # when socket connection failed
        print(e)
        print("EMERGENCY LAND!!")
        vehicle.mode = VehicleMode("LAND")
        time.sleep(1)
        print("Close vehicle object")
        vehicle.close()
        GPIO.cleanup()
    except KeyboardInterrupt:
        msgTo_log_server("EMERGENCY LAND!!")
        vehicle.mode = VehicleMode("LAND")
        time.sleep(1)
        msgTo_log_server("Close vehicle object")
        vehicle.close()
        GPIO.cleanup()

class SG90_92R_Class:
# mPin : GPIO Number (PWM)
# mPwm : PWM컨트롤러용 인스턴스
# m_Zero_offset_duty

    def __init__(self, Channel, ZeroOffset):
        self.mChannel = Channel
        self.m_ZeroOffset = ZeroOffset

        # Adafruit_PCA9685 init
        # address : I2C Channel 0x40 of PCA9685
        self.mPwm = Adafruit_PCA9685.PCA9685(address = 0x40)
        # set 50Hz, but  60Hz is better
        self.mPwm.set_pwm_freq(60)

    # set servo motor position
    def SetPos(self, pos):
        pulse = (650 - 150) * pos / 180 + 150 + self.m_ZeroOffset
        self.mPwm.set_pwm(self.mChannel, 0, int(pulse))

    # end
    def Cleanup(self):
        # reset servo motor 90 degree
        self.SetPos(0)
        time.sleep(1)
# function to put mini cargo
def put_cargo(ord):
    Servo0.SetPos(0)
    Servo4.SetPos(0)

    time.sleep(1)
    if ord % 2 == 1:  # drone arrives odd number point, set servo motor 110degree
        Servo0.SetPos(100)
        print(" ** " + str(ord) + "point Delivery complete ** ")
        time.sleep(3)  # wait for finish
        Servo0.SetPos(0)
        time.sleep(1)
        Servo0.Cleanup()


    elif ord % 2 == 0:  #  drone arrives even number point, set servo motor 110degree
        Servo4.SetPos(100)
        print(" ** " + str(ord) + "point Delivery complete ** ")
        time.sleep(3)  # wait for finish
        Servo4.SetPos(0)
        time.sleep(1)
        Servo4.Cleanup()

    time.sleep(1)


# Using thread to connect HPC image processing server and Web server
## Thread 1     for send video
def send_To_HPC_Imgserver(sock):
    print("Connect to Image Processing Server")
    try:
        # to send drone cam image to HPC image processing server
        # PI camera image capture
        cam = cv2.VideoCapture(0)
        # Frame size 3 = width, 4 = height
        cam.set(3, 360);
        cam.set(4, 270);
        # image quality range : 0~100, set 90 (default = 95)
        encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 30]

        while True:
            # get 1 frame
            # Success ret = True, Fail ret = False, frame = read frame

            ret, frame = cam.read()
            #font = cv2.FONT_HERSHEY_COMPLEX

            #gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

            # cv2.putText(frame, "Lat : " + str(clat), (20, 30), font, 0.5, (255, 255, 255), 1, cv2.LINE_4)
            # cv2.putText(frame, "Long : " + str(clong), (20, 60), font, 0.5, (255, 255, 255), 1, cv2.LINE_4)
            # cv2.putText(frame, "Alt : " + str(calt) + "m", (20, 90), font, 0.5, (255, 255, 255), 1, cv2.LINE_4)

            # cv2. imencode(ext, img [, params])
            # encode_param format, frame to jpg image encode
            result, frame = cv2.imencode('.jpg', frame, encode_param)
            # convert frame to String type
            data = numpy.array(frame)
            stringData = data.tobytes()

            # send data to HPC image processing server
            # (str(len(stringData))).encode().ljust(16)
            img_clientSocket.sendall((str(len(stringData))).encode().ljust(16) + stringData)

    except:  # when socket connection failed
        print("Socket Close!!")
        cam.release()
        img_clientSocket.close()
    finally:
        img_clientSocket.close()
## Thread 2     for landing data
# def recv_From_HPC_Imgserver(sock):
#     global land_point
#     while True:
#         data = img_clientSocket.recv(1024)
#         land_point = data.decode("utf-8")
#         time.sleep(1)

def msgTo_log_server(msg_to_web):  # make message to HPC image processing server
    global vehicle
    msg_to_web = msg_to_web + "\", \"" + str(clat) + "\", \"" + str(clong)
    log_clientSocket.sendall(str(msg_to_web).encode("utf-8"))
    print(str(msg_to_web))

    data = log_clientSocket.recv(1024)
    data = data.decode("utf-8")
    print(str(data))
## Thread 3
# Move drone for TSP path and send log data to Web
def send_To_HPC_Logserver(sock):
    global vehicle, flydistance
    #   To send Drone log, video and other information to Web Server
    #   Client socket connection to Web Server
    try:
        print("Connect Drone to Web Server!")

        # send locations order
        log_clientSocket.sendall(str(locationsTo_Web).encode("utf-8"))
        print(str(locationsTo_Web))
        data = log_clientSocket.recv(1024)

        # send point to point fly time data
        log_clientSocket.sendall(str(distanceTo_Web).encode("utf-8"))
        print(str(distanceTo_Web))
        data = log_clientSocket.recv(1024)

        flydistance = list(map(float, flydistance))

        num = 0  # Current Target point to send Server

        msgTo_log_server("Start to move")  # convert num to string type     send 1 to server

        # 1  start Drone delivery.    The number of point(including Home base) : 12
        while num < client_index:  # loop 12 times, manipulate it when you test this system
            num = num + 1     # to move first(1) point
            drone_fly(latitude[num], longitude[num])
            point = str(latitude[num]) + '/' + str(longitude[num])
            msgTo_log_server(point)
            point = "Target " + str(num) + " arrive"
            msgTo_log_server(point)
            msgTo_log_server("Arrive")      ## recognization for delivery order in Web
            time.sleep(5)
            vehicle = connect("/dev/ttyACM0", wait_ready=True, baud=57600)
            msgTo_log_server("Vehicle Reconnect!")
            if num == client_index - 1:
                msgTo_log_server("Return To Base")

        time.sleep(1)
        vehicle.close()
        # 2(Finish Drone delivery)
        msgTo_log_server("Completed to Delivery")
        msgTo_log_server("Finish")

        log_clientSocket.close()  # close socket connection

        ### End Drone Delivery System

    except Exception as e:  # when socket connection failed
        print(e)
        print("EMERGENCY LAND!!")
        vehicle.mode = VehicleMode("LAND")
        time.sleep(1)
        print("Close vehicle object")
        vehicle.close()
        log_clientSocket.close()
    except KeyboardInterrupt:
        print("EMERGENCY LAND!!")
        vehicle.mode = VehicleMode("LAND")
        time.sleep(1)
        print("Close vehicle object")
        vehicle.close()
        log_clientSocket.close()
    finally:
        log_clientSocket.close()



if __name__=="__main__":

    # connect lidar to raspberry pi 4
    if ser.isOpen() == False:
        ser.open()

    Servo0 = SG90_92R_Class(Channel=0, ZeroOffset=-10)
    Servo4 = SG90_92R_Class(Channel=4, ZeroOffset=-10)

    # socket connection address and port for Koren VM TSP server
    # get shortest path data from Koren VM TSP server
    TSP_SERVER_IP = "116.89.189.31"  # Koren VM TSP server IP(Web)
    TSP_SERVER_PORT = 22042
    SIZE = 512
    tsp_client_socket = socket(AF_INET, SOCK_STREAM)
    tsp_client_socket.connect((TSP_SERVER_IP, TSP_SERVER_PORT))
    # to get TSP path from Koren VM TSP server
    get_TSP_path()

    ######## Start flying Drone ########
    try:
        ## Image processing Server(HPC)
        # send drone cam image to HPC image processing server and get landing data from HPC server
        IMG_SERVER_IP = "116.89.189.55"   # HPC Image Processing server IP(Middle)
        IMG_SERVER_PORT = 22044    # HPC external port 22044(10011)
        img_clientSocket = socket(AF_INET, SOCK_STREAM)
        img_clientSocket.connect((IMG_SERVER_IP, IMG_SERVER_PORT))

        time.sleep(3)
        try:
            ## Log Server (HPC)
            # send drone log(altitude, arrive point point etc..) to Log server
            Log_SERVER_IP = "116.89.189.55"  # Log Server IP(Middle)
            Log_SERVER_PORT = 22045
            log_clientSocket = socket(AF_INET, SOCK_STREAM)
            log_clientSocket.connect((Log_SERVER_IP, Log_SERVER_PORT))

            try:
                ##   Declare Thread
                # Log Server Thread
                sendLog = threading.Thread(target=send_To_HPC_Logserver, args=(log_clientSocket,))
                # Image Processing Server Thread
                sendImg = threading.Thread(target=send_To_HPC_Imgserver, args=(img_clientSocket,))
                # receiver = threading.Thread(target=recv_From_HPC_Imgserver, args=(img_clientSocket,))

                ##  Start Thread
                # Image Processing Server Thread
                sendImg.start()
                # receiver.start()
                # Log Server Thread
                sendLog.start()


                while True:
                    time.sleep(1)   # thread 간의 우선순위 관계 없이 다른 thread에게 cpu를 넘겨줌(1 일때)
                    pass            # sleep(0)은 cpu 선점권을 풀지 않음

                log_clientSocket.close()
                img_clientSocket.close()
            except Exception as e:  # when socket connection failed
                print(e)
                print("EMERGENCY LAND!!")
                vehicle.mode = VehicleMode("LAND")
                time.sleep(1)
                print("Close vehicle object")
                GPIO.cleanup()
                log_clientSocket.close()
                img_clientSocket.close()
            except KeyboardInterrupt:
                msgTo_log_server("EMERGENCY Return!!")
                msgTo_log_server("***********************\nPlease wait for 10 sec to return!!\n***********************")
                loc_point = LocationGlobalRelative(latitude[0], longitude[0], 3)
                vehicle.simple_goto(loc_point, groundspeed=3)
                time.sleep(10)
                vehicle.mode = VehicleMode("LAND")
                time.sleep(1)
                msgTo_log_server("Close vehicle object")
                GPIO.cleanup()
                log_clientSocket.close()
                img_clientSocket.close()
        except Exception as e:  # when socket connection failed
            print(e)
    except Exception as e:  # when socket connection failed
        print(e)
