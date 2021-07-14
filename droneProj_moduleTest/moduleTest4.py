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
distanceTo_Web = "0/"   # to send point to point distance to Web server
land_point = "Land"
dist = 0

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
    flydist = math.sin(toRad(lat1)) * math.sin(toRad(lat2)) + math.cos(toRad(lat1)) * math.cos(toRad(lat2)) * math.cos(toRad(long2 - long1))
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
def distance(distance):
    global dist
    try:
        while True:
            counter = ser.in_waiting
            if counter > 8:
                bytes_serial = ser.read(9)
                ser.reset_input_buffer()

                if bytes_serial[0] == 0x59 and bytes_serial[1] == 0x59:
                    dist = bytes_serial[2] + bytes_serial[3] * 256
                    time.sleep(1)
                    ser.reset_input_buffer()
    except Exception as e:
        print("Ignore this Error!!" + str(e))
        return 0

## Drone Control function
def drone_fly(lati, longi):
    global clat, clong, calt, visitOrder, dist

    try:
        msgTo_log_server("(Go)Take off!")

        i = 3  # start altitude to move 3M

        msgTo_log_server("(Go)Set default/target airspeed to 1")

        airspeed = 1
        vehicle.airspeed = airspeed

        msgTo_log_server("(Go)Angle Positioning and move toward")  # move to next point

        starttime=time.time()
        flytime=0
        endtime = int(flydistance[visitOrder])/int(airspeed) + 10
        visitOrder = visitOrder + 1
        msgTo_log_server("(Go)Flying time : " + str(endtime-10))

        while flytime <= endtime:

            if 150 <= dist <= 400:  # 4M from obstacle
                msgTo_log_server("(Go)Detect Obstacle to " + str(dist) + "M")

                i = i + 1
                msgTo_log_server("(Go)Up to : " + str(i))

                msgTo_log_server("(Go)Altitude : " + str(i))

                clat = vehicle.location.global_relative_frame.lat
                clong = vehicle.location.global_relative_frame.lon
                calt = vehicle.location.global_relative_frame.alt

                dist = 0
            else:
                msgTo_log_server("(Go)Go Forward")
                clat = vehicle.location.global_relative_frame.lat
                clong = vehicle.location.global_relative_frame.lon
                calt = vehicle.location.global_relative_frame.alt
                time.sleep(1)

            if 150 <= dist:
                msgTo_log_server("(Go)Vehicle to Obstacle : " + str(dist))
            flytime = time.time() - starttime
            # For a complete implementation of follow me you'd want adjust this delay

        #msgTo_web("(L)Set General Landing Mode")
        #vehicle.mode = VehicleMode("LAND")
        #time.sleep(1)

        drone_land(lati, longi)     # image processing landing

        # put mini cargo on landing point
        put_cargo(visitOrder)

        msgTo_log_server("(Go)Close vehicle object")
        vehicle.close()
        msgTo_log_server("(Go)Ready to leave to next Landing Point")
    except Exception as e:  # when socket connection failed
        print(e)
        print("EMERGENCY LAND!!")
        time.sleep(1)
        print("Close vehicle object")
        vehicle.close()
        GPIO.cleanup()
    except KeyboardInterrupt:
        msgTo_log_server("EMERGENCY LAND!!")
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

        msgTo_log_server("(L)Target Panel Detect : " + str(land_point))
        # find_point = str(land_point)

        i = 4

        while True:
            i = i - 1
            # print(find_point)       # to print center or not

            if i <= 1:     # if altitude is less than 1m
                msgTo_log_server("(L)Set General Landing Mode")

                clat = vehicle.location.global_relative_frame.lat
                clong = vehicle.location.global_relative_frame.lon
                calt = vehicle.location.global_relative_frame.alt

                time.sleep(1)
                break

            elif lati == vehicle.location.global_relative_frame.lat and longi == vehicle.location.global_relative_frame.lon:
                msgTo_log_server("(L)Simple descending Landing Mode(Center)")

                msgTo_log_server("(L)Altitude : " + str(i))

                clat = vehicle.location.global_relative_frame.lat
                clong = vehicle.location.global_relative_frame.lon
                calt = vehicle.location.global_relative_frame.alt

                time.sleep(1)

            else:       # if Drone is not on right landing point, then move to right point
                msgTo_log_server("(L)Precision Landing Mode(Out of Target)")

                msgTo_log_server("(L)Altitude : " + str(i))

                clat = vehicle.location.global_relative_frame.lat
                clong = vehicle.location.global_relative_frame.lon
                calt = vehicle.location.global_relative_frame.alt

                # Send a new target every two seconds
                # For a complete implementation of follow me you'd want adjust this delay
                time.sleep(3)

    except Exception as e:  # when socket connection failed
        print(e)
        print("EMERGENCY LAND!!")
        time.sleep(1)
        print("Close vehicle object")
        vehicle.close()
        GPIO.cleanup()
    except KeyboardInterrupt:
        msgTo_log_server("EMERGENCY LAND!!")
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
    if ord % 2 == 1:  # drone arrives odd number point, set servo motor 130degree
        Servo0.SetPos(130)
        print(" ** " + str(ord) + "point Delivery complete ** ")
        time.sleep(3)  # wait for finish
        Servo0.SetPos(0)
        time.sleep(1)
        Servo0.Cleanup()


    elif ord % 2 == 0:  #  drone arrives even number point, set servo motor 130degree
        Servo4.SetPos(130)
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
        encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 40]

        while True:
            # get 1 frame
            # Success ret = True, Fail ret = False, frame = read frame

            ret, frame = cam.read()
            # font = cv2.FONT_HERSHEY_COMPLEX

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
# Thread 2     for landing data
def recv_From_HPC_Imgserver(sock):
    global land_point
    while True:
        data = img_clientSocket.recv(1024)
        land_point = data.decode("utf-8")
        time.sleep(1)

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
        data = log_clientSocket.recv(1024)

        # send point to point fly time data
        log_clientSocket.sendall(str(distanceTo_Web).encode("utf-8"))
        print(str(distanceTo_Web))
        data = log_clientSocket.recv(1024)

        flydistance = list(map(float, flydistance))

        num = 0  # Current Target point to send Server

        msgTo_log_server("Start to move")  # convert num to string type     send 1 to server

        # 1  start Drone delivery.    The number of point(including Home base) : 12
        while num < client_index:  # loop 6 times, manipulate it when you test this system
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

## Thread 4     for send video
def send_Front_video(sock):
    print("Connect to Image Processing Server")
    try:
        # to send drone cam image to HPC image processing server
        # PI camera image capture
        cam2 = cv2.VideoCapture(1)
        # Frame size 3 = width, 4 = height
        cam2.set(3, 680);
        cam2.set(4, 490);
        # image quality range : 0~100, set 90 (default = 95)
        encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 90]

        while True:
            # get 1 frame
            # Success ret = True, Fail ret = False, frame = read frame

            ret2, frame2 = cam2.read()
            # font = cv2.FONT_HERSHEY_COMPLEX

            #gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

            # cv2.putText(frame, "Lat : " + str(clat), (20, 30), font, 0.5, (255, 255, 255), 1, cv2.LINE_4)
            # cv2.putText(frame, "Long : " + str(clong), (20, 60), font, 0.5, (255, 255, 255), 1, cv2.LINE_4)
            # cv2.putText(frame, "Alt : " + str(calt) + "m", (20, 90), font, 0.5, (255, 255, 255), 1, cv2.LINE_4)

            # cv2. imencode(ext, img [, params])
            # encode_param format, frame to jpg image encode
            result2, frame2 = cv2.imencode('.jpg', frame2, encode_param)
            # convert frame to String type
            data2 = numpy.array(frame2)
            stringData2 = data2.tobytes()

            # send data to HPC image processing server
            # (str(len(stringData))).encode().ljust(16)
            img_clientSocket3.sendall((str(len(stringData2))).encode().ljust(16) + stringData2)

    except:  # when socket connection failed
        print("Socket Close!!")
        cam2.release()
        img_clientSocket3.close()
    finally:
        img_clientSocket3.close()

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
        print("Connect 1")
        time.sleep(3)
        try:
            ## Log Server (HPC)
            # send drone log(altitude, arrive point point etc..) to Log server
            Log_SERVER_IP = "116.89.189.55"  # Log Server IP(Middle)
            Log_SERVER_PORT = 22045
            log_clientSocket = socket(AF_INET, SOCK_STREAM)
            log_clientSocket.connect((Log_SERVER_IP, Log_SERVER_PORT))
            print("Connect 2")
            time.sleep(3)
            try:
                ## Front Image Server(HPC)
                # send drone cam image to HPC image processing server and get landing data from HPC server
                Front_video_server_IP = "116.89.189.55"  # HPC Image Processing server IP(Middle)
                Front_video_server_PORT = 22047  # HPC external port 22044(10011)
                img_clientSocket3 = socket(AF_INET, SOCK_STREAM)
                img_clientSocket3.connect((Front_video_server_IP, Front_video_server_PORT))
                print("Connect 3")
                time.sleep(3)

                ##   Declare Thread
                # Log Server Thread
                sendLog = threading.Thread(target=send_To_HPC_Logserver, args=(log_clientSocket,))
                # Image Processing Server Thread
                sendImg = threading.Thread(target=send_To_HPC_Imgserver, args=(img_clientSocket,))
                receiver = threading.Thread(target=recv_From_HPC_Imgserver, args=(img_clientSocket,))
                find_obstacle = threading.Thread(target=distance, args=(dist,))
                front_video = threading.Thread(target=send_Front_video, args=(img_clientSocket3,))

                time.sleep(3)
                
                ##  Start Thread
                # Image Processing Server Thread
                sendImg.start()
                receiver.start()
                # Log Server Thread
                sendLog.start()
                # get distance to Ostacle
                find_obstacle.start()
                # get Drone front cam
                front_video.start()

                while True:
                    time.sleep(1)   # thread 간의 우선순위 관계 없이 다른 thread에게 cpu를 넘겨줌(1 일때)
                    pass            # sleep(0)은 cpu 선점권을 풀지 않음

                log_clientSocket.close()
                img_clientSocket.close()
                img_clientSocket3.close()

            except Exception as e:  # when socket connection failed
                print(e)
                print("EMERGENCY LAND!!")
                time.sleep(1)
                print("Close vehicle object")
                GPIO.cleanup()
                log_clientSocket.close()
                img_clientSocket.close()
                img_clientSocket3.close()
            except KeyboardInterrupt:
                msgTo_log_server("EMERGENCY Return!!")
                msgTo_log_server("***********************\nPlease wait for 10 sec to return!!\n***********************")
                time.sleep(1)
                msgTo_log_server("Close vehicle object")
                GPIO.cleanup()
                log_clientSocket.close()
                img_clientSocket.close()
                img_clientSocket3.close()
        except Exception as e:  # when socket connection failed
            print(e)
    except Exception as e:  # when socket connection failed
        print(e)
