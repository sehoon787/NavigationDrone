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
import RPi.GPIO as GPIO     # RaspberryPi lib

client_index = 6  # the number of client. Add 1 to use path information(for Home base and to return)

clat = 0
clong = 0

latitude = []
longitude = []

## Raspberry pi setting
# GPIO Mode (BOARD / BCM)
GPIO.setmode(GPIO.BCM)
# set GPIO Pins
GPIO_TRIGGER = 26
GPIO_ECHO = 19
# set GPIO direction (IN / OUT)
GPIO.setup(GPIO_TRIGGER, GPIO.OUT)
GPIO.setup(GPIO_ECHO, GPIO.IN)

vehicle = connect('/dev/ttyACM0', wait_ready=True, baud=57600)
print("Vehicle Connect")


# get TSP path from TSP HCP server
# not thread
def get_TSP_path():
    global locationsTo_Web
    #   To get shortest visiting path by using HPC TSP algorithm and point
    #   Client socket connection to HPC TSP Server
    msg = tsp_client_socket.recv(256)  # get message from server

    print("Connect Drone to Server!")

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

    msgTo_server("Basic pre-arm checks")
    # Don't try to arm until autopilot is ready
    while not vehicle.is_armable:
        msgTo_server(" Waiting for vehicle to initialise...")
        time.sleep(1)

    msgTo_server("Arming motors")
    # Copter should arm in GUIDED mode
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True

    # Confirm vehicle armed before attempting to take off
    while not vehicle.armed:
        msgTo_server(" Waiting for arming...")
        time.sleep(1)

    msgTo_server("Taking off!")
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
    global clat, clong
    try:
        msgTo_server("(Go)Take off!")
        arm_and_takeoff(2)  # take off altitude 2M

        i = 3  # start altitude to move 4M

        msgTo_server("(Go)Set default/target airspeed to 3")
        vehicle.airspeed = 1

        msgTo_server("(Go)Angle Positioning and move toward")  # move to next point

        dist = 1000

        starttime=time.time()
        flytime=0
        while flytime <= 30:

            if 150 <= dist <= 300:  # 3M from obstacle
                msgTo_server("(Go)Detect Obstacle")

                i = i + 1
                msgTo_server("(Go)Up to : " + str(i))
                while True:
                    msgTo_server("(Go)Altitude : " + str(vehicle.location.global_relative_frame.alt))
                    # Break and return from function just below target altitude.
                    send_attitude_target(roll_angle=0.0, pitch_angle=0.0,
                                         yaw_angle=None, yaw_rate=0.0, use_yaw_rate=False,
                                         thrust=0.6)
                    clat = vehicle.location.global_relative_frame.lat
                    clong = vehicle.location.global_relative_frame.lon
                    if vehicle.location.global_relative_frame.alt >= i * 0.95:
                        msgTo_server("(Go)Reached target altitude")
                        break
                    time.sleep(1)
            else:
                msgTo_server("(Go)Go Forward")
                loc_point = LocationGlobalRelative(lati, longi, i)
                vehicle.simple_goto(loc_point, groundspeed=1)
                clat = vehicle.location.global_relative_frame.lat
                clong = vehicle.location.global_relative_frame.lon
                time.sleep(1)

            # dist = distance()
            # if 150 <= dist <= 300:
            #     msgTo_server("(Go)Vehicle from Obstacle : " + str(dist))
            flytime = time.time() - starttime
            # For a complete implementation of follow me you'd want adjust this delay

        msgTo_server("(L)Set General Landing Mode")
        vehicle.mode = VehicleMode("LAND")
        time.sleep(1)

        msgTo_server("Close vehicle object")
        vehicle.close()
        msgTo_server("Ready to leave to next Landing Point")
    except Exception as e:  # when socket connection failed
        print(e)
        print("EMERGENCY LAND!!")
        vehicle.mode = VehicleMode("LAND")
        time.sleep(1)
        print("Close vehicle object")
        vehicle.close()
    except KeyboardInterrupt:
        msgTo_server("EMERGENCY LAND!!")
        vehicle.mode = VehicleMode("LAND")
        time.sleep(1)
        msgTo_server("Close vehicle object")
        vehicle.close()


# Using thread to connect HPC image processing server and Web server
## Thread 1
def send_img_Toserver(sock):
    global clat, clong

    print("Connect to Server")
    try:

        ## webcam 이미지 capture
        cam = cv2.VideoCapture(0)

        ## 이미지 속성 변경 3 = width, 4 = height
        cam.set(3, 690);
        cam.set(4, 480);

        ## 0~100에서 90의 이미지 품질로 설정 (default = 95)
        encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 90]

        while True:
            # 비디오의 한 프레임씩 읽는다.
            # 제대로 읽으면 ret = True, 실패면 ret = False, frame에는 읽은 프레임

            ret, frame = cam.read()

            font = cv2.FONT_HERSHEY_SIMPLEX

            #gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

            cv2.putText(frame,
                        "Latitude : " + str(clat) + ", Longitude : " + str(clong),
                        (50, 50),
                        font, 0.5,
                        (0, 255, 255),
                        1,
                        cv2.LINE_4)

            # cv2. imencode(ext, img [, params])
            # encode_param의 형식으로 frame을 jpg로 이미지를 인코딩한다.
            result, frame = cv2.imencode('.jpg', frame, encode_param)
            # frame을 String 형태로 변환
            data = numpy.array(frame)
            stringData = data.tobytes()

            # 서버에 데이터 전송
            # (str(len(stringData))).encode().ljust(16)
            img_Client_socket.sendall((str(len(stringData))).encode().ljust(16) + stringData)

    except:  # when socket connection failed
        print("Socket Close!!")
        cam.release()
        img_Client_socket.close()
    finally:
        img_Client_socket.close()

def msgTo_server(msg_to_web):  # make message to HPC image processing server
    global vehicle
    msg_to_web = msg_to_web + ", Lat : " + str(clat) + ", Lng : " + str(clong)
    log_Client_socket.sendall(str(msg_to_web).encode("utf-8"))
    print(str(msg_to_web))

    data = log_Client_socket.recv(1024)
    data = data.decode("utf-8")
    print(str(data))
## Thread 3
# Move drone for TSP path and send log data to Web
def send_log_Toserver(sock):
    global vehicle
    #   To send Drone log, video and other information to Web Server
    #   Client socket connection to Web Server
    try:
        print("Connect Drone to Web Server!")

        num = 0  # Current Target point to send Server

        msgTo_server("Start to move")  # convert num to string type     send 1 to server

        # 1  start Drone delivery.    The number of point(including Home base) : 6
        while num < client_index:  # loop 6 times, manipulate it when you test this system
            num = num + 1     # to move first(1) point
            drone_fly(latitude[num], longitude[num])
            point = str(latitude[num]) + '/' + str(longitude[num])
            msgTo_server(point)
            point = "Target " + str(num) + " arrive"
            msgTo_server(point)
            time.sleep(1)
            if num < client_index:
                vehicle = connect("/dev/ttyACM0", wait_ready=True, baud=57600)
                msgTo_server("Vehicle Reconnect!")
            elif num == client_index - 1:
                msgTo_server("Return To Base")

        # 2(Finish Drone delivery)
        msgTo_server("Completed to Delivery")
        msgTo_server("Finish")

        msgTo_server("arrive")
        log_Client_socket.close()  # close socket connection

        ### End Drone Delivery System

    except Exception as e:  # when socket connection failed
        print(e)
        print("EMERGENCY LAND!!")
        vehicle.mode = VehicleMode("LAND")
        time.sleep(1)
        print("Close vehicle object")
        vehicle.close()
        log_Client_socket.close()
    except KeyboardInterrupt:
        print("EMERGENCY LAND!!")
        vehicle.mode = VehicleMode("LAND")
        time.sleep(1)
        print("Close vehicle object")
        vehicle.close()
        log_Client_socket.close()
    finally:
        log_Client_socket.close()



if __name__=="__main__":

    # socket connection address and port for Koren VM TSP server
    # get shortest path data from Koren VM TSP server
    TSP_SERVER_IP = "192.168.1.221"  # Koren VM TSP server IP
    TSP_SERVER_PORT = 2204
    SIZE = 512
    tsp_client_socket = socket(AF_INET, SOCK_STREAM)
    tsp_client_socket.connect((TSP_SERVER_IP, TSP_SERVER_PORT))
    # to get TSP path from Koren VM TSP server
    get_TSP_path()
    time.sleep(5)


    ######## Start flying Drone ########

    ## HPC Image processing Server(Image)
    # send drone cam image to HPC image processing server and get landing data from HPC server
    IMG_SERVER_IP = "192.168.1.221"   # Koren VM Image Processing server IP
    IMG_SERVER_PORT = 22044    # HPC external port 22044(10011)
    img_Client_socket = socket(AF_INET, SOCK_STREAM)
    img_Client_socket.connect((IMG_SERVER_IP, IMG_SERVER_PORT))


    try:
        ## Web Server(Log)
        # send drone log(altitude, arrive point point etc..) to Web server
        Web_SERVER_IP = "192.168.1.221"  # koren SDI VM IP
        Web_SERVER_PORT = 22045
        log_Client_socket = socket(AF_INET, SOCK_STREAM)
        log_Client_socket.connect((Web_SERVER_IP, Web_SERVER_PORT))

        try:
            ##   Declare Thread
            # Log to Server Thread
            sendLog = threading.Thread(target=send_log_Toserver, args=(log_Client_socket,))
            # Image to Server Thread
            sendImg = threading.Thread(target=send_img_Toserver, args=(img_Client_socket,))

            ##  Start Thread
            # Image to Server Thread
            sendImg.start()
            # Log to Server Thread
            sendLog.start()


            while True:
                time.sleep(1)   # thread 간의 우선순위 관계 없이 다른 thread에게 cpu를 넘겨줌(1 일때)
                pass            # sleep(0)은 cpu 선점권을 풀지 않음
        except Exception as e:  # when socket connection failed
            print(e)
            print("EMERGENCY LAND!!")
            vehicle.mode = VehicleMode("LAND")
            time.sleep(1)
            print("Close vehicle object")
            GPIO.cleanup()
            img_Client_socket.close()
        except KeyboardInterrupt:
            msgTo_server("EMERGENCY LAND!!")
            vehicle.mode = VehicleMode("LAND")
            time.sleep(1)
            msgTo_server("Close vehicle object")
            GPIO.cleanup()
            img_Client_socket.close()
    except Exception as e:  # when socket connection failed
        print(e)
        print("EMERGENCY LAND!!")
        vehicle.mode = VehicleMode("LAND")
        time.sleep(1)
        print("Close vehicle object")
        GPIO.cleanup()
        log_Client_socket.close()
    except KeyboardInterrupt:
        msgTo_server("EMERGENCY LAND!!")
        vehicle.mode = VehicleMode("LAND")
        time.sleep(1)
        msgTo_server("Close vehicle object")
        GPIO.cleanup()
        log_Client_socket.close()
