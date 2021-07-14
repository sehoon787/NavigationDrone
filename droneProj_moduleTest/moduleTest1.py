## TSP에서 경로 받은 후 해당 경로 및 로그 Koren Web에 전송

from dronekit import connect, VehicleMode, LocationGlobal, LocationGlobalRelative
from pymavlink import mavutil  # Needed for command message definitions
import time
import math
import cv2
import numpy
import threading
from socket import *
import sys

client_index = 11  # the number of client. Add 1 to use path information(for Home base and to return)
locationsTo_Web = ""

latitude = []
longitude = []


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


def msgTo_webserver(msg_to_web):  # make message to HPC image processing server
    clientSocket.sendall(str(msg_to_web).encode("utf-8"))
    print(str(msg_to_web))
    data = clientSocket.recv(1024)
    data = str(data).split("b'", 1)[1].rsplit("'", 1)[0]
    print(data)

## Thread 3
# Move drone for TSP path and send log data to Web
def send_Logdata_toWebserver(sock):
    #   To send Drone log, video and other informations to Web Server
    #   Client socket connection to Web Server
    time.sleep(1)

    try:
        msgTo_webserver("Connect Drone to Web Server!")
        msgTo_webserver(locationsTo_Web)

        num = 0  # Current Target point to send Server

        msgTo_webserver("Start to move")  # convert num to string type     send 1 to server

        # 2(Finish Drone delivery)
        msgTo_webserver("Completed to Base")
        msgTo_webserver("Finish")

        msgTo_webserver("arrive")

        clientSocket.close()  # close socket connection

    except:  # when socket connection failed
        print("EMERGENCY LAND!!")

        time.sleep(1)
        print("Close vehicle object")
        clientSocket.close()
    finally:
        clientSocket.close()

if __name__=="__main__":

    # socket connection address and port for HPC TSP server
    # get shortest path data from HPC TSP server
    TSP_SERVER_IP = '116.89.189.55'  # HPC TSP server IP
    TSP_SERVER_PORT = 22042
    SIZE = 512
    tsp_client_socket = socket(AF_INET, SOCK_STREAM)
    tsp_client_socket.connect((TSP_SERVER_IP, TSP_SERVER_PORT))
    # to get TSP path from HPC TSP server
    get_TSP_path()



    ip = "116.89.189.55"
    port = 22044

    clientSocket = socket(AF_INET, SOCK_STREAM)
    clientSocket.connect((ip, port))

    print("접속 완료")

    sender = threading.Thread(target=send_Logdata_toWebserver, args=(clientSocket,))
    sender.start()

    while True:
        time.sleep(0)
        pass
