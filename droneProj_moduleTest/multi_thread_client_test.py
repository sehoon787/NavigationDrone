# tcpclient.py

import cv2
import numpy
from socket import *
import threading
import time
import sys

def send(sock):

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
            clientSocket.sendall((str(len(stringData))).encode().ljust(16) + stringData)

    except socket.error:  # when socket connection failed
        print("Socket Close!!")
        cam.release()
        clientSocket.close()
        sys.exit()
    finally:
        clientSocket.close()

def recv(sock):
    try:
        while True:
            data = clientSocket.recv(1024)
            print("상대방 : ", data.decode("utf-8"))
    except socket.error:  # when socket connection failed
        print("Socket Close!!")
        clientSocket.close()
    finally:
        clientSocket.close()

ip = "192.168.0.6"
port = 22043

clientSocket = socket(AF_INET, SOCK_STREAM)
clientSocket.connect((ip, port))

print("접속 완료")

sender = threading.Thread(target=send, args=(clientSocket,))
receiver = threading.Thread(target=recv, args=(clientSocket,))

sender.daemon = True
receiver.daemon = True

sender.start()
receiver.start()

while True:
    time.sleep(1)
    pass

clientSocket.close()
