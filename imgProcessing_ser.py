## Middle Server

import threading
from socket import *
import cv2
import numpy as np
import time
# from pymongo import MongoClient
# import gridfs

msg_to_drone = "Land"
logdata = "{\"log\":\"empty\"}"

# function to return received buffer from socket
def recvall(sock, count):
    # byte string
    buf = b''
    while count:
        newbuf = sock.recv(count)
        if not newbuf: return None
        buf += newbuf
        count -= len(newbuf)
    return buf

# Thread 1
def recv_video_from_Drone(sock):     # get Drone cam image from Drone, and send image to KorenVM Web Server
    try:
        global msg_to_drone
        cX = 0
        cY = 0
        while True:
            # stringData size from client (==(str(len(stringData))).encode().ljust(16))
            length = recvall(connectionSocket, 16)
            stringData = recvall(connectionSocket, int(length))
            data = np.fromstring(stringData, dtype='uint8')

            Img_Web.sendall((str(len(stringData))).encode().ljust(16) + stringData)  # send image to Web server

            # Decode data
            frame = cv2.imdecode(data, cv2.IMREAD_COLOR)
            original = frame.copy()
            frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            lower = np.array([0, 208, 94], dtype="uint8")
            upper = np.array([179, 255, 232], dtype="uint8")
            mask = cv2.inRange(frame, lower, upper)

            cv2.imwrite("./original.jpg", original)

            # Find contours
            cnts = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            # Extract contours depending on OpenCV version
            cnts = cnts[0] if len(cnts) == 2 else cnts[1]

            # Iterate through contours and filter by the number of vertices
            for c in cnts:
                # compute the center of the contour
                M = cv2.moments(c)
                try:
                    cX = int(M["m10"] / M["m00"])
                    cY = int(M["m01"] / M["m00"])
                except ZeroDivisionError:
                    print("")

                '''    
                # draw the contour and center of the shape on the image
                # using mask is better for speed
                cv2.drawContours(original, [c], -1, (0, 255, 0), 2)
                cv2.circle(original, (cX, cY), 7, (255, 255, 255), -1)
                if (cX >= 350 and cX <= 850) and (cY >= 200 and cY <= 600):
                    cv2.putText(mask, "center", (cX - 20, cY - 20),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
                else:
                    cv2.putText(mask, "Out of Target", (cX - 20, cY - 20),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
                '''

                # better for watching
                cv2.drawContours(original, [c], -1, (0, 255, 0), 2)
                cv2.circle(original, (cX, cY), 7, (255, 255, 255), -1)

                if (150 <= cX <= 410) and (120 <= cY <= 380):
                    cv2.putText(original, "Center", (cX - 20, cY - 20),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
                    #print("X : " + str(cX) + ", Y : " + str(cY))
                    msg_to_drone = "Center"
                    #print(msg_to_drone)
                else:
                    cv2.putText(original, "Out of Target", (cX - 20, cY - 20),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
                    #print("X : " + str(cX) + ", Y : " + str(cY))
                    msg_to_drone = "Out of Target"
                    #print(msg_to_drone)

            # cv2.imshow("mask", mask)
            cv2.imshow("imgProcessing", original)
            cv2.imwrite("imgProcessing_data.jpg", original)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    except:  # when socket connection failed
        # When everything is done, release the capture
        cv2.destroyAllWindows()
        print("Socket close!!")
        Img_Web.close()
    finally:
        Img_Web.close()

# Thraed 2
def send_log_to_Web(sock):
    while True:
        Log_Web.send(logdata.encode("utf-8"))
        Log_Web.recv(1024)

# Thread 3
def send_To_Drone(sock):     # send landing data to Drone
    while True:
        sock.send(msg_to_drone.encode("utf-8"))
        time.sleep(1)

# Thread 4
def get_log_from_Drone(sock):
    global logdata
    cnt = 0
    while logdata!="Finish":
        logdata = connectionSocket2.recv(1024)
        logdata = str(logdata).split("b'", 1)[1].rsplit("'", 1)[0]
        if cnt == 0:
            logdata = "{\"order\":\"" + logdata + "\"}"
            cnt = cnt + 1
        elif cnt == 1:
            logdata = "{\"dist\":\"" + logdata + "\"}"
            cnt = cnt + 2
        else:
            logdata = "{\"log\":[\"" + logdata + "\"] }"
        print(logdata)
        ## send receive message to drone
        connectionSocket2.sendall(str("Server Get!").encode("utf-8"))
        ## send log to KorenVM Web server
    connectionSocket2.close()
    serverSocket2.close()
    connectionSocket.close()
    serverSocket.close()
    Log_Web.close()
    Img_Web.close()
    print("Connect Finish")


if __name__=="__main__":

    print("Start Image processing Server")

    ## here, Client role 1(Image)
    # then this program is client to send image to Web Server
    WebSERVER_IP = '116.89.189.31'  # image Web server IP
    WebSERVER_PORT = 22043  # to send image to Web(10004 external port)
    ## Connect to Web Server for Image
    Img_Web = socket(AF_INET, SOCK_STREAM)
    try:
        Img_Web.connect((WebSERVER_IP, WebSERVER_PORT))
        print("Connect to Web for Image!")
        try:
            time.sleep(3)
            ## here, Client role 2(Log)
            # then this program is client to send log to Web Server
            WebSERVER_IP = '116.89.189.31'  # log Web server IP
            WebSERVER_PORT2 = 22046  # to send log to Web(10004 external port)
            ## Connect to Web Server for Log
            Log_Web = socket(AF_INET, SOCK_STREAM)

            Log_Web.connect((WebSERVER_IP, WebSERVER_PORT2))
            print("Connect to Web for Log!")
            try:
                # Image Server(Koren vm)
                HPCServer_IP = "116.89.189.55"
                HPCServer_PORT = 22044
                serverSocket = socket(AF_INET, SOCK_STREAM)
                serverSocket.bind((HPCServer_IP,HPCServer_PORT))
                serverSocket.listen(1)
                print("Waiting Drone Client")
                connectionSocket,addr = serverSocket.accept()
                print(str(addr),"has connected.")

                try:
                    # Log server(Koren vm)
                    HPCServer_IP = "116.89.189.55"
                    HPCServer_PORT2 = 22045
                    serverSocket2 = socket(AF_INET, SOCK_STREAM)
                    serverSocket2.bind((HPCServer_IP, HPCServer_PORT2))
                    serverSocket2.listen(1)
                    print("Waiting Drone Client...")
                    connectionSocket2, addr2 = serverSocket2.accept()
                    print("Connect Drone!")

                    # 영상 수신 및 전송 쓰레드, 22043(VM-DB), 22044(drone-VM)
                    receiver = threading.Thread(target=recv_video_from_Drone, args=(Img_Web,))  # connectionSocket
                    # 로그 전송 쓰레드, 22046(VM-DB)
                    sendlog = threading.Thread(target=send_log_to_Web, args=(Log_Web,))
                    # 영상처리결과 송신 쓰레드, 22044(drone-VM)    드론으로부터 영싱 수신 쓰레드와 동일 소켓
                    sender = threading.Thread(target=send_To_Drone, args=(connectionSocket,))
                    # 로그 수신 쓰레드, 22045(drone-VM)
                    log = threading.Thread(target=get_log_from_Drone, args=(connectionSocket2,))
                    #sendImg = threading.Thread(target=send_img_to_Web, args=(HPCServer_PORT2,))  # 영상 전송 쓰레드

                    receiver.start()
                    sendlog.start()
                    sender.start()
                    log.start()
                    #sendImg.start()

                    while True:
                        time.sleep(1)  # thread 간의 우선순위 관계 없이 다른 thread에게 cpu를 넘겨줌
                        pass  # sleep(0)은 cpu 선점권을 풀지 않음

                except Exception as e:
                    print(e)
                    connectionSocket2.close()  # close server
                    serverSocket2.close()
            except Exception as e:
                print(e)
                connectionSocket.close()    # close server
                serverSocket.close()
        except Exception as e:
            print(e)
            Log_Web.close()
    except Exception as e:
        print(e)
        Img_Web.close()
