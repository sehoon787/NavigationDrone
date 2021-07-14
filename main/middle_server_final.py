## Middle Server

import threading
from socket import *
import cv2
import numpy as np
import time

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
        while True:
            # stringData size from client (==(str(len(stringData))).encode().ljust(16))
            length = recvall(connectionSocket, 16)
            stringData = recvall(connectionSocket, int(length))
            data = np.fromstring(stringData, dtype='uint8')

            Img_Web.sendall((str(len(stringData))).encode().ljust(16) + stringData)  # send image to Web server

            # Decode data
            frame = cv2.imdecode(data, cv2.IMREAD_COLOR)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    except:  # when socket connection failed
        # When everything is done, release the capture
        cv2.destroyAllWindows()
        print("Socket close!!")
        # Img_Web.close()
    # finally:
        # Img_Web.close()

# Thraed 2
def send_log_to_Web(sock):
    while True:
        Log_Web.send(logdata.encode("utf-8"))
        Log_Web.recv(1024)

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

    print("Service Finish")


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
                    receiver = threading.Thread(target=recv_video_from_Drone, args=(Log_Web,))  # connectionSocket
                    # 로그 전송 쓰레드, 22046(VM-DB)
                    sendlog = threading.Thread(target=send_log_to_Web, args=(Log_Web,))
                    # 로그 수신 쓰레드, 22045(drone-VM)
                    log = threading.Thread(target=get_log_from_Drone, args=(connectionSocket2,))

                    receiver.start()
                    sendlog.start()
                    # sender.start()
                    log.start()

                    while True:
                        time.sleep(1)  # thread 간의 우선순위 관계 없이 다른 thread에게 cpu를 넘겨줌
                        pass  # sleep(0)은 cpu 선점권을 풀지 않음

                    connectionSocket2.close()
                    serverSocket2.close()
                    connectionSocket.close()
                    serverSocket.close()
                    Log_Web.close()
                    Img_Web.close()

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
