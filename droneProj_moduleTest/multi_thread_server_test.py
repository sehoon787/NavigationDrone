#tcpserver.py
from socket import *
import cv2
import numpy as np
import threading
import time

msg_to_drone = "Center"

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

def send(sock):                 # 데이터 송신 함수
    while True:
        sock.send(msg_to_drone.encode("utf-8"))
        time.sleep(3)

def recv(sock):                 # 데이터 수신 함수
    try:
        global msg_to_drone
        cX = 0
        cY = 0
        while True:
            # stringData size from client (==(str(len(stringData))).encode().ljust(16))
            length = recvall(connectionSocket, 16)
            stringData = recvall(connectionSocket, int(length))
            data = np.fromstring(stringData, dtype='uint8')

            toWeb.sendall((str(len(stringData))).encode().ljust(16) + stringData)  # send image to Web server

            # Decode data
            frame = cv2.imdecode(data, cv2.IMREAD_COLOR)
            original = frame.copy()
            frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            lower = np.array([0, 208, 94], dtype="uint8")
            upper = np.array([179, 255, 232], dtype="uint8")
            mask = cv2.inRange(frame, lower, upper)

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

                if (cX >= 350 and cX <= 850) and (cY >= 200 and cY <= 600):
                    cv2.putText(original, "Center", (cX - 20, cY - 20),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
                    print("X : " + str(cX) + ", Y : " + str(cY))
                    msg_to_drone = "Center"
                    print(msg_to_drone)
                else:
                    cv2.putText(original, "Out of Target", (cX - 20, cY - 20),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
                    print("X : " + str(cX) + ", Y : " + str(cY))
                    msg_to_drone = "Out of Target"
                    print(msg_to_drone)

            # cv2.imshow("mask", mask)
            cv2.imshow("original", original)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    except socket.error:  # when socket connection failed
        # When everything is done, release the capture
        cv2.destroyAllWindows()
        print("Socket close!!")
        serverSocket.close()
    finally:
        serverSocket.close()

if __name__=="__main__":

    # then this program is client to send image to Web Server
    WebSERVER_IP = '192.168.0.2'  # HPC TSP server IP
    PORT2 = 22044  # to send image to Web(10004 external port)
    ## Connect to Web Server processing server
    toWeb = socket(AF_INET, SOCK_STREAM)
    ## server ip, portq
    toWeb.connect((WebSERVER_IP, PORT2))
    print("Connect to Web!")


    host = "192.168.0.6"
    port = 22043

    serverSocket = socket(AF_INET, SOCK_STREAM)
    serverSocket.bind((host,port))
    serverSocket.listen(1)
    print("대기중입니다")

    connectionSocket,addr = serverSocket.accept()

    print(str(addr),"에서 접속되었습니다.")

    sender = threading.Thread(target=send, args=(connectionSocket,))            # 송신 쓰레드
    receiever = threading.Thread(target=recv, args=(connectionSocket,))         # 수신 쓰레드

    sender.start()
    receiever.start()

    while True:
        time.sleep(1)   # thread 간의 우선순위 관계 없이 다른 thread에게 cpu를 넘겨줌
        pass            # sleep(0)은 cpu 선점권을 풀지 않음

    serverSocket.close()                                    # 서버 닫기
