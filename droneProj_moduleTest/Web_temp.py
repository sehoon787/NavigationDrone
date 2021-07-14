import threading
from socket import *
import cv2
import numpy as np
import time

data = ""

# socket에서 수신한 버퍼를 반환하는 함수
def recvall(sock, count):
    # 바이트 문자열
    buf = b''
    while count:
        newbuf = sock.recv(count)
        if not newbuf: return None
        buf += newbuf
        count -= len(newbuf)
    return buf

def recv_video_from_Drone(sock):     # get Drone cam image from Drone, and send image to KorenVM Web Server
    while True:
        # client에서 받은 stringData의 크기 (==(str(len(stringData))).encode().ljust(16))
        length = recvall(connectionSocket, 16)
        stringData = recvall(connectionSocket, int(length))
        data = np.fromstring(stringData, dtype='uint8')

        # data를 디코딩한다.
        frame = cv2.imdecode(data, cv2.IMREAD_COLOR)
        original = frame.copy()

        cv2.imshow('original', original)
        cv2.imwrite('./original.jpg', original)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

def get_log_from_Drone(port):
    global data

    while data!="arrive":
        data = connectionSocket2.recv(1024)
        data = str(data).split("b'", 1)[1].rsplit("'", 1)[0]
        print(data)
        ## send receive message to drone
        connectionSocket2.sendall(str("Server Log Get!").encode("utf-8"))
        ## send log to KorenVM Web server
    connectionSocket2.close()
    serverSocket2.close()
    print("Connect Finish")

if __name__ == "__main__":
    try:
        # Image Server(Koren vm)
        WebServer_IP = "192.168.0.2"
        WebServer_PORT = 22043
        serverSocket = socket(AF_INET, SOCK_STREAM)
        serverSocket.bind((WebServer_IP, WebServer_PORT))
        serverSocket.listen(1)
        print("Web Image server waiting...")
        connectionSocket, addr = serverSocket.accept()
        print(str(addr), "에서 접속되었습니다.")

        try:
            # Log server(Koren vm)
            WebServer_IP = "192.168.0.2"
            WebServer_PORT2 = 22046
            serverSocket2 = socket(AF_INET, SOCK_STREAM)
            serverSocket2.bind((WebServer_IP, WebServer_PORT2))
            serverSocket2.listen(1)
            print("Web Log server waiting...")
            connectionSocket2, addr2 = serverSocket2.accept()
            print("Connect HPC!")

            receiver = threading.Thread(target=recv_video_from_Drone, args=(connectionSocket,))  # 영상 수신 및 전송 쓰레드
            log = threading.Thread(target=get_log_from_Drone, args=(connectionSocket2,))  # 로그 수신 쓰레드

            receiver.start()
            log.start()

            while True:
                time.sleep(1)  # thread 간의 우선순위 관계 없이 다른 thread에게 cpu를 넘겨줌
                pass  # sleep(0)은 cpu 선점권을 풀지 않음

        except Exception as e:
            print(e)
            connectionSocket2.close()  # 서버 닫기
            serverSocket2.close()
    except Exception as e:
        print(e)
        connectionSocket.close()  # 서버 닫기
        serverSocket.close()
