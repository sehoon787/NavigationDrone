import threading
from socket import *
import cv2
import numpy as np
import time
import os

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
        length = recvall(connectionSocket, 16)
        stringData = recvall(connectionSocket, int(length))
        data = np.fromstring(stringData, dtype='uint8')

        # data decode
        frame = cv2.imdecode(data, cv2.IMREAD_COLOR)
        original = frame.copy()

        cv2.imwrite('/root/work/web/original.jpg', original)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

if __name__ == "__main__":
    try:
        # Image Server(Koren vm)
        WebServer_IP = "116.89.189.31"
        WebServer_PORT = 22043
        serverSocket = socket(AF_INET, SOCK_STREAM)
        serverSocket.bind((WebServer_IP, WebServer_PORT))
        serverSocket.listen(1)
        print("Web Image server waiting...")
        connectionSocket, addr = serverSocket.accept()
        print(str(addr), "has connected")

        receiver = threading.Thread(target=recv_video_from_Drone, args=(connectionSocket,))  # 영상 수신 및 전송 쓰레드

        receiver.start()

        os.system("node /root/work/web/index.js")

        while True:
            time.sleep(1)  # thread 간의 우선순위 관계 없이 다른 thread에게 cpu를 넘겨줌
            pass  # sleep(0)은 cpu 선점권을 풀지 않음

    except Exception as e:
        print(e)
        connectionSocket.close()  # 서버 닫기
        serverSocket.close()
