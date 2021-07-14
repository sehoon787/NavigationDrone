# echo_client.py
# -*- coding:utf-8 -*-

import socket
import numpy

# 접속 정보 설정
SERVER_IP = '116.89.189.55'
SERVER_PORT = 22041
SIZE = 512
SERVER_ADDR = (SERVER_IP, SERVER_PORT)

# 클라이언트 소켓 설정
with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as client_socket:
    client_socket.connect(SERVER_ADDR)  # 서버에 접속

    point = input('>>>')
    client_socket.send(point.encode('utf-8'))

    msg = client_socket.recv(256)  # 서버로부터 응답받은 메시지 반환
    msg = str(msg)
    msg_from_server = msg.split('\'')
    msg_from_server = msg_from_server[1].split('\\')
    msg_from_server = msg_from_server[0]
    print("Message from Drone Delivery Server : {}".format(msg_from_server))  # 서버로부터 응답받은 메시지 출력

# Server로부터 받은 각 좌표들을 위의 targetdata와 비교하고 행렬에서 다음 타겟으로 이동하기 위한 distance 값 밑 angle 추출

# 마브링크 등을 이용한 Pixhawk 통제
