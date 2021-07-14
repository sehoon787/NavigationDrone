import socket
import cv2
import numpy as np

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


HOST = ''
PORT = 22044

try:
    # TCP 사용
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    print('Socket created')

    # 서버의 아이피와 포트번호 지정
    s.bind((HOST, PORT))
    print('Socket bind complete')
    # 클라이언트의 접속을 기다린다. (클라이언트 연결을 10개까지 받는다)
    s.listen(10)
    print('Socket now listening')

    # 연결, conn에는 소켓 객체, addr은 소켓에 바인드 된 주소
    conn, addr = s.accept()

    while True:
        # client에서 받은 stringData의 크기 (==(str(len(stringData))).encode().ljust(16))
        length = recvall(conn, 16)
        stringData = recvall(conn, int(length))
        data = np.fromstring(stringData, dtype='uint8')

        # data를 디코딩한다.
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
            perimeter = cv2.arcLength(c, True)
            approx = cv2.approxPolyDP(c, 0.04 * perimeter, True)
            if len(approx) > 5:
                cv2.drawContours(original, [c], -1, (36, 255, 12), -1)

        cv2.imshow('mask', mask)
        cv2.imshow('original', original)
        cv2.imwrite('../mask.png', mask)
        cv2.imwrite('../original.png', original)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

except s.error:    # when socket connection failed
    # When everything is done, release the capture
    cv2.destroyAllWindows()
    print("Socket close!!")
    s.close()
finally:
    s.close()
