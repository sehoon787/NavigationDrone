import socket
import cv2
import numpy as np
import threading
import numpy

# return recived buffer from socket client
def recvall(sock, count):
    # Byte string
    buf = b''
    while count:
        newbuf = sock.recv(count)
        if not newbuf: return None
        buf += newbuf
        count -= len(newbuf)
    return buf


# HPC image processing server
HOST = ''
PORT = 22043    # to get image from Drone

# then this program is client to send image to Web Server
WebSERVER_IP = '116.89.189.55'  # HPC TSP server IP
PORT2 = 22044   # to send image to Web(10004 external port)

def getImage_fromDrone(port):
    try:
        # TCP
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        print('Socket created')

        # Set Server IP and port number
        s.bind((HOST, PORT))
        print('Socket bind complete')
        # Waiting Client (Maximum 10)
        s.listen(10)
        print('Socket now listening')

        # Connect, conn : socket object, addr : binding address
        conn, addr = s.accept()

        while True:
            # stringData size from client (==(str(len(stringData))).encode().ljust(16))
            length = recvall(conn, 16)
            stringData = recvall(conn, int(length))
            data = np.fromstring(stringData, dtype='uint8')

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
        print("Drone Socket close!!")
        s.close()
    finally:
        s.close()


def sendImage_toWeb(port):
    try:
        ## Connect to HPC image processing server
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        ## server ip, port
        s.connect((WebSERVER_IP, PORT2))

        # to send drone cam image to HPC image processing server
        # PI camera image capture
        cam = cv2.imread("../original.png", 1)
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
            s.sendall((str(len(stringData))).encode().ljust(16) + stringData)

    except socket.error:  # when socket connection failed
        print("Web Socket Close!!")
        s.close()
    finally:
        s.close()


if __name__=="__main__":
    thread_getImage = threading.Thread(target=getImage_fromDrone, args=(PORT,))
    thread_getImage.start()

    thread_sendImage = threading.Thread(target=sendImage_toWeb, args=(PORT2,))
    thread_sendImage.start()
