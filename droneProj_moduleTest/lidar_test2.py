import serial

ser = serial.Serial("/dev/ttyS0", 115200)

def get_distance():
    while True:
        counter = ser.in_waiting
        if counter > 8:
            bytes_serial = ser.read(9)
            ser.reset_input_buffer()

            if bytes_serial[0] == 0x59 and bytes_serial[1] == 0x59:
                distance = bytes_serial[2] + bytes_serial[3]*256
                print("Distance : " + str(distance))
                ser.reset_input_buffer()
                return distance


if __name__ == "__main__":
    if ser.isOpen() == False:
        ser.open()
    get_distance()

    get_distance()
