import pickle
import zmq
import serial
import struct

START_BYTE = 0xF1
END_BYTE = 0xF2
ACCLEROMETER_MSG_ID = 0x01

ser = serial.Serial('/dev/ttyACM0', 115200)

def find_packet_start():
    while ser.read()[0] != START_BYTE :
        print("waiting for start")    
 
def read_data():
    find_packet_start()
    print("pkg id : ", ser.read())
    data = ser.read()
    print("payload length : %d" % data[0])
    x = ser.read(4)
    y = ser.read(4)
    z = ser.read(4)
    acc_x = bytes_to_float(x)
    acc_y = bytes_to_float(y)
    acc_z = bytes_to_float(z)
    
    return acc_x, acc_y, acc_z

def bytes_to_float(b):
    val = (b[0] << 25) | (b[1] << 18) | (b[2] << 11) | (b[3] << 4)
    value = struct.unpack('<f', struct.pack('<I', val))[0]
    return value

while True:
    acc_x, acc_y, acc_z = read_data()
    print("acc x : ", acc_x)
    print("acc y : ", acc_y)
    print("acc z : ", acc_z)

