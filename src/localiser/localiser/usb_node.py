import serial
import struct
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometery_msgs.msg import Vector3

START_BYTE = 0xF1
END_BYTE = 0xF2

ACCLEROMETER_MSG_ID = 0x01
LINEAR_ACCELERATION_MSG_ID = 0x02
QUATERNION_MSG_ID = 0x03

ser = serial.Serial('/dev/ttyACM0', 115200)


class UsbNode():
    def __init__():
        super().__init__('usb_node')
        self.publisher_ = self.create_publisher(String, 'topic', 10)


#def find_packet_start():
#    val = ser.read()[0]
#    while val != START_BYTE :
#        print("waiting for start : ", val)    

    def find_packet_start():
        while ser.read()[0] != START_BYTE :
            print("waiting for packet")
 
    def read_data():
        find_packet_start()
        msg_id = ser.read()
        len_data = ser.read()
        length =  len_data[0]
        data = [0] * int(length/4)
        for d in range (0,int(length/4)):
            data[d] = ser.read(4)
        ser.read()
        return data, msg_id 

    def read_linear_acceleration_data(lin_acc):
        print("lin_acc x : ", bytes_to_float(lin_acc[0]))
        print("lin_acc y : ", bytes_to_float(lin_acc[1]))
        print("lin_acc z : ", bytes_to_float(lin_acc[2]), "\n\n")

        
    def read_accelerometer_data(acc):
        print("acc x : ", bytes_to_float(acc[0]))
        print("acc y : ", bytes_to_float(acc[1]))
        print("acc z : ", bytes_to_float(acc[2]), "\n\n")

    def read_quaternion_data(quat):
        print("quat x : ", bytes_to_float(quat[0]))
        print("quat y : ", bytes_to_float(quat[1]))
        print("quat z : ", bytes_to_float(quat[2]))
        print("quat w : ", bytes_to_float(quat[3]), "\n\n")
        ser.read(4)

    def bytes_to_float(b):
        val = (b[0] << 25) | (b[1] << 18) | (b[2] << 11) | (b[3] << 4)
        value = struct.unpack('<f', struct.pack('<I', val))[0]
        return value

    def run():
        data, msg_id = read_data()
        if msg_id[0] == ACCLEROMETER_MSG_ID:
            read_accelerometer_data(data)
        elif msg_id[0] == LINEAR_ACCELERATION_MSG_ID:
            read_linear_acceleration_data(data)
        elif msg_id[0] == QUATERNION_MSG_ID:
            read_quaternion_data(data)


def main():
    rclpy.init(args = args)
    usb_node = UsbNode()
    rclpy.spin(usb_node)

    minimal_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
