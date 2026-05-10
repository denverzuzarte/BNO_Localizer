import struct
import threading
import serial
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3, Quaternion

START_BYTE = 0xF1
END_BYTE   = 0xF2

GYRO_MSG_ID    = 0x01  # gyroscope uncalibrated, rad/s
LIN_ACC_MSG_ID = 0x02  # linear acceleration, m/s^2
ROT_VEC_MSG_ID = 0x03  # rotation vector quaternion
ACCEL_MSG_ID   = 0x04  # raw accelerometer, m/s^2


def bytes_to_float(b):
    val = (b[0] << 25) | (b[1] << 18) | (b[2] << 11) | (b[3] << 4)
    return struct.unpack('<f', struct.pack('<I', val))[0]


class UsbTestNode(Node):
    def __init__(self):
        super().__init__('usb_test_node')
        self.ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1.0)

        self.gyro_pub    = self.create_publisher(Vector3,     '/imu1/data/gyro',    10)
        self.lin_acc_pub = self.create_publisher(Vector3,     '/imu1/data/lin_acc', 10)
        self.rot_vec_pub = self.create_publisher(Quaternion,  '/imu1/data/rot_vec', 10)
        self.accel_pub   = self.create_publisher(Vector3,     '/imu1/data/accel',   10)

        self._thread = threading.Thread(target=self._read_loop, daemon=True)
        self._thread.start()

    def _find_packet_start(self):
        while True:
            byte = self.ser.read(1)
            if byte and byte[0] == START_BYTE:
                return

    def _read_packet(self):
        self._find_packet_start()
        msg_id = self.ser.read(1)[0]
        length = self.ser.read(1)[0]
        floats = []
        for _ in range(length // 4):
            floats.append(self.ser.read(4))
        self.ser.read(1)  # END_BYTE
        return msg_id, floats

    def _read_loop(self):
        while True:
            try:
                msg_id, floats = self._read_packet()

                if msg_id == GYRO_MSG_ID and len(floats) >= 3:
                    msg = Vector3()
                    msg.x = bytes_to_float(floats[0])
                    msg.y = bytes_to_float(floats[1])
                    msg.z = bytes_to_float(floats[2])
                    self.gyro_pub.publish(msg)

                elif msg_id == LIN_ACC_MSG_ID and len(floats) >= 3:
                    msg = Vector3()
                    msg.x = bytes_to_float(floats[0])
                    msg.y = bytes_to_float(floats[1])
                    msg.z = bytes_to_float(floats[2])
                    self.lin_acc_pub.publish(msg)

                elif msg_id == ROT_VEC_MSG_ID and len(floats) >= 4:
                    msg = Quaternion()
                    msg.x = bytes_to_float(floats[0])
                    msg.y = bytes_to_float(floats[1])
                    msg.z = bytes_to_float(floats[2])
                    msg.w = bytes_to_float(floats[3])
                    self.rot_vec_pub.publish(msg)

                elif msg_id == ACCEL_MSG_ID and len(floats) >= 3:
                    msg = Vector3()
                    msg.x = bytes_to_float(floats[0])
                    msg.y = bytes_to_float(floats[1])
                    msg.z = bytes_to_float(floats[2])
                    self.accel_pub.publish(msg)

            except Exception as e:
                self.get_logger().warn(f'Serial read error: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = UsbTestNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
