import threading
import serial
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3, Quaternion


class UsbTestFlipperNode(Node):
    def __init__(self):
        super().__init__('usb_test_flipper')
        self.ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1.0)

        self.gyro_pub    = self.create_publisher(Vector3,    '/imu1/test/gyro',    10)
        self.lin_acc_pub = self.create_publisher(Vector3,    '/imu1/test/lin_acc', 10)
        self.rot_vec_pub = self.create_publisher(Quaternion, '/imu1/test/rot_vec', 10)

        self._thread = threading.Thread(target=self._read_loop, daemon=True)
        self._thread.start()

    def _read_loop(self):
        while True:
            try:
                line = self.ser.readline().decode('ascii', errors='ignore').strip()
                if not line:
                    continue

                parts = line.split(',')
                kind  = parts[0]

                if kind == 'G' and len(parts) == 4:
                    x, y, z = int(parts[1]), int(parts[2]), int(parts[3])
                    msg = Vector3()
                    msg.x = x / 512.0   # Q9 -> rad/s
                    msg.y = y / 512.0
                    msg.z = z / 512.0
                    self.gyro_pub.publish(msg)

                elif kind == 'A' and len(parts) == 4:
                    x, y, z = int(parts[1]), int(parts[2]), int(parts[3])
                    msg = Vector3()
                    msg.x = x / 256.0   # Q8 -> m/s^2
                    msg.y = y / 256.0
                    msg.z = z / 256.0
                    self.lin_acc_pub.publish(msg)

                elif kind == 'R' and len(parts) == 5:
                    x, y, z, w = int(parts[1]), int(parts[2]), int(parts[3]), int(parts[4])
                    msg = Quaternion()
                    msg.x = x / 16384.0   # Q14
                    msg.y = y / 16384.0
                    msg.z = z / 16384.0
                    msg.w = w / 16384.0
                    self.rot_vec_pub.publish(msg)

            except Exception as e:
                self.get_logger().warn(f'Parse error: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = UsbTestFlipperNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
