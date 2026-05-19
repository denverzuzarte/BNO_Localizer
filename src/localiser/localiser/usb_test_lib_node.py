import threading
import serial
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3, Quaternion


class UsbTestLibNode(Node):
    def __init__(self):
        super().__init__('usb_test_lib_node')
        self.ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1.0)

        self.gyro_pub    = self.create_publisher(Vector3,    '/imu1/lib/gyro',    10)
        self.lin_acc_pub = self.create_publisher(Vector3,    '/imu1/lib/lin_acc', 10)
        self.rot_vec_pub = self.create_publisher(Quaternion, '/imu1/lib/rot_vec', 10)

        self._thread = threading.Thread(target=self._read_loop, daemon=True)
        self._thread.start()

    def _read_loop(self):
        while True:
            try:
                line  = self.ser.readline().decode('ascii', errors='ignore').strip()
                if not line:
                    continue

                parts = line.split(',')
                kind  = parts[0]

                if kind == 'G' and len(parts) == 4:
                    msg = Vector3()
                    msg.x = float(parts[1])   # rad/s, converted by library
                    msg.y = float(parts[2])
                    msg.z = float(parts[3])
                    self.gyro_pub.publish(msg)

                elif kind == 'A' and len(parts) == 4:
                    msg = Vector3()
                    msg.x = float(parts[1])   # m/s^2, converted by library
                    msg.y = float(parts[2])
                    msg.z = float(parts[3])
                    self.lin_acc_pub.publish(msg)

                elif kind == 'R' and len(parts) == 5:
                    msg = Quaternion()
                    msg.x = float(parts[1])   # i
                    msg.y = float(parts[2])   # j
                    msg.z = float(parts[3])   # k
                    msg.w = float(parts[4])   # real
                    self.rot_vec_pub.publish(msg)

            except Exception as e:
                self.get_logger().warn(f'Parse error: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = UsbTestLibNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
