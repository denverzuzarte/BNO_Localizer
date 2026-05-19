import json
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3, Quaternion
from sensor_msgs.msg import Imu

DEFAULT_CONFIG_PATH = '/home/denver/BNO_Localizer/src/localiser/config/calibration.json'


class ImuNode(Node):
    def __init__(self):
        super().__init__('imu_node')

        self.declare_parameter('config_path', DEFAULT_CONFIG_PATH)
        self.declare_parameter('alpha',       0.99)
        self.declare_parameter('R_ori_init',  0.03 ** 2)

        config_path = str(self.get_parameter('config_path').value)
        alpha       = float(self.get_parameter('alpha').value)
        R_ori_init  = float(self.get_parameter('R_ori_init').value)

        with open(config_path) as f:
            cal = json.load(f)

        self.b     = np.array(cal['b'])
        self.S     = np.array(cal['S'])
        self.S_inv = np.linalg.inv(self.S)
        self.alpha = alpha

        # Adaptive R — seeded from calibration, updated online
        self.R_accel  = np.array(cal['R_accel'])
        self.R_gyro   = np.array(cal['R_gyro'])
        self.mu_accel = np.zeros(3)
        self.mu_gyro  = np.zeros(3)
        self.R_ori    = R_ori_init * np.eye(3)

        # Latest cached sensor readings
        self.latest_gyro: np.ndarray       = np.zeros(3)
        self.latest_q:    Quaternion | None = None

        self.create_subscription(Vector3,    '/imu1/data/accel',   self._accel_cb, 10)
        self.create_subscription(Vector3,    '/imu1/data/gyro',    self._gyro_cb,  10)
        self.create_subscription(Quaternion, '/imu1/data/rot_vec', self._rot_cb,   10)
        self.create_subscription(Vector3,    '/eskf/innovation',   self._innov_cb, 10)

        self.imu_pub = self.create_publisher(Imu, '/imu1/data', 10)

        self.get_logger().info(
            f'ImuNode ready  alpha={self.alpha}  config={config_path}'
        )

    def _gyro_cb(self, msg: Vector3) -> None:
        g = np.array([msg.x, msg.y, msg.z])
        self.mu_gyro = self.alpha * self.mu_gyro + (1 - self.alpha) * g
        delta        = g - self.mu_gyro
        self.R_gyro  = self.alpha * self.R_gyro  + (1 - self.alpha) * np.outer(delta, delta)
        self.latest_gyro = g

    def _rot_cb(self, msg: Quaternion) -> None:
        self.latest_q = msg

    def _innov_cb(self, msg: Vector3) -> None:
        innov      = np.array([msg.x, msg.y, msg.z])
        self.R_ori = self.alpha * self.R_ori + (1 - self.alpha) * np.outer(innov, innov)

    def _accel_cb(self, msg: Vector3) -> None:
        a_raw   = np.array([msg.x, msg.y, msg.z])
        a_calib = self.S_inv @ (a_raw - self.b)

        # EWA covariance update
        self.mu_accel = self.alpha * self.mu_accel + (1 - self.alpha) * a_calib
        delta         = a_calib - self.mu_accel
        self.R_accel  = self.alpha * self.R_accel  + (1 - self.alpha) * np.outer(delta, delta)

        out             = Imu()
        out.header.stamp    = self.get_clock().now().to_msg()
        out.header.frame_id = 'imu'

        out.linear_acceleration.x          = float(a_calib[0])
        out.linear_acceleration.y          = float(a_calib[1])
        out.linear_acceleration.z          = float(a_calib[2])
        out.linear_acceleration_covariance = self.R_accel.flatten().tolist()

        out.angular_velocity.x          = float(self.latest_gyro[0])
        out.angular_velocity.y          = float(self.latest_gyro[1])
        out.angular_velocity.z          = float(self.latest_gyro[2])
        out.angular_velocity_covariance = self.R_gyro.flatten().tolist()

        if self.latest_q is not None:
            out.orientation            = self.latest_q
            out.orientation_covariance = self.R_ori.flatten().tolist()
        else:
            out.orientation_covariance[0] = -1.0  # ROS convention: orientation not available

        self.imu_pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(ImuNode())


if __name__ == '__main__':
    main()
