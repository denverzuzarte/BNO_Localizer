import json
from collections import deque
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3, Quaternion
from std_msgs.msg import Float64

from localiser.eskf import ESKF

DEFAULT_CONFIG_PATH = '/home/denver/BNO_Localizer/src/localiser/config/calibration.json'


class EskfLocaliserNode(Node):
    def __init__(self):
        super().__init__('eskf_localiser')

        self.declare_parameter('config_path',         DEFAULT_CONFIG_PATH)
        self.declare_parameter('alpha',               0.99)
        self.declare_parameter('R_ori_init',          0.03 ** 2)
        self.declare_parameter('shoe_window',         30)
        self.declare_parameter('static_init_seconds', 10)
        self.declare_parameter('zupt_scale',          10.0)

        config_path         = str(self.get_parameter('config_path').value or DEFAULT_CONFIG_PATH)
        alpha               = float(self.get_parameter('alpha').value or 0.99)
        R_ori_init          = float(self.get_parameter('R_ori_init').value or 0.03 ** 2)
        shoe_window         = int(self.get_parameter('shoe_window').value or 30)
        static_init_seconds = int(self.get_parameter('static_init_seconds').value or 10)
        zupt_scale          = float(self.get_parameter('zupt_scale').value or 10.0)

        with open(config_path) as f:
            cal = json.load(f)

        self.alpha = alpha
        self.R_ori = R_ori_init * np.eye(3)
        self._cal  = cal

        sigma_a     = float(cal.get('sigma_a',    0.5))
        sigma_omega = float(cal.get('sigma_omega', 0.01))
        zupt_gamma  = float(cal.get('zupt_gamma',  50.0))

        self._sigma_a2     = sigma_a ** 2
        self._sigma_omega2 = sigma_omega ** 2
        self._zupt_gamma   = zupt_gamma

        zupt_var          = sigma_a ** 2 * shoe_window / 100.0 #* zupt_scale * 10.0
        self._R_zupt      = zupt_var * np.eye(3)
        self._R_zupt_init = zupt_var * 0.01 * np.eye(3)

        try:
            with open(config_path) as f:
                cfg = json.load(f)
            cfg['zupt_var'] = zupt_var
            with open(config_path, 'w') as f:
                json.dump(cfg, f, indent=2)
        except Exception as e:
            self.get_logger().warn(f'Could not save zupt_var: {e}')

        self._shoe_window = shoe_window
        self._gyro_buf    = deque(maxlen=shoe_window)
        self._accel_buf   = deque(maxlen=shoe_window)

        self._in_static_init   = True
        self._static_remaining = static_init_seconds
        self._static_timer     = self.create_timer(1.0, self._static_tick_cb)

        self.eskf: ESKF | None = None
        self._last_stamp_ns: int | None = None
        self._ori_initialized = False

        self.create_subscription(Vector3,    '/imu1/data/lin_acc', self._accel_cb,   10)
        self.create_subscription(Vector3,    '/imu1/data/gyro',    self._gyro_cb,    10)
        self.create_subscription(Quaternion, '/imu1/data/rot_vec', self._rot_vec_cb, 10)

        self.pos_pub       = self.create_publisher(Vector3,    '/localisation/pose/position',    10)
        self.vel_pub       = self.create_publisher(Vector3,    '/localisation/pose/velocity',    10)
        self.ori_pub       = self.create_publisher(Quaternion, '/localisation/pose/orientation', 10)
        self.acc_bias_pub  = self.create_publisher(Vector3,    '/localisation/pose/accel_bias',  10)
        self.gyro_bias_pub = self.create_publisher(Vector3,    '/localisation/pose/gyro_bias',   10)
        self.innov_pub     = self.create_publisher(Vector3,    '/eskf/innovation',               10)
        self.zupt_vel_pub  = self.create_publisher(Vector3,    '/eskf/zupt_velocity',            10)
        self.shoe_pub      = self.create_publisher(Float64,    '/eskf/shoe_statistic',           10)

        self.get_logger().info(
            f'EskfLocaliserNode ready  var_acc={cal["var_acc"]:.2e}  var_gyro={cal["var_gyro"]:.2e}  '
            f'sigma_a={sigma_a:.4f}  zupt_gamma={zupt_gamma:.1f}  '
            f'waiting for first rot_vec to initialize ESKF'
        )

    def _static_tick_cb(self) -> None:
        self._static_remaining -= 1
        if self._static_remaining > 0:
            self.get_logger().info(f'Static init: {self._static_remaining}s remaining — stay still!')
        else:
            self._in_static_init = False
            self._static_timer.cancel()
            self.get_logger().info('Static init complete — free to move.')

    def _gyro_cb(self, msg: Vector3) -> None:
        self._gyro_buf.append(np.array([msg.x, msg.y, msg.z]))

    def _rot_vec_cb(self, msg: Quaternion) -> None:
        q_meas = np.array([msg.x, msg.y, msg.z, msg.w])

        if not self._ori_initialized:
            cal = self._cal
            self.eskf = ESKF(
                var_acc=float(cal['var_acc']),
                var_gyro=float(cal['var_gyro']),
                var_acc_bias=float(cal['var_acc_bias']),
                var_gyro_bias=float(cal['var_gyro_bias']),
                init_quat=q_meas,
            )
            self._ori_initialized = True
            self.get_logger().info(f'ESKF initialized  q={q_meas}')
            return

        assert self.eskf is not None
        dtheta = self.eskf.update_quat(q_meas, self.R_ori)
        self.R_ori = self.alpha * self.R_ori + (1 - self.alpha) * np.outer(dtheta, dtheta)
        self.innov_pub.publish(Vector3(
            x=float(dtheta[0]),
            y=float(dtheta[1]),
            z=float(dtheta[2]),
        ))

    def _accel_cb(self, msg: Vector3) -> None:
        now_ns = self.get_clock().now().nanoseconds

        if self._last_stamp_ns is None:
            self._last_stamp_ns = now_ns
            return

        dt = (now_ns - self._last_stamp_ns) * 1e-9
        self._last_stamp_ns = now_ns

        if dt <= 0.0 or dt > 0.5 or self.eskf is None:
            return

        a_lin = np.array([msg.x, msg.y, msg.z])
        self._accel_buf.append(a_lin)

        self.eskf.predict(a_lin, dt)

        if self._in_static_init:
            self.eskf.update_zero_pos()
            self.eskf.update_zupt(self._R_zupt_init)
        else:
            T = self._compute_shoe()
            if T is not None:
                self.shoe_pub.publish(Float64(data=float(T)))
            if T is not None and T < self._zupt_gamma:
                accepted = self.eskf.update_zupt(self._R_zupt)
                if accepted:
                    v = self.eskf.x[3:6] * 100.0
                    self.zupt_vel_pub.publish(Vector3(
                        x=float(v[0]), y=float(v[1]), z=float(v[2])
                    ))

        self._publish_pose()

    def _compute_shoe(self) -> float | None:
        if len(self._accel_buf) < self._shoe_window or len(self._gyro_buf) < self._shoe_window:
            return None
        T = 0.0
        for a, w in zip(self._accel_buf, self._gyro_buf):
            T += np.dot(a, a) / self._sigma_a2 + np.dot(w, w) / self._sigma_omega2
        return T / len(self._accel_buf)

    def _publish_pose(self) -> None:
        assert self.eskf is not None
        x  = self.eskf.x
        cm = 100.0
        self.pos_pub.publish(Vector3(x=float(x[0]*cm),  y=float(x[1]*cm),  z=float(x[2]*cm)))
        self.vel_pub.publish(Vector3(x=float(x[3]*cm),  y=float(x[4]*cm),  z=float(x[5]*cm)))
        q = x[6:10]
        self.ori_pub.publish(Quaternion(x=float(q[0]),  y=float(q[1]),  z=float(q[2]),  w=float(q[3])))
        self.acc_bias_pub.publish( Vector3(x=float(x[10]*cm), y=float(x[11]*cm), z=float(x[12]*cm)))
        self.gyro_bias_pub.publish(Vector3(x=float(x[13]),    y=float(x[14]),    z=float(x[15])))


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(EskfLocaliserNode())


if __name__ == '__main__':
    main()
