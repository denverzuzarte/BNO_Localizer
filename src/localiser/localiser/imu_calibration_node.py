import json
import os
import numpy as np
from collections import deque
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3, Quaternion

POCKET_NAMES = ['+X', '-X', '+Y', '-Y', '+Z', '-Z']
GRAVITY      = 9.81

TRUE_G = {
    '+X': np.array([ GRAVITY,      0,      0]),
    '-X': np.array([-GRAVITY,      0,      0]),
    '+Y': np.array([      0,  GRAVITY,     0]),
    '-Y': np.array([      0, -GRAVITY,     0]),
    '+Z': np.array([      0,      0,  GRAVITY]),
    '-Z': np.array([      0,      0, -GRAVITY]),
}

DEFAULT_CONFIG_PATH = '/home/denver/BNO_Localizer/src/localiser/config/calibration.json'

POCKETS    = 'pockets'
COUNTDOWN  = 'countdown'
R_COLLECT  = 'r_collect'


def quat_to_rot(q):
    x, y, z, w = q
    return np.array([
        [1-2*(y*y+z*z),  2*(x*y-w*z),   2*(x*z+w*y)],
        [2*(x*y+w*z),    1-2*(x*x+z*z), 2*(y*z-w*x)],
        [2*(x*z-w*y),    2*(y*z+w*x),   1-2*(x*x+y*y)],
    ])


def classify_pocket(g_body, thresh):
    dirs = {
        '+X': np.array([ 1, 0, 0]),
        '-X': np.array([-1, 0, 0]),
        '+Y': np.array([ 0, 1, 0]),
        '-Y': np.array([ 0,-1, 0]),
        '+Z': np.array([ 0, 0, 1]),
        '-Z': np.array([ 0, 0,-1]),
    }
    for name, d in dirs.items():
        if np.dot(g_body, d) > thresh:
            return name
    return None


class ImuCalibrationNode(Node):
    def __init__(self):
        super().__init__('imu_calibration_node')

        self.declare_parameter('orient_thresh',  0.95)
        self.declare_parameter('stability_var',  0.05)
        self.declare_parameter('stability_win',  50)
        self.declare_parameter('samples_needed', 500)
        self.declare_parameter('r_samples',      300)
        self.declare_parameter('countdown_secs', 5)
        self.declare_parameter('output_path',    DEFAULT_CONFIG_PATH)

        self.orient_thresh  = self.get_parameter('orient_thresh').value
        self.stability_var  = self.get_parameter('stability_var').value
        self.stability_win  = self.get_parameter('stability_win').value
        self.samples_needed = self.get_parameter('samples_needed').value
        self.r_samples      = self.get_parameter('r_samples').value
        self.countdown_secs = self.get_parameter('countdown_secs').value
        self.output_path    = self.get_parameter('output_path').value

        self.create_subscription(Vector3,    '/imu1/data/accel',   self._accel_cb, 10)
        self.create_subscription(Quaternion, '/imu1/data/rot_vec', self._rot_cb,   10)

        self.latest_q    = None
        self.win         = deque(maxlen=self.stability_win)
        self.pockets     = {p: [] for p in POCKET_NAMES}
        self.done        = set()
        self.r_buf       = []
        self.state       = POCKETS
        self.countdown_n = 0
        self._timer      = None

        self.get_logger().info(
            f'Calibration node ready  '
            f'(orient_thresh={self.orient_thresh}, stability_var={self.stability_var})\n'
            'Place the sensor flat on each of the 6 faces and hold still.'
        )

    def _rot_cb(self, msg):
        self.latest_q = (msg.x, msg.y, msg.z, msg.w)

    # ── accel callback — routes to active state ───────────────────────────────
    def _accel_cb(self, msg):
        a = np.array([msg.x, msg.y, msg.z])
        self.win.append(a)

        if self.state == POCKETS:
            self._collect_pocket(a)
        elif self.state == R_COLLECT:
            self._collect_r(a)

    # ── pocket collection ─────────────────────────────────────────────────────
    def _collect_pocket(self, a):
        if self.latest_q is None or len(self.win) < self.stability_win:
            return

        R      = quat_to_rot(self.latest_q)
        g_body = R.T @ np.array([0.0, 0.0, 1.0])
        pocket = classify_pocket(g_body, self.orient_thresh)

        if pocket is None or pocket in self.done:
            return
        if np.array(self.win).var(axis=0).max() >= self.stability_var:
            return

        self.pockets[pocket].append(a)
        n = len(self.pockets[pocket])

        if n % 50 == 0:
            self.get_logger().info(f'{pocket}: {n}/{self.samples_needed}')

        if n >= self.samples_needed:
            self.done.add(pocket)
            self.get_logger().info(f'{pocket} COMPLETE  ({len(self.done)}/6 done)')
            if len(self.done) == 6:
                self._start_countdown()

    # ── countdown ─────────────────────────────────────────────────────────────
    def _start_countdown(self):
        self.state       = COUNTDOWN
        self.countdown_n = self.countdown_secs
        self.get_logger().info(
            '\nAll 6 pockets done!  Place the sensor flat and release it.'
        )
        self._timer = self.create_timer(1.0, self._countdown_tick)

    def _countdown_tick(self):
        if self.countdown_n > 0:
            self.get_logger().info(f'Starting R collection in {self.countdown_n}...')
            self.countdown_n -= 1
        else:
            self._timer.cancel()
            self.get_logger().info(f'Collecting {self.r_samples} samples for R...')
            self.state = R_COLLECT

    # ── R collection ──────────────────────────────────────────────────────────
    def _collect_r(self, a):
        self.r_buf.append(a)
        if len(self.r_buf) >= self.r_samples:
            self.state = 'done'
            self._solve_and_save()

    # ── solve ─────────────────────────────────────────────────────────────────
    def _solve_and_save(self):
        means = {p: np.mean(self.pockets[p], axis=0) for p in POCKET_NAMES}

        # b and S via augmented least squares: m_i = S @ t_i + b
        T     = np.column_stack([TRUE_G[p] for p in POCKET_NAMES])
        M     = np.column_stack([means[p]  for p in POCKET_NAMES])
        T_aug = np.vstack([T, np.ones((1, 6))])
        Sb    = M @ np.linalg.pinv(T_aug)
        S     = Sb[:, :3]
        b     = Sb[:,  3]

        # R from fresh undisturbed samples collected after countdown
        S_inv    = np.linalg.inv(S)
        r_cal    = np.array([S_inv @ (a - b) for a in self.r_buf])  # calibrated
        R_noise  = np.cov((r_cal - r_cal.mean(axis=0)).T)            # 3×3

        out = {'b': b.tolist(), 'S': S.tolist(), 'R_accel': R_noise.tolist()}

        os.makedirs(os.path.dirname(os.path.abspath(self.output_path)), exist_ok=True)
        with open(self.output_path, 'w') as f:
            json.dump(out, f, indent=2)

        self.get_logger().info(f'Saved to {self.output_path}')
        self.get_logger().info(f'b       = {np.round(b, 4)}')
        self.get_logger().info(f'S       =\n{np.round(S, 6)}')
        self.get_logger().info(f'R_accel =\n{np.round(R_noise, 8)}')
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(ImuCalibrationNode())


if __name__ == '__main__':
    main()
