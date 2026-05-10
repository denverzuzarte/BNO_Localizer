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

POCKETS   = 'pockets'
COUNTDOWN = 'countdown'
R_COLLECT = 'r_collect'
DONE      = 'done'


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

        # What to update — all default to False (nothing updated unless requested)
        self.declare_parameter('update_b',       False)
        self.declare_parameter('update_S',       False)
        self.declare_parameter('update_R_accel', False)
        self.declare_parameter('update_R_gyro',  False)

        self.declare_parameter('orient_thresh',  0.95)
        self.declare_parameter('stability_var',  0.05)
        self.declare_parameter('stability_win',  50)
        self.declare_parameter('samples_needed', 500)
        self.declare_parameter('r_samples',      300)
        self.declare_parameter('countdown_secs', 5)
        self.declare_parameter('output_path',    DEFAULT_CONFIG_PATH)

        self.update_b       = self.get_parameter('update_b').value
        self.update_S       = self.get_parameter('update_S').value
        self.update_R_accel = self.get_parameter('update_R_accel').value
        self.update_R_gyro  = self.get_parameter('update_R_gyro').value

        self.orient_thresh  = self.get_parameter('orient_thresh').value
        self.stability_var  = self.get_parameter('stability_var').value
        self.stability_win  = self.get_parameter('stability_win').value
        self.samples_needed = self.get_parameter('samples_needed').value
        self.r_samples      = self.get_parameter('r_samples').value
        self.countdown_secs = self.get_parameter('countdown_secs').value
        self.output_path    = self.get_parameter('output_path').value

        self.need_pockets   = self.update_b or self.update_S
        self.need_r_collect = self.update_R_accel or self.update_R_gyro

        if not self.need_pockets and not self.need_r_collect:
            self.get_logger().warn(
                'Nothing to update. Pass at least one of:\n'
                '  --ros-args -p update_b:=true\n'
                '             -p update_S:=true\n'
                '             -p update_R_accel:=true\n'
                '             -p update_R_gyro:=true'
            )
            rclpy.shutdown()
            return

        updating = [k for k, v in [
            ('b', self.update_b), ('S', self.update_S),
            ('R_accel', self.update_R_accel), ('R_gyro', self.update_R_gyro),
        ] if v]
        self.get_logger().info(f'Will update: {", ".join(updating)}')

        self.create_subscription(Vector3,    '/imu1/data/accel',   self._accel_cb, 10)
        self.create_subscription(Quaternion, '/imu1/data/rot_vec', self._rot_cb,   10)
        self.create_subscription(Vector3,    '/imu1/data/gyro',    self._gyro_cb,  10)

        self.latest_q    = None
        self.latest_gyro = None
        self.win         = deque(maxlen=self.stability_win)
        self.pockets     = {p: [] for p in POCKET_NAMES}
        self.done        = set()
        self.r_accel_buf = []
        self.r_gyro_buf  = []
        self.r_tick      = 0
        self._timer      = None

        if self.need_pockets:
            self.state = POCKETS
            self.get_logger().info(
                'Place the sensor flat on each of the 6 faces and hold still.'
            )
        else:
            self.state = COUNTDOWN
            self._start_countdown()

    # ── sensor callbacks ──────────────────────────────────────────────────────

    def _gyro_cb(self, msg):
        self.latest_gyro = np.array([msg.x, msg.y, msg.z])

    def _rot_cb(self, msg):
        self.latest_q = (msg.x, msg.y, msg.z, msg.w)

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
                if self.need_r_collect:
                    self._start_countdown()
                else:
                    self._solve_and_save()

    # ── countdown ─────────────────────────────────────────────────────────────

    def _start_countdown(self):
        self.state       = COUNTDOWN
        self.countdown_n = self.countdown_secs
        self.get_logger().info('\nPlace the sensor flat and release it.')
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
        if self.update_R_accel:
            self.r_accel_buf.append(a)
        if self.update_R_gyro and self.latest_gyro is not None:
            self.r_gyro_buf.append(self.latest_gyro.copy())

        self.r_tick += 1
        if self.r_tick >= self.r_samples:
            self.state = DONE
            self._solve_and_save()

    # ── solve and save ────────────────────────────────────────────────────────

    def _solve_and_save(self):
        # Merge into existing calibration.json rather than overwriting
        out = {}
        if os.path.exists(self.output_path):
            with open(self.output_path) as f:
                out = json.load(f)

        if self.need_pockets:
            means = {p: np.mean(self.pockets[p], axis=0) for p in POCKET_NAMES}
            T     = np.column_stack([TRUE_G[p] for p in POCKET_NAMES])
            M     = np.column_stack([means[p]  for p in POCKET_NAMES])
            T_aug = np.vstack([T, np.ones((1, 6))])
            Sb    = M @ np.linalg.pinv(T_aug)
            S     = Sb[:, :3]
            b     = Sb[:,  3]

            if self.update_b:
                out['b'] = b.tolist()
                self.get_logger().info(f'b = {np.round(b, 4)}')
            if self.update_S:
                out['S'] = S.tolist()
                self.get_logger().info(f'S =\n{np.round(S, 6)}')

        if self.update_R_accel and len(self.r_accel_buf) >= 2:
            S_val = np.array(out.get('S', np.eye(3).tolist()))
            b_val = np.array(out.get('b', [0.0, 0.0, 0.0]))
            S_inv = np.linalg.inv(S_val)
            r_cal = np.array([S_inv @ (a - b_val) for a in self.r_accel_buf])
            R_accel = np.cov(r_cal.T)
            out['R_accel'] = R_accel.tolist()
            self.get_logger().info(f'R_accel =\n{np.round(R_accel, 8)}')

        if self.update_R_gyro and len(self.r_gyro_buf) >= 2:
            r_gyro = np.array(self.r_gyro_buf)
            R_gyro = np.cov(r_gyro.T)
            out['R_gyro'] = R_gyro.tolist()
            self.get_logger().info(f'R_gyro =\n{np.round(R_gyro, 8)}')

        os.makedirs(os.path.dirname(os.path.abspath(self.output_path)), exist_ok=True)
        with open(self.output_path, 'w') as f:
            json.dump(out, f, indent=2)

        self.get_logger().info(f'Saved to {self.output_path}')
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(ImuCalibrationNode())


if __name__ == '__main__':
    main()
