# calibration.json keys managed by this file:
#   sigma_a      (float)    - RMS of ||a_lin|| at rest; SHOE accel noise floor
#   sigma_omega  (float)    - RMS of ||omega|| at rest; SHOE gyro noise floor
#   zupt_gamma   (float)    - SHOE threshold; T < gamma → stance phase (valley between bimodal peaks)
#   zupt_var     (float)    - ZUPT measurement noise variance; computed from sigma_a and shoe_window
#   b0           [x, y, z]  - mean linear acceleration at rest; should be near zero if BNO calibrated
#   std_acc      (float)    - per-axis std of linear acceleration at rest; Shivansh-style accel noise
#   T_bias       (float)    - Gauss-Markov bias time constant (heuristic: 300 s)
#   std_bias_rw  (float)    - bias random walk rate m/s²/√s (heuristic: 1e-4)

import json
import math
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3
from scipy.ndimage import gaussian_filter1d
from scipy.signal import find_peaks
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

DEFAULT_CONFIG_PATH = '/home/denver/BNO_Localizer/src/localiser/config/calibration.json'


class EskfCalibratorNode(Node):
    def __init__(self):
        super().__init__('eskf_calibrator')

        self.declare_parameter('still_duration',       20)
        self.declare_parameter('walk_duration',        30)
        self.declare_parameter('shoe_window',          30)
        self.declare_parameter('n_bins',               100)
        self.declare_parameter('config_path',          DEFAULT_CONFIG_PATH)
        self.declare_parameter('calibrate_shoe',         True)
        self.declare_parameter('calibrate_std_acc',      True)
        self.declare_parameter('calibrate_std_bias_rw',  True)

        self._still_duration        = int(self.get_parameter('still_duration').value or 20)
        self._walk_duration         = int(self.get_parameter('walk_duration').value  or 30)
        self._shoe_window           = int(self.get_parameter('shoe_window').value    or 30)
        self._n_bins                = int(self.get_parameter('n_bins').value         or 100)
        self._config_path           = str(self.get_parameter('config_path').value    or DEFAULT_CONFIG_PATH)
        self._calibrate_shoe        = bool(self.get_parameter('calibrate_shoe').value)
        self._calibrate_std_acc     = bool(self.get_parameter('calibrate_std_acc').value)
        self._calibrate_std_bias_rw = bool(self.get_parameter('calibrate_std_bias_rw').value)

        self._still_accel: list[np.ndarray] = []
        self._still_gyro:  list[np.ndarray] = []
        self._walk_accel:  list[np.ndarray] = []
        self._walk_gyro:   list[np.ndarray] = []

        self._phase       = 'still'
        self._sigma_a     = 0.0
        self._sigma_omega = 0.0
        self._b0          = np.zeros(3)
        self._std_acc     = 0.0

        self.create_subscription(Vector3, '/imu1/data/lin_acc', self._accel_cb, 10)
        self.create_subscription(Vector3, '/imu1/data/gyro',    self._gyro_cb,  10)

        self._still_timer = self.create_timer(float(self._still_duration), self._end_still_phase)
        self.get_logger().info(f'Phase 1/{self._still_duration}s — keep IMU flat and STILL.')

    def _accel_cb(self, msg: Vector3) -> None:
        a = np.array([msg.x, msg.y, msg.z])
        if self._phase == 'still':
            self._still_accel.append(a)
        else:
            self._walk_accel.append(a)

    def _gyro_cb(self, msg: Vector3) -> None:
        w = np.array([msg.x, msg.y, msg.z])
        if self._phase == 'still':
            self._still_gyro.append(w)
        else:
            self._walk_gyro.append(w)

    def _end_still_phase(self) -> None:
        self._still_timer.cancel()

        if not self._still_accel or not self._still_gyro:
            self.get_logger().error('No still samples collected — check topics are publishing.')
            rclpy.shutdown()
            return

        A = np.array(self._still_accel)   # (N, 3)
        W = np.array(self._still_gyro)    # (N, 3)

        # SHOE noise floor
        a_norms2 = np.sum(A ** 2, axis=1)
        w_norms2 = np.sum(W ** 2, axis=1)
        self._sigma_a     = math.sqrt(float(np.mean(a_norms2)))
        self._sigma_omega = math.sqrt(float(np.mean(w_norms2)))

        # Shivansh-style: per-axis bias and noise std
        self._b0      = A.mean(axis=0)                    # [bx, by, bz] m/s²
        self._std_acc = float(np.std(A - self._b0))       # pooled per-axis std

        self.get_logger().info(
            f'Still phase done.\n'
            f'  sigma_a={self._sigma_a:.6f}  sigma_omega={self._sigma_omega:.6f}\n'
            f'  b0=[{self._b0[0]:+.5f}, {self._b0[1]:+.5f}, {self._b0[2]:+.5f}] m/s²\n'
            f'  std_acc={self._std_acc:.6f} m/s²'
        )

        if not self._calibrate_shoe:
            self.get_logger().info('Skipping walk phase (calibrate_shoe=false).')
            self._write_config(gamma=None)
            rclpy.shutdown()
            return

        self._phase = 'walk'
        self.create_timer(float(self._walk_duration), self._end_walk_phase)
        self.get_logger().info(f'Phase 2/{self._walk_duration}s — START WALKING normally.')

    def _compute_T_series(self, accel_samples: list, gyro_samples: list) -> np.ndarray:
        sigma_a2     = self._sigma_a ** 2
        sigma_omega2 = self._sigma_omega ** 2
        W = self._shoe_window
        n = min(len(accel_samples), len(gyro_samples))
        T_vals = []
        for i in range(W, n + 1):
            T = 0.0
            for j in range(i - W, i):
                a = accel_samples[j]
                w = gyro_samples[j]
                T += np.dot(a, a) / sigma_a2 + np.dot(w, w) / sigma_omega2
            T_vals.append(T / W)
        return np.array(T_vals)

    def _find_valley(self, T_vals: np.ndarray) -> float | None:
        """Valley between bimodal peaks in log(T) space — minimum density point between the two tallest peaks."""
        log_T = np.log(T_vals[T_vals > 0])
        if len(log_T) < 2 * self._shoe_window:
            return None

        counts, edges = np.histogram(log_T, bins=self._n_bins)
        smoothed = gaussian_filter1d(counts.astype(float), sigma=2)
        centers  = 0.5 * (edges[:-1] + edges[1:])

        peaks, _ = find_peaks(smoothed, height=smoothed.max() * 0.1, distance=5)
        if len(peaks) < 2:
            return None

        top2 = peaks[np.argsort(smoothed[peaks])[-2:]]
        p1, p2 = sorted(top2)

        valley_idx = p1 + int(np.argmin(smoothed[p1:p2 + 1]))
        return float(np.exp(centers[valley_idx]))

    def _save_plot(self, T_still: np.ndarray, T_walk: np.ndarray, gamma: float, fallback: bool) -> None:
        log_still = np.log(T_still[T_still > 0])
        log_walk  = np.log(T_walk[T_walk   > 0])

        lo    = min(log_still.min(), log_walk.min())
        hi    = max(log_still.max(), log_walk.max())
        edges = np.linspace(lo, hi, self._n_bins + 1)
        centers = 0.5 * (edges[:-1] + edges[1:])
        width   = edges[1] - edges[0]

        c_still, _ = np.histogram(log_still, bins=edges)
        c_walk,  _ = np.histogram(log_walk,  bins=edges)

        max_still = c_still.max() or 1
        max_walk  = c_walk.max()  or 1

        fig, ax = plt.subplots(figsize=(12, 5))

        for i in range(len(centers)):
            if c_still[i] > 0:
                ax.bar(centers[i], c_still[i], width=width,
                       color=plt.cm.Blues(0.3 + 0.7 * c_still[i] / max_still), alpha=0.8)
            if c_walk[i] > 0:
                ax.bar(centers[i], c_walk[i], width=width,
                       color=plt.cm.Reds(0.3 + 0.7 * c_walk[i] / max_walk), alpha=0.8)

        s_still = gaussian_filter1d(c_still.astype(float), sigma=2)
        s_walk  = gaussian_filter1d(c_walk.astype(float),  sigma=2)
        ax.plot(centers, s_still, color='#1565c0', linewidth=2, label=f'still ({len(T_still)} windows)')
        ax.plot(centers, s_walk,  color='#b71c1c', linewidth=2, label=f'walking ({len(T_walk)} windows)')

        ax.axvline(math.log(gamma), color='black', linewidth=2, linestyle='--',
                   label=f'gamma = {gamma:.2f}{"  (fallback)" if fallback else "  (valley)"}')

        ax.set_xlabel('log(T)  —  SHOE statistic')
        ax.set_ylabel('count')
        ax.set_title('SHOE T distribution: still (blue) vs walking (red), intensity = density')
        ax.legend()

        out_path = '/tmp/shoe_threshold.png'
        fig.savefig(out_path, dpi=150, bbox_inches='tight')
        plt.close(fig)
        self.get_logger().info(f'Plot saved: {out_path}')

    def _write_config(self, gamma: float | None) -> None:
        if not self._config_path:
            return
        try:
            with open(self._config_path) as f:
                cfg = json.load(f)
            if self._calibrate_shoe:
                cfg['sigma_a']    = self._sigma_a
                cfg['sigma_omega'] = self._sigma_omega
                if gamma is not None:
                    cfg['zupt_gamma'] = gamma
            if self._calibrate_std_acc:
                cfg['b0']      = self._b0.tolist()
                cfg['std_acc'] = self._std_acc
            if self._calibrate_std_bias_rw:
                cfg['T_bias']      = 300.0
                cfg['std_bias_rw'] = 1e-4
            with open(self._config_path, 'w') as f:
                json.dump(cfg, f, indent=2)
            self.get_logger().info(f'calibration.json updated: {self._config_path}')
        except Exception as e:
            self.get_logger().error(f'Failed to write config: {e}')

    def _end_walk_phase(self) -> None:
        if not self._walk_accel or not self._walk_gyro:
            self.get_logger().error('No walking samples collected.')
            rclpy.shutdown()
            return

        T_walk = self._compute_T_series(self._walk_accel, self._walk_gyro)
        if len(T_walk) == 0:
            self.get_logger().error(
                f'Not enough walking samples for a window of {self._shoe_window}.'
            )
            rclpy.shutdown()
            return

        T_still  = self._compute_T_series(self._still_accel, self._still_gyro)
        gamma    = self._find_valley(T_walk)
        fallback = gamma is None

        if fallback:
            gamma = float(np.percentile(T_still, 99)) * 5.0

        assert gamma is not None
        T_stance = T_walk[T_walk <  gamma]
        T_swing  = T_walk[T_walk >= gamma]

        # heuristic values (from Shivansh/literature)
        T_bias_val   = 300.0
        std_bias_rw  = 1e-4

        self.get_logger().info(
            f'\n'
            f'=== ESKF Calibration Results ===\n'
            f'\n'
            f'  --- Accel noise (Shivansh-style) ---\n'
            f'  b0      : [{self._b0[0]:+.6f}, {self._b0[1]:+.6f}, {self._b0[2]:+.6f}] m/s²\n'
            f'  std_acc : {self._std_acc:.6f} m/s²\n'
            f'\n'
            f'  --- SHOE calibration ---\n'
            f'  sigma_a     : {self._sigma_a:.6f} m/s²\n'
            f'  sigma_omega : {self._sigma_omega:.6f} rad/s\n'
            f'\n'
            f'  Walking T distribution ({len(T_walk)} windows):\n'
            f'    min : {float(np.min(T_walk)):.4f}\n'
            f'    max : {float(np.max(T_walk)):.4f}\n'
            f'\n'
            f'  gamma = {gamma:.4f}  '
            f'{"(valley between stance/swing peaks)" if not fallback else "(FALLBACK — no clear valley, used static P99×5)"}\n'
            f'  Stance windows (T < gamma) : {len(T_stance)}\n'
            f'  Swing  windows (T >= gamma): {len(T_swing)}\n'
            f'\n'
            f'  --- Heuristic values (from Shivansh/literature) ---\n'
            f'  T_bias     : {T_bias_val} s  (Gauss-Markov bias time constant)\n'
            f'  std_bias_rw: {std_bias_rw} m/s²/√s  (bias random walk rate)\n'
        )

        self._save_plot(T_still, T_walk, gamma, fallback)
        self._write_config(gamma=gamma)
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = EskfCalibratorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
