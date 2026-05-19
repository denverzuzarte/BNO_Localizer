from scipy.spatial.transform import Rotation
import numpy as np

def quat_multiply(p, q):
    r = Rotation.from_quat(p) * Rotation.from_quat(q)
    return r.as_quat()

def quat_conjugate(q):
    return Rotation.from_quat(q).inv().as_quat()

def rotvec_to_quat(v):
    return Rotation.from_rotvec(v).as_quat()

def quat_to_rotvec(q):
    return Rotation.from_quat(q).as_rotvec()

def quat_to_rot(q):
    return Rotation.from_quat(q).as_matrix()

def skew(v):
    return np.array([[ 0,    -v[2],  v[1]],
                     [ v[2],  0,    -v[0]],
                     [-v[1],  v[0],  0   ]])

class ESKF():
    def __init__(self,
             var_acc, var_gyro, var_acc_bias, var_gyro_bias,
             init_quat=None,
             init_var_pos=1.0, init_var_vel=0.1, init_var_ori=0.1,
             init_var_acc_bias=0.01, init_var_gyro_bias=0.01):

        self.x = np.zeros(16)
        self.x[6:10] = np.array(init_quat) if init_quat is not None else np.array([0.0, 0.0, 0.0, 1.0])
        self._R_init = quat_to_rot(self.x[6:10]).T

        self.P = np.diag([
            init_var_pos,       init_var_pos,       init_var_pos,
            init_var_vel,       init_var_vel,       init_var_vel,
            init_var_ori,       init_var_ori,       init_var_ori,
            init_var_acc_bias,  init_var_acc_bias,  init_var_acc_bias,
            init_var_gyro_bias, init_var_gyro_bias, init_var_gyro_bias,
        ])

        self.var_acc        = var_acc
        self.var_gyro       = var_gyro
        self.var_acc_bias   = var_acc_bias
        self.var_gyro_bias  = var_gyro_bias

    def _inject(self, dx):
        self.x[0:3]   += dx[0:3]
        self.x[3:6]   += dx[3:6]
        self.x[6:10]   = quat_multiply(self.x[6:10], rotvec_to_quat(dx[6:9]))
        self.x[6:10]  /= np.linalg.norm(self.x[6:10])
        self.x[10:13] += dx[9:12]
        self.x[13:16] += dx[12:15]
        G = np.eye(3) - skew(0.5 * dx[6:9])
        self.P[6:9, 6:9] = G @ self.P[6:9, 6:9] @ G.T

    def _kf_update(self, H, z, R_meas):
        S = H @ self.P @ H.T + R_meas
        K = self.P @ H.T @ np.linalg.inv(S)
        dx = K @ z
        I_KH = np.eye(15) - K @ H
        self.P = I_KH @ self.P @ I_KH.T + K @ R_meas @ K.T
        return dx

    def predict(self, a_m, dt):
        acc_body  = a_m - self.x[10:13]
        R         = self._R_init @ quat_to_rot(self.x[6:10])
        acc_world = R @ acc_body

        self.x[0:3] += self.x[3:6] * dt + 0.5 * acc_world * dt**2
        self.x[3:6] += acc_world * dt
        # quaternion held constant — updated only by update_quat

        F = np.eye(15)
        F[0:3,  3:6]  = np.eye(3) * dt
        F[3:6,  6:9]  = -R @ skew(acc_body) * dt
        F[3:6,  9:12] = -R * dt

        Q = np.zeros((15, 15))
        Q[3:6,   3:6]   = self.var_acc       * dt    * np.eye(3)
        Q[6:9,   6:9]   = self.var_gyro      * dt**2 * np.eye(3)
        Q[9:12,  9:12]  = self.var_acc_bias  * dt    * np.eye(3)
        Q[12:15, 12:15] = self.var_gyro_bias * dt    * np.eye(3)

        self.P = F @ self.P @ F.T + Q

    def update_quat(self, q_meas, R_ori):
        q_nominal = self.x[6:10]
        delta_q   = quat_multiply(quat_conjugate(q_nominal), q_meas)
        dtheta    = quat_to_rotvec(delta_q)

        H = np.zeros((3, 15))
        H[0:3, 6:9] = np.eye(3)

        dx = self._kf_update(H, dtheta, R_ori)
        self._inject(dx)
        return dtheta

    def update_zupt(self, R_zupt):
        H = np.zeros((3, 15))
        H[0:3, 3:6] = np.eye(3)
        dx = self._kf_update(H, -self.x[3:6], R_zupt)
        self._inject(dx)
        return True

    def update_zero_pos(self):
        """Anchor position to origin. R_pos hardcoded — position = origin is a definition."""
        R_pos = 1e-4 * np.eye(3)
        H     = np.zeros((3, 15))
        H[0:3, 0:3] = np.eye(3)
        dx = self._kf_update(H, -self.x[0:3], R_pos)
        self._inject(dx)
