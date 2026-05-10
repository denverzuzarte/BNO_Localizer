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
             init_var_pos=1.0, init_var_vel=0.1, init_var_ori=0.1,
             init_var_acc_bias=0.01, init_var_gyro_bias=0.01,
             g=9.81):


    #initialise filter
    self.x = np.zeros(16) # used quaternion for rotation to avoid gimbal lock (during barrel roll)
    self.x[6:10] = np.array([0.0, 0.0, 0.0, 1.0])  # identity quat (x,y,z,w)

    self.P = np.diag([
        init_var_pos,       init_var_pos,       init_var_pos,
        init_var_vel,       init_var_vel,       init_var_vel,
        init_var_ori,       init_var_ori,       init_var_ori,
        init_var_acc_bias,  init_var_acc_bias,  init_var_acc_bias,
        init_var_gyro_bias, init_var_gyro_bias, init_var_gyro_bias,
    ])

    # Q scalars (process noise)
    self.var_acc        = var_acc
    self.var_gyro       = var_gyro
    self.var_acc_bias   = var_acc_bias
    self.var_gyro_bias  = var_gyro_bias

    self.g_world = np.array([0.0, 0.0, -g])
    
    def update_accel(self, a_m, R_accel):
        # expected gravity in body frame when vehicle is not accelerating
        R = quat_to_rot(self.x[6:10])
        h = R.T @ (-self.g_world)

        # innovation
        z = a_m - h

        # H - Jacobian of gravity measurement w.r.t δθ
        H = np.zeros((3, 15))
        H[0:3, 6:9] = skew(h)

        # Kalman gain
        S = H @ self.P @ H.T + R_accel
        K = self.P @ H.T @ np.linalg.inv(S)

        # inject correction into nominal state
        dx = K @ z
        self.x[0:3] += dx[0:3]
        self.x[3:6] += dx[3:6]
        self.x[6:10] = quat_multiply(self.x[6:10], rotvec_to_quat(dx[6:9]))
        self.x[6:10] /= np.linalg.norm(self.x[6:10])
        self.x[10:13] += dx[9:12]
        self.x[13:16] += dx[12:15]

        # update covariance - Joseph form
        I_KH = np.eye(15) - K @ H
        self.P = I_KH @ self.P @ I_KH.T + K @ R_accel @ K.T

   def update_quat(self, q_meas, R_ori):
        # innovation - difference between predicted and measured orientation
        q_nominal = self.x[6:10]
        delta_q = quat_multiply(quat_conjugate(q_nominal), q_meas)
        dtheta = quat_to_rotvec(delta_q)

        # H - measurement Jacobian, selects dtheta block from error state
        H = np.zeros((3, 15))
        H[0:3, 6:9] = np.eye(3)

        # Kalman gain
        S = H @ self.P @ H.T + R_ori
        K = self.P @ H.T @ np.linalg.inv(S)

        # inject correction into nominal state
        dx = K @ dtheta
        self.x[0:3] += dx[0:3]
        self.x[3:6] += dx[3:6]
        self.x[6:10] = quat_multiply(self.x[6:10], rotvec_to_quat(dx[6:9]))
        self.x[6:10] /= np.linalg.norm(self.x[6:10])
        self.x[10:13] += dx[9:12]
        self.x[13:16] += dx[12:15]

        # update covariance 
        I_KH = np.eye(15) - K @ H
        self.P = I_KH @ self.P @ I_KH.T + K @ R_ori @ K.T

        return dtheta 

    def predict(self, a_m, omega_m, dt):
        # remove estimated biases
        acc_body = a_m - self.x[10:13]
        omega = omega_m - self.x[13:16]

        # rotate accel into world frame
        R = quat_to_rot(self.x[6:10])
        acc_world = R @ acc_body

        # integrate nominal state
        self.x[0:3] += self.x[3:6] * dt + 0.5 * (acc_world + self.g_world) * dt**2
        self.x[3:6] += (acc_world + self.g_world) * dt
        dq = rotvec_to_quat(omega * dt)
        self.x[6:10] = quat_multiply(self.x[6:10], dq)
        self.x[6:10] /= np.linalg.norm(self.x[6:10])

        # F - 15×15 state transition Jacobian
        F = np.eye(15)
        F[0:3,  3:6] = np.eye(3) * dt
        F[3:6,  6:9] = -R @ skew(acc_body) * dt
        F[3:6,  9:12] = -R * dt
        F[6:9,  6:9] = Rotation.from_rotvec(omega * dt).as_matrix().T
        F[6:9,  12:15] = -np.eye(3) * dt

        # Q - 15×15 process noise
        Q = np.zeros((15, 15))
        Q[3:6,   3:6] = self.var_acc * dt**2 * np.eye(3)
        Q[6:9,   6:9] = self.var_gyro * dt**2 * np.eye(3)
        Q[9:12,  9:12] = self.var_acc_bias * dt * np.eye(3)
        Q[12:15, 12:15] = self.var_gyro_bias * dt * np.eye(3)

        # propagate covariance
        self.P = F @ self.P @ F.T + Q
