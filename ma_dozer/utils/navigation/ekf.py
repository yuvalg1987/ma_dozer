from dataclasses import dataclass
import numpy as np
from typing import Tuple
import navpy as nav

from ma_dozer.configs.navigation_config import NavigationConfig
from ma_dozer.utils.navigation.strapdown import StrapDown


@dataclass
class ind:
    dp: Tuple = (0, 3)
    dpsi: Tuple = (3, 6)
    dv: Tuple = (6, 9)
    dBc: Tuple = (9, 12)
    dDc: Tuple = (12, 15)


@dataclass
class error_model_meas:
    Q_mat_params: np.ndarray = None
    R_mat_params: np.ndarray = None


class EKF:
    def __init__(self, config: NavigationConfig = None, sd: StrapDown = None):
        """
        X = [dP, dPsi, dV, dBc, dDc]
        nav_config = dict({'pos_erorr': pos_error,
                       'att_error': att_error,
                       'add_nav_error': error_model,
                       'dt_nav': dt_nav,
                       'dt_kal': dt_kal,
                       'R_mat_params': R_mat_params,
                       'Q_mat_params': Q_mat_params
    })
        """

        self.ind = ind()
        self.error_model = error_model_meas()
        self.dt_kal = config.dt_kal
        self.state_size = 15
        self.meaurement_size = 6
        self.x_plus = np.zeros((self.state_size))
        self.A = np.eye(self.state_size, self.state_size)
        self.Q = np.zeros((self.state_size, self.state_size))
        self.P_plus = np.zeros((self.state_size, self.state_size))
        self.sd = sd
        self.prev_kalman_velocity = self.sd.measured_state.vel
        self.prev_kalman_pos = self.sd.measured_state.pos

        self.R = np.zeros((self.meaurement_size, self.meaurement_size))
        self.K = np.zeros((self.state_size, self.meaurement_size ))
        self.H = np.zeros((self.meaurement_size, self.state_size))

        self.error_model.Q_mat_params = config.Q_mat_params.copy()
        self.error_model.R_mat_params = config.R_mat_params.copy()

        self.init_R_mat()
        self.init_Q_mat()
        self.init_A_mat()
        self.init_H_mat()

        self.init_x_plus_P_Plus(config=config)
        self.I_P_size = np.eye(self.state_size, self.state_size) # allocate for kalman update

    def kalman_update(self, cam_measurement: np.ndarray = None):
        self.init_A_mat()
        self.kalman_estimate(cam_measurement)
        self.reset_after_kalman()

    def reset_after_kalman(self):

        a = 1
        EE = self.x_plus

        # ---Pos reset - --
        self.sd.measured_state_prev.pos -= EE[self.ind.dp[0]:self.ind.dp[1]].T

        # ---Vel reset - --
        self.sd.measured_state_prev.vel -= EE[self.ind.dv[0]:self.ind.dv[1]].T

        # ---Psi reset - --
        # Q_ee = VectorEuler2Quat(EE(ind.Psi(1)), EE(ind.Psi(2)), EE(ind.Psi(3))) # inputs: (dx, dy, dz)
        # SdData_prev.Quat = MulQuat(Q_ee, SdData_prev.Quat); # DCM_n = DCM_p * DCM_ee
        # [SdData_prev.Euler.psi, SdData_prev.Euler.teta, SdData_prev.Euler.phi] = Quat2Euler(SdData_prev.Quat)
        EE_psi_x_y_z = -EE[self.ind.dpsi[0]:self.ind.dpsi[1]]
        EE_psi_z_y_x = np.flip(EE_psi_x_y_z)
        D_ee = nav.angle2dcm(EE_psi_z_y_x[0], EE_psi_z_y_x[1], EE_psi_z_y_x[2])

        D_new = (nav.angle2dcm(self.sd.measured_state_prev.att[0],
                               self.sd.measured_state_prev.att[1],
                               self.sd.measured_state_prev.att[2])) @ D_ee

        self.sd.measured_state_prev.att = np.asarray(nav.dcm2angle(D_new))

        # ---Bias Drift reset - --
        self.sd.estimated_acc_bias += EE[self.ind.dBc[0]:self.ind.dBc[1]]  # x,y,z
        self.sd.estimated_gyro_drift += EE[self.ind.dDc[0]:self.ind.dDc[1]] # x,y,z

        # --- Zeroing - --
        self.sd.sum_dcm = np.zeros((3,3))
        self.x_plus *= 0 # zero for next iteration.

    def init_H_mat(self):
        self.H[self.ind.dp[0]:self.ind.dp[1], self.ind.dp[0]:self.ind.dp[1]] = np.eye((3))
        self.H[self.ind.dpsi[0]:self.ind.dpsi[1], self.ind.dpsi[0]:self.ind.dpsi[1]] = np.eye((3))

    def init_R_mat(self):
        self.R[self.ind.dp[0]:self.ind.dp[1], self.ind.dp[0]:self.ind.dp[1]] = np.diag((self.error_model.R_mat_params[0].copy())**2)
        self.R[self.ind.dpsi[0]:self.ind.dpsi[1], self.ind.dpsi[0]:self.ind.dpsi[1]] = np.diag((self.error_model.R_mat_params[1].copy())**2)

    def init_Q_mat(self):
        self.Q[self.ind.dp[0]:self.ind.dp[1], self.ind.dp[0]:self.ind.dp[1]]            = np.diag((self.error_model.Q_mat_params[0].copy())**2)
        self.Q[self.ind.dpsi[0]:self.ind.dpsi[1], self.ind.dpsi[0]:self.ind.dpsi[1]]    = np.diag((self.error_model.Q_mat_params[1].copy())**2)
        self.Q[self.ind.dv[0]:self.ind.dv[1], self.ind.dv[0]:self.ind.dv[1]]            = np.diag((self.error_model.Q_mat_params[2].copy())**2)
        self.Q[self.ind.dBc[0]:self.ind.dBc[1], self.ind.dBc[0]:self.ind.dBc[1]]        = np.diag((self.error_model.Q_mat_params[3].copy())**2)
        self.Q[self.ind.dDc[0]:self.ind.dDc[1], self.ind.dDc[0]:self.ind.dDc[1]]        = np.diag((self.error_model.Q_mat_params[4].copy())**2)

    def init_A_mat(self):
        self.A[self.ind.dp[0]:self.ind.dp[1], self.ind.dv[0]:self.ind.dv[1]]         = np.eye(3) * self.dt_kal
        self.A[self.ind.dpsi[0]:self.ind.dpsi[1], self.ind.dDc[0]:self.ind.dDc[1]]   = self.sd.sum_dcm.copy()
        self.A[self.ind.dv[0]:self.ind.dv[1], self.ind.dBc[0]:self.ind.dBc[1]] = self.sd.sum_dcm.copy()
        self.A[self.ind.dv[0]:self.ind.dv[1], self.ind.dpsi[0]:self.ind.dpsi[1]] = self.Calc_As()
        self.A[self.ind.dv[0]:self.ind.dv[1], self.ind.dp[0]:self.ind.dp[1]] = self.Calc_F()

    def Calc_F(self):

        F = np.eye(3) * (1/self.dt_kal)

        return F

    def Calc_As(self):

        VL_1 = self.prev_kalman_velocity  # prev velocity.
        VL_2 = self.sd.measured_state.vel # current measured velocity

        A_s_xy = VL_2[2] - VL_1[2] - self.sd.g * self.dt_kal
        A_s_xz = VL_2[1] - VL_1[1]
        A_s_yz = VL_2[0] - VL_1[0]

        A_s              = np.array([[0      , -A_s_xy , +A_s_xz],
                                    [+A_s_xy,    0    , -A_s_yz],
                                    [-A_s_xz, +A_s_yz ,    0   ]])

        self.prev_kalman_velocity = VL_2 # for next cycle.

        return A_s

    def init_x_plus_P_Plus(self, config: NavigationConfig = None):

        if config.add_IC_errors:
            sf_ic_errors_x = 1
            d_vel = (3**2) * config.IC_params[0, :].max()/self.dt_kal
            sf_ic_errors_P = np.array([3, 3, d_vel, 3, 3, ])
        else:
            sf_ic_errors_x = 0
            sf_ic_errors_P = np.zeros(5,)
        if config.add_nav_errors:
            sf_nav_errors = 1
        else:
            sf_nav_errors = 0

        self.x_plus[self.ind.dp[0]:self.ind.dp[1]] = sf_ic_errors_x*config.IC_params[0, :].copy()
        self.x_plus[self.ind.dpsi[0]:self.ind.dpsi[1]] = sf_ic_errors_x*config.IC_params[1, :].copy()
        self.x_plus[self.ind.dv[0]:self.ind.dv[1]] = sf_ic_errors_x*config.IC_params[2, :].copy()
        self.x_plus[self.ind.dBc[0]:self.ind.dBc[1]] = sf_nav_errors*config.true_bias
        self.x_plus[self.ind.dDc[0]:self.ind.dDc[1]] = sf_nav_errors*config.true_drift

        self.P_plus[self.ind.dp[0]:self.ind.dp[1],self.ind.dp[0]:self.ind.dp[1]]         = sf_ic_errors_P[0] * np.diag((config.IC_params[0, :].copy())**2)
        self.P_plus[self.ind.dpsi[0]:self.ind.dpsi[1],self.ind.dpsi[0]:self.ind.dpsi[1]] = sf_ic_errors_P[1] * np.diag((config.IC_params[1, :].copy()**2))
        self.P_plus[self.ind.dv[0]:self.ind.dv[1],self.ind.dv[0]:self.ind.dv[1]]         = sf_ic_errors_P[2] * np.diag((config.IC_params[2, :].copy())**2)
        self.P_plus[self.ind.dBc[0]:self.ind.dBc[1],self.ind.dBc[0]:self.ind.dBc[1]]     = sf_ic_errors_P[3] * np.diag((config.true_bias.copy())**2)
        self.P_plus[self.ind.dDc[0]:self.ind.dDc[1],self.ind.dDc[0]:self.ind.dDc[1]]     = sf_ic_errors_P[4] * np.diag((config.true_drift.copy())**2)

    def kalman_estimate(self, cam_measurement: np.ndarray = None):

        # Z
        Z = np.zeros(self.meaurement_size,)
        Z[self.ind.dp[0]:self.ind.dp[1]]     = self.sd.measured_state.pos.copy() - cam_measurement[0:3].copy()

        nav_att = self.sd.measured_state.att.copy()
        meas_att = cam_measurement[3:6].copy()
        curr_att_err_z_y_x = nav.dcm2angle(
            (nav.angle2dcm(meas_att[0], meas_att[1], meas_att[2])).T @
            (nav.angle2dcm(nav_att[0], nav_att[1], nav_att[2]))
            )
        curr_att_err_x_y_z = np.flip(curr_att_err_z_y_x)
        Z[self.ind.dpsi[0]:self.ind.dpsi[1]] = curr_att_err_x_y_z # [x,y,z]  # D_N_sd_to_N_m = D_b_to_N_m * D_N_to_b_sd

        # Kalman
        # time update(predict):
        X_minus = self.A @ self.x_plus
        P_minus = self.A @ self.P_plus @ self.A.T + self.Q

        # kalman gain:
        K = P_minus @ self.H.T @ np.linalg.inv(self.H @ P_minus @ self.H.T + self.R)

        # meassurement update(filtering):
        X_plus = X_minus + K @ (Z - self.H @ X_minus)
        P_plus = (self.I_P_size - K @ self.H) @ P_minus @ (self.I_P_size - K @ self.H).T + K @ self.R @ K.T

        # save for reset. not for next iter, since EE is zeroed at reset!
        self.x_plus = X_plus
        self.P_plus = P_plus

        return


