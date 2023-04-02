import numpy as np
from ma_dozer.configs.pydantic_config import BaseConfig


def nav_config_factory(add_measurement_errors: bool = False,
                       add_nav_errors: bool = False,
                       add_IC_errors: bool = False,
                       dt_nav: float = 0.05,
                       ARW_val_one_sigma: np.ndarray = None,
                       GRW_val_one_sigma: np.ndarray = None,
                       R_mat_params: np.ndarray = None,
                       IC_params_value: np.ndarray = None,
                       bias_value_one_sigma: np.ndarray = None,
                       drift_value_one_sigma: np.ndarray = None,
                       add_random_IMU: bool = None):
    # kalman params

    ARW = ARW_val_one_sigma * np.sqrt(dt_nav)  # x, y, z
    GRW = GRW_val_one_sigma * np.sqrt(dt_nav)

    Q_mat_params = np.array(([0.0, 0.0, 0.0],  # pos:   [m, m, m]
                             [GRW[0], GRW[1], GRW[2]],  # att:   rad [phi,theta, psi]
                             [ARW[0], ARW[1], ARW[2]],  # vel:   m/sec
                             [0, 0, 0],  # bias:  m/sec
                             [0, 0, 0],  # drift: rad/sec
                             ))

    R_mat_params[1, :] *= np.pi / 180
    IC_params_value[1, :] *= np.pi / 180

    if add_IC_errors:
        IC_params = IC_params_value
    else:
        IC_params = np.zeros((3, 3))

    if add_measurement_errors:
        cam_meas_pos_error = np.array(R_mat_params[0, :])
        cam_meas_att_error = np.array(R_mat_params[1, :])
    else:
        cam_meas_pos_error = np.zeros(3, )
        cam_meas_att_error = np.zeros(3, )

    if not add_nav_errors:
        bias_value_one_sigma = np.zeros(3, )  # x y z
        drift_value_one_sigma = np.zeros(3, )  # x y z

    if add_random_IMU:
        rand_vec = np.random.randn(12, )
    else:
        rand_vec = np.ones(12, )
    if add_nav_errors:
        true_bias = bias_value_one_sigma * rand_vec[0:3]
        true_drift = drift_value_one_sigma * rand_vec[3:6]
        true_ARW = ARW_val_one_sigma * rand_vec[6:9]
        true_GRW = GRW_val_one_sigma * rand_vec[9:12]
    else:
        true_bias = np.zeros((3,))
        true_drift = np.zeros((3,))
        true_ARW = np.zeros((3,))
        true_GRW = np.zeros((3,))

    return (cam_meas_pos_error,
            cam_meas_att_error,
            true_bias,
            true_drift,
            bias_value_one_sigma,
            drift_value_one_sigma,
            Q_mat_params,
            R_mat_params,
            IC_params,
            true_ARW,
            true_GRW)


class NavigationConfig(BaseConfig):

    add_measurement_errors: bool = False
    add_nav_errors: bool = False
    add_IC_errors: bool = False
    add_IC_errors_const_vec: bool = False
    dt_nav: float = 0.05
    dt_kal: float = 1.
    ARW_val_one_sigma: np.ndarray = np.zeros(3, )
    GRW_val_one_sigma: np.ndarray = np.zeros(3, )
    bias_value_one_sigma: np.ndarray = np.zeros(3, )  # x y z
    drift_value_one_sigma: np.ndarray = np.zeros(3, )  # x y z
    add_random_IMU: bool = False
    add_random_meas_noise: bool = False
    R_mat_params: np.ndarray = np.zeros((2, 3))
    IC_params_value: np.ndarray = np.zeros((3, 3))
    epsilon_dt_nav_to_dt_kal: float = None
    L_cut: int = None
    plt_navigation_convergence_flag: bool = False

    cam_meas_pos_error: np.ndarray = None
    cam_meas_att_error: np.ndarray = None
    true_bias: np.ndarray = None
    true_drift: np.ndarray = None
    Q_mat_params: np.ndarray = None
    IC_params: np.ndarray = None
    true_ARW: np.ndarray = None
    true_GRW: np.ndarray = None

    activate_kalman_filter: bool = True

    pose_position_noise: np.ndarray = np.array([-2, 2])
    pose_rotation_noise: np.ndarray = np.array([-2, 2])

    def __post_init__(self):
        (self.cam_meas_pos_error, self.cam_meas_att_error, self.true_bias,
         self.true_drift, self.bias_value_one_sigma, self.drift_value_one_sigma,
         self.Q_mat_params,
         self.R_mat_params, self.IC_params,
         self.true_ARW, self.true_GRW) = nav_config_factory(add_measurement_errors=self.add_measurement_errors,
                                                            add_nav_errors=self.add_nav_errors,
                                                            add_IC_errors=self.add_IC_errors,
                                                            dt_nav=self.dt_nav,
                                                            ARW_val_one_sigma=self.ARW_val_one_sigma,
                                                            GRW_val_one_sigma=self.GRW_val_one_sigma,
                                                            R_mat_params=self.R_mat_params,
                                                            IC_params_value=self.IC_params_value,
                                                            bias_value_one_sigma=self.bias_value_one_sigma,
                                                            drift_value_one_sigma=self.drift_value_one_sigma)
