import numpy as np
from ma_dozer.configs.pydantic_config import BaseModel
from pydantic import validator

# def nav_config_factory(add_measurement_errors: bool = False,
#                        add_nav_errors: bool = False,
#                        add_IC_errors: bool = False,
#                        dt_nav: float = 0.05,
#                        ARW_val_one_sigma: np.ndarray = None,
#                        GRW_val_one_sigma: np.ndarray = None,
#                        R_mat_params: np.ndarray = None,
#                        IC_params_value: np.ndarray = None,
#                        bias_value_one_sigma: np.ndarray = None,
#                        drift_value_one_sigma: np.ndarray = None,
#                        add_random_IMU: bool = None):
#     # kalman params
#
#     ARW = ARW_val_one_sigma * np.sqrt(dt_nav)  # x, y, z
#     GRW = GRW_val_one_sigma * np.sqrt(dt_nav)
#
#     Q_mat_params = np.array(([0.0, 0.0, 0.0],  # pos:   [m, m, m]
#                              [GRW[0], GRW[1], GRW[2]],  # att:   rad [phi,theta, psi]
#                              [ARW[0], ARW[1], ARW[2]],  # vel:   m/sec
#                              [0, 0, 0],  # bias:  m/sec
#                              [0, 0, 0],  # drift: rad/sec
#                              ))
#
#     R_mat_params[1, :] *= np.pi / 180
#     IC_params_value[1, :] *= np.pi / 180
#
#     if add_IC_errors:
#         IC_params = IC_params_value
#     else:
#         IC_params = np.zeros((3, 3))
#
#     if add_measurement_errors:
#         cam_meas_pos_error = np.array(R_mat_params[0, :])
#         cam_meas_att_error = np.array(R_mat_params[1, :])
#     else:
#         cam_meas_pos_error = np.zeros(3, )
#         cam_meas_att_error = np.zeros(3, )
#
#     if not add_nav_errors:
#         bias_value_one_sigma = np.zeros(3, )  # x y z
#         drift_value_one_sigma = np.zeros(3, )  # x y z
#
#     if add_random_IMU:
#         rand_vec = np.random.randn(12, )
#     else:
#         rand_vec = np.ones(12, )
#     if add_nav_errors:
#         true_bias = bias_value_one_sigma * rand_vec[0:3]
#         true_drift = drift_value_one_sigma * rand_vec[3:6]
#         true_ARW = ARW_val_one_sigma * rand_vec[6:9]
#         true_GRW = GRW_val_one_sigma * rand_vec[9:12]
#     else:
#         true_bias = np.zeros((3,))
#         true_drift = np.zeros((3,))
#         true_ARW = np.zeros((3,))
#         true_GRW = np.zeros((3,))
#
#     return (cam_meas_pos_error,
#             cam_meas_att_error,
#             true_bias,
#             true_drift,
#             bias_value_one_sigma,
#             drift_value_one_sigma,
#             Q_mat_params,
#             R_mat_params,
#             IC_params,
#             true_ARW,
#             true_GRW)

class NavigationConfig(BaseModel):

    add_measurement_errors: bool = False
    add_nav_errors: bool = False
    add_IC_errors: bool = False
    add_IC_errors_const_vec: bool = False
    add_random_IMU: bool = False
    add_random_meas_noise: bool = False
    activate_kalman_filter: bool = True

    dt_nav: float = 0.05
    dt_kal: float = 1.

    ARW_val_one_sigma: np.ndarray = np.zeros(3, )
    GRW_val_one_sigma: np.ndarray = np.zeros(3, )
    bias_value_one_sigma: np.ndarray = np.zeros(3, )  # x y z
    drift_value_one_sigma: np.ndarray = np.zeros(3, )  # x y z

    ARW: np.ndarray = None
    GRW: np.ndarray = None

    Q_mat_params: np.ndarray = None
    R_mat_params: np.ndarray = np.zeros((2, 3))

    IC_params_value: np.ndarray = np.zeros((3, 3))
    IC_params: np.ndarray = None

    rand_vec: np.ndarray = None
    cam_meas_pos_error: np.ndarray = None
    cam_meas_att_error: np.ndarray = None
    true_bias: np.ndarray = None
    true_drift: np.ndarray = None
    true_ARW: np.ndarray = None
    true_GRW: np.ndarray = None

    pose_position_noise: np.ndarray = np.array([-2, 2])
    pose_rotation_noise: np.ndarray = np.array([-2, 2])

    # For Simulation
    epsilon_dt_nav_to_dt_kal: float = None
    L_cut: int = None
    plt_navigation_convergance_flag: bool = False


    @validator("ARW", always=True)
    def init_ARW(cls, v, values):
        return values['ARW_val_one_sigma'] * np.sqrt(values['dt_nav'])  # x, y, z

    @validator("GRW", always=True)
    def init_GRW(cls, v, values):
        return values['GRW_val_one_sigma'] * np.sqrt(values['dt_nav'])

    @validator("Q_mat_params", always=True)
    def init_Q_mat_params(cls, v, values):
        return np.array(([0.0, 0.0, 0.0],  # pos:   [m, m, m]
                         [values['GRW'][0], values['GRW'][1], values['GRW'][2]],  # att:   rad [phi,theta, psi]
                         [values['ARW'][0], values['ARW'][1], values['ARW'][2]],  # vel:   m/sec
                         [0, 0, 0],  # bias:  m/sec
                         [0, 0, 0],  # drift: rad/sec
                                  ))

    @validator("R_mat_params", always=True)
    def init_R_mat_params(cls, v, values):
        v[1, :] *= np.pi / 180
        return v

    @validator("IC_params_value", always=True)
    def init_IC_params_value(cls, v, values):
        v[1, :] *= np.pi / 180
        return v

    @validator("IC_params", always=True)
    def init_IC_params(cls, v, values):
        if values['add_IC_errors']:
            return values['IC_params_value']
        else:
            return np.zeros((3, 3))

    @validator("rand_vec", always=True)
    def init_rand_vec(cls, v, values):
        if values['add_random_IMU']:
            return np.random.randn(12, )
        else:
            return np.ones(12, )

    @validator("cam_meas_pos_error", always=True)
    def init_cam_meas_pos_error(cls, v, values):
        if values['add_measurement_errors']:
            return np.array(values['R_mat_params'][0, :])
        else:
            return np.zeros(3, )

    @validator("cam_meas_att_error", always=True)
    def init_cam_meas_att_error(cls, v, values):
        if values['add_measurement_errors']:
            return np.array(values['R_mat_params'][1, :])
        else:
            return np.zeros(3, )

    @validator("true_bias", always=True)
    def init_true_bias(cls, v, values):
        if values['add_nav_errors']:
            return values['bias_value_one_sigma'] * values['rand_vec'][0:3]
        else:
            return np.zeros((3,))

    @validator("true_drift", always=True)
    def init_true_drift(cls, v, values):
        if values['add_nav_errors']:
            return values['drift_value_one_sigma'] * values['rand_vec'][3:6]
        else:
            return np.zeros((3,))

    @validator("true_ARW", always=True)
    def init_true_ARW(cls, v, values):
        if values['add_nav_errors']:
            return values['ARW_val_one_sigma'] * values['rand_vec'][6:9]
        else:
            return np.zeros((3,))

    @validator("true_GRW", always=True)
    def init_true_GRW(cls, v, values):
        if values['add_nav_errors']:
            return values['GRW_val_one_sigma'] * values['rand_vec'][9:12]
        else:
            return np.zeros((3,))
