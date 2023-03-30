import numpy as np
import navpy as nav

from ma_dozer.utils.helpers.classes import NavState


def copy_nav_state(src: NavState):
    dst = NavState(time=src.time,
                   pos=src.pos.copy(),
                   vel=src.vel.copy(),
                   att=src.att.copy())
    return dst


def add_IC_errors(IC_params: np.ndarray, measured_nav_state: NavState = None, random_vector: np.ndarray = None):
    """
    :param measured_nav_state: a place holder containing a copy version of the true nav state for this function to add
    the erorrs and get the measured
    :return:
    logic: since IC are initiated before SD can be initiated (increments are not avaliable at navigation_wrapper reset,
    the actual values that were used are given rahter than generated once again.
    """
    if random_vector is None:
        SF = np.random.randn(9, )
    else:
        SF = random_vector

    measured_nav_state.pos += IC_params[0, :] * SF[0:3]
    erros_att_x_y_z = IC_params[1, :] * SF[3:6]  # x,y,z
    erros_att_z_y_x = np.flip(erros_att_x_y_z)  # z,y,x
    measured_nav_state.att = np.asarray(
        nav.dcm2angle(nav.angle2dcm(measured_nav_state.att[0], measured_nav_state.att[1], measured_nav_state.att[2]) @ \
                      nav.angle2dcm(erros_att_z_y_x[0], erros_att_z_y_x[1], erros_att_z_y_x[2])))

    measured_nav_state.vel += IC_params[2, :] * SF[6:9]
    a = 1

    return measured_nav_state