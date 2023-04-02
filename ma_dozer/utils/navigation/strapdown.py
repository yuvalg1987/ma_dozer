import numpy as np
import navpy as nav

from ma_dozer.configs.navigation_config import NavigationConfig
from ma_dozer.utils.helpers.classes import NavState
from ma_dozer.utils.navigation.utils import copy_nav_state, add_IC_errors


class StrapDown:
    def __init__(self, config: NavigationConfig, pos: np.ndarray, vel: np.ndarray, att: np.ndarray, time: np.float,
                 g: float = 9.8, dt_nav: float = 0.01, dt_kal: float = 1., add_random: bool = False,
                 ic_random_vector_in: np.ndarray = None):
        self.true_state = NavState(time = time, pos = pos.copy(), vel = vel.copy(), att = att.copy())
        self.true_state_prev = copy_nav_state(self.true_state)

        self.measured_state = copy_nav_state(self.true_state)
        if config.add_IC_errors:
            self.measured_state = add_IC_errors(IC_params=config.IC_params,
                                                measured_nav_state=self.measured_state, random_vector=ic_random_vector_in)
        else:
            pass
        self.measured_state_prev = copy_nav_state(self.measured_state)
        self.g = g
        self.dt_nav = dt_nav
        self.dt_kal = dt_kal
        self.sum_dcm = np.zeros((3,3))
        self.estimated_acc_bias = np.zeros(3, ) # x,y,z
        self.estimated_gyro_drift = np.zeros(3, ) # x,y,z

        self.add_random = add_random

    def SD(self, Qt, Qv, time, dt):

        # save current time
        self.measured_state.time = time

        # Calc attitude
        self.measured_state.att = self.SD_Attitude(Qt)

        # ---Calc velocity NED---
        self.measured_state.vel = self.SD_Velocity(Qv, dt)

        # ---Calc position NED---
        self.measured_state.pos = self.SD_Position(dt)

        # sum dcm for kalman - the DCM from Body to NED
        self.sum_dcm += self.dt_nav * nav.angle2dcm(self.measured_state.att[0], self.measured_state.att[1], self.measured_state.att[2]).T

        # for next iteration
        self.measured_state_prev = copy_nav_state(self.measured_state)

    def SD_Velocity(self, Qv, dt):
        prev_DCM = nav.angle2dcm(self.measured_state_prev.att[0], self.measured_state_prev.att[1],
                                 self.measured_state_prev.att[2])
        curr_DCM = nav.angle2dcm(self.measured_state.att[0], self.measured_state.att[1],
                                 self.measured_state.att[2])
        # dcm_avg = (prev_DCM + curr_DCM )/ 2

        dcm_avg = curr_DCM
        vel_l = self.measured_state_prev.vel + dcm_avg.T @ Qv + np.array([0,0,1]) * self.g * dt
        return vel_l

    def SD_Position(self, dt):
        Pos_ned = np.zeros(3,)
        # Pos_ned[0] = self.measured_state_prev.pos[0] + (self.measured_state_prev.vel[0] + self.measured_state.vel[0]) / 2  * dt
        # Pos_ned[1] = self.measured_state_prev.pos[1] + (self.measured_state_prev.vel[1] + self.measured_state.vel[1]) / 2  * dt
        # Pos_ned[2] = self.measured_state_prev.pos[2] - (self.measured_state_prev.vel[2] + self.measured_state.vel[2]) / 2 * dt

        Pos_ned[0] = self.measured_state_prev.pos[0] + self.measured_state_prev.vel[0]  * dt
        Pos_ned[1] = self.measured_state_prev.pos[1] + self.measured_state_prev.vel[1]  * dt
        # Pos_ned[2] = self.measured_state_prev.pos[2] - self.measured_state_prev.vel[2]  * dt
        Pos_ned[2] = self.measured_state_prev.pos[2] + self.measured_state_prev.vel[2] * dt

        return Pos_ned

    def SD_Attitude(self, Qt):
        att_prev = self.measured_state_prev.att
        Qt_z_y_x = np.flip(Qt)
        att = nav.dcm2angle(nav.angle2dcm(Qt_z_y_x[0], Qt_z_y_x[1], Qt_z_y_x[2]) @ (nav.angle2dcm(att_prev[0], att_prev[1], att_prev[2])))
        att = np.asarray(att)
        return att

    def true_state_update(self, pos, vel, att, time):
        self.true_state_prev = copy_nav_state(self.true_state)
        self.true_state = NavState(time = time, pos = pos, vel = vel, att = att)
