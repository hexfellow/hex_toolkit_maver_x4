#!/usr/bin/env python3
# -*- coding:utf-8 -*-
################################################################
# Copyright 2024 Dong Zhaorui. All rights reserved.
# Author: Dong Zhaorui 847235539@qq.com
# Date  : 2024-12-23
################################################################

import copy
import numpy as np

import hex_utils
from hex_utils import HexCartState


class ObsUtil:

    def __init__(
        self,
        rate_param: dict,
        limit_param: dict,
        obs_param: dict,
    ):
        ### parameters
        self.__rate_param = copy.deepcopy(rate_param)
        self.__limit_param = copy.deepcopy(limit_param)
        self.__obs_param = copy.deepcopy(obs_param)

        ### variables
        self.__vel_limit = self.__limit_param["vel"]
        self.__acc_limit = self.__limit_param["acc"]
        self.__dt = 1.0 / rate_param["ros"]
        self.__ready = False
        self.__obs_state = HexCartState()

    def is_ready(self) -> bool:
        return self.__ready

    def set_state(self, state: HexCartState):
        self.__obs_state = state
        self.__ready = True

    def update(self, state: HexCartState):
        obs_pos = self.__obs_state.pose().get_pos()
        obs_quat = self.__obs_state.pose().get_quat()
        obs_vel = self.__obs_state.vel().linear()
        obs_omega = self.__obs_state.vel().angular()

        cur_pos = state.pose().get_pos()
        cur_quat = state.pose().get_quat()
        cur_vel = state.vel().linear()
        cur_omega = state.vel().angular()

        obs_weight = self.__obs_param["weights"]
        cur_weight = 1.0 - obs_weight
        new_pos = obs_weight * obs_pos + cur_weight * cur_pos
        new_quat = hex_utils.quat_slerp(obs_quat, cur_quat, cur_weight)
        new_vel = obs_weight * obs_vel + cur_weight * cur_vel
        new_omega = obs_weight * obs_omega + cur_weight * cur_omega

        self.__obs_state.pose().set_pos(new_pos)
        self.__obs_state.pose().set_quat(new_quat)
        self.__obs_state.vel().set_linear(new_vel)
        self.__obs_state.vel().set_angular(new_omega)

    def get_state(self) -> HexCartState:
        return self.__obs_state
    
    def limit_norm(self, vec, max_norm):
        norm = np.linalg.norm(vec)
        if norm > max_norm:
            return vec / (norm + 1e-8) * max_norm
        return vec

    def predict(
        self,
        acc: np.ndarray,
        alpha: np.ndarray,
    ):
        # curr state
        trans_body_in_odom = self.__obs_state.pose().get_trans()
        pos_in_odom = self.__obs_state.pose().get_pos()
        quat_body_in_odom = self.__obs_state.pose().get_quat()
        cur_lin_in_body = self.__obs_state.vel().linear()
        cur_ang_in_body = self.__obs_state.vel().angular()

        acc_max = abs(self.__acc_limit[0])
        alpha_max = abs(self.__acc_limit[1])
        limited_acc = self.limit_norm(acc, acc_max)
        limited_alpha = self.limit_norm(alpha, alpha_max)
        tar_lin_in_body = cur_lin_in_body + limited_acc * self.__dt
        tar_ang_in_body = cur_ang_in_body + limited_alpha * self.__dt

        vel_max = abs(self.__vel_limit[0])
        ang_max = abs(self.__vel_limit[1])
        tar_lin_in_body = self.limit_norm(tar_lin_in_body, vel_max)
        tar_ang_in_body = self.limit_norm(tar_ang_in_body, ang_max)

        # pose
        rot_body_in_odom = trans_body_in_odom[:3, :3]
        avg_lin_in_body = 0.5 * (cur_lin_in_body + tar_lin_in_body)
        avg_ang_in_body = 0.5 * (cur_ang_in_body + tar_ang_in_body)
        avg_lin_in_odom = rot_body_in_odom @ avg_lin_in_body
        new_pos_in_odom = pos_in_odom + avg_lin_in_odom * self.__dt
        so3_new_in_body = avg_ang_in_body * self.__dt
        if np.linalg.norm(so3_new_in_body) < 1e-6:
            quat_new_in_odom = copy.deepcopy(quat_body_in_odom)
        else:
            quat_new_in_body = hex_utils.so32quat(so3_new_in_body)
            quat_new_in_odom = hex_utils.quat_mul(
                quat_new_in_body,
                quat_body_in_odom,
            )

        # update state
        self.__obs_state.pose().set_pos(new_pos_in_odom)
        self.__obs_state.pose().set_quat(quat_new_in_odom)
        self.__obs_state.vel().set_linear(tar_lin_in_body)
        self.__obs_state.vel().set_angular(tar_ang_in_body)
