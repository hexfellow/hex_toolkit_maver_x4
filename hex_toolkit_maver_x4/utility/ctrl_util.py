#!/usr/bin/env python3
# -*- coding:utf-8 -*-
################################################################
# Copyright 2024 Dong Zhaorui. All rights reserved.
# Author: Dong Zhaorui 847235539@qq.com
# Date  : 2024-12-23
################################################################

import numpy as np

import hex_utils


class PdCtrl:

    def __init__(
        self,
        trace_param: dict,
    ):
        self.__kp = trace_param["pid"][0]
        self.__kd = trace_param["pid"][2]
        self.__dt = trace_param["dt"]
        self.__err_limit = trace_param["err_limit"]

    def __call__(self, p_cur, p_ref, v_cur, v_ref=np.zeros(1)):
        p_err = p_ref - p_cur

        # pos error
        pos_err_norm = np.linalg.norm(p_err[:2])
        pos_err_dir = p_err[:2] / (pos_err_norm + 1e-6)
        pos_err_norm = np.clip(pos_err_norm, -self.__err_limit[0],
                               self.__err_limit[0])
        p_err[:2] = pos_err_dir * pos_err_norm

        # yaw error
        p_err[2] = hex_utils.angle_norm(p_err[2])
        p_err[2] = np.clip(p_err[2], -self.__err_limit[1], self.__err_limit[1])

        # vel error
        v_err = v_ref - v_cur
        acc = self.__kp * p_err + self.__kd * v_err
        return acc
