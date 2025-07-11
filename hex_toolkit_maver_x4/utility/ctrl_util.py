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
        p_err[2] = hex_utils.angle_norm(p_err[2])
        xy = p_err[:2]
        xy_max = abs(self.__err_limit[0])
        xy_norm = np.linalg.norm(xy)
        xy = xy * min(1.0, xy_max / (xy_norm + 1e-8))
        p_err[:2] = xy
        yaw_max = abs(self.__err_limit[1])
        p_err[2] = np.clip(p_err[2], -yaw_max, yaw_max)
        v_err = v_ref - v_cur
        acc = self.__kp * p_err + self.__kd * v_err
        return acc
