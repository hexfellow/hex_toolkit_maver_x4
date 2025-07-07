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
        p_err = np.clip(p_err, self.__err_limit[:, 0], self.__err_limit[:, 1])
        v_err = v_ref - v_cur
        acc = self.__kp * p_err + self.__kd * v_err
        return acc
