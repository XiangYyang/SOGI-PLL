// SPDX-License-Identifier: LGPL-2.1

/**
 * @file    mc_pid.cpp
 * @brief   数字PID控制器
 * @author  向阳 (hinata.hoshino@foxmail.com)
 * @date    2022-07-16
 * 
 * @copyright Copyright (C) 向阳 2022
 */
#include "mc_pid.h"

using namespace control;

void PID::reset() noexcept
{
    i_sum = 0;
    sat_err = 0;
}

value_t PID::p_transfer(value_t e) noexcept
{
    value_t m = param.kp * e;
    value_t out
        = m > param.i_max ? param.i_max
        : m < param.i_min ? param.i_min
        : m;
    return out;
}

value_t PID::pi_transfer(value_t e) noexcept
{
    value_t sat = param.kp * e + i_sum;
    // PI输出
    // U(s) = kp * E(s) + ki * E(s) / s
    value_t out
        = sat > param.i_max ? param.i_max
        : sat < param.i_min ? param.i_min
        : sat;
    // 饱和误差
    value_t sat_err = out - sat;
    // 累积积分, 加上饱和误差
    i_sum += param.ki * e + param.kc * sat_err;
    // 积分限幅, clamp
    if (i_sum > param.i_max)
    {
        i_sum = param.i_max;
    }
    else if (i_sum < param.i_min)
    {
        i_sum = param.i_min;
    }
    //
    return out;
}

value_t PID::pd_transfer(value_t e) noexcept
{
    return 0;
}

value_t PID::pid_transfer(value_t e) noexcept
{
    return 0;
}
