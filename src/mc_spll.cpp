// SPDX-License-Identifier: LGPL-2.1

/**
 * @file    mc_spll.cpp
 * @brief   数字锁相环
 * @author  向阳 (hinata.hoshino@foxmail.com)
 * @date    2022-07-27
 * 
 * @copyright Copyright (C) 向阳 2022
 */

#include "mc_spll.h"

extern "C" {
    #include "arm_math.h"
}

using namespace control;

SPLL::SPLL()
{
    pid.param.kp = 750;
    pid.param.ki = 15;
    pid.param.kd = 0;
    pid.param.kc = 10;
    // 输出限幅
    pid.param.i_min = -(TARGET_FREQ + 15) * constant_value<value_t>::TAU;
    pid.param.i_max = +(TARGET_FREQ + 15) * constant_value<value_t>::TAU;
    // 复位程序
    reset();
}

void SPLL::reset()
{
    cur_phase = 0;
    last_error = 0;
    launch_loop = false;
    sample_index = 0;
    sogi_s1 = 0;
    sogi_s2 = 0;
    auto_offset_min = constant_value<value_t>::MAX;
    auto_offset_max = constant_value<value_t>::MIN;
    pid.reset();
}

void SPLL::transfer_1phase(value_t val)
{
    value_t v_org = auto_offset(val);
    //
    if (sample_index < N_SAMPLE)
    {
        sample_index += 1;
        launch_loop = false;
        return;
    }
    else {
        launch_loop = true;
    }
    //
    // 缩放
    //
    value_t v = v_org / (auto_offset_max - auto_offset_min);
    //
    // SOGI
    //
    // SOGI: k
    constexpr value_t k = 1.414;
    // SOGI: w
    constexpr value_t w = TARGET_FREQ * constant_value<value_t>::TAU;
    // 2pi -> 360°的系数
    constexpr value_t angle_k = 360 / constant_value<value_t>::TAU;
    //
    value_t sogi_u = (k * (v - sogi_s1) - sogi_s2) * w;
    sogi_s1 += Ti * sogi_u;
    sogi_s2 += Ti * w * sogi_s1;
    //
    // VCO(park)
    //
    value_t ua = sogi_s1;
    value_t ub = sogi_s2;
    value_t theta = angle_k * cur_phase;
    value_t st, ct;
    arm_sin_cos_f32(theta, &st, &ct);
    value_t uq = ct * ub - st * ua;
    //
    // LPF (PI)
    //
    value_t e = 0 - uq;
    value_t u = pid.pi_transfer(e);
    //
    // 鉴相器
    //
    value_t i = cur_phase;
    i += Ti * u;
    if (i > constant_value<value_t>::TAU)
    {
        i -= constant_value<value_t>::TAU;
    }
    else if (i < -constant_value<value_t>::TAU)
    {
        i += constant_value<value_t>::TAU;
    }
    //
    // 输出
    //
    omega = u;
    cur_phase = i;
    last_error = e;
    // -- loop --
}

bool SPLL::is_lock(value_t th) const
{
    return launch_loop && last_error < th;
}

value_t SPLL::auto_offset(value_t inp)
{
    if (inp > auto_offset_max)
    {
        auto_offset_max = inp;
    }
    else if (inp < auto_offset_min)
    {
        auto_offset_min = inp;
    }
    value_t mid = (auto_offset_min + auto_offset_max) * 0.5f;
    return inp - mid;
}
