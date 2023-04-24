// SPDX-License-Identifier: LGPL-2.1

/**
 * @file    ma_pid.h
 * @brief   数字PID控制器
 * @author  向阳 (hinata.hoshino@foxmail.com)
 * @date    2022-07-16
 *
 * @copyright Copyright (C) 向阳 2022
 */
#if !defined(__INCLUDE_CONTROL_MC_PID_H__)
#define __INCLUDE_CONTROL_MC_PID_H__
#include "mc_config.h"
namespace control
{
    /**
     * @brief 带退饱和反馈和积分限幅的PID控制器(float32)
     *
     */
    class PID final
    {
    public:
        /**
         * @brief 控制器参数
         *
         */
        struct Factors
        {
            //! 比例系数
            value_t kp;

            //! 积分系数
            value_t ki;

            //! 微分系数
            value_t kd;

            //! 微分增益
            value_t kg;

            //! 饱和系数
            value_t kc;

            //! 积分器限幅(min)
            value_t i_min;

            //! 积分器限幅(max)
            value_t i_max;
        };

        PID() = default;
        ~PID() = default;

        PID(const PID &) = delete;
        PID &operator=(const PID &) = delete;

        PID(PID &&) = delete;
        PID &operator=(PID &&) = delete;

        //! @get
        //! @set
        //! 控制器参数
        Factors param = {0};

        /**
         * @brief 重置调节器状态
         *
         */
        void reset() noexcept;

        /**
         * @brief 数字P
         *
         * @param [in] e: 输入, error
         *
         * @return value_t: 输出
         *
         */
        value_t p_transfer(value_t e) noexcept;

        /**
         * @brief 数字PI
         *
         * @param [in] e: 输入, error
         *
         * @return value_t: 输出
         *
         */
        value_t pi_transfer(value_t e) noexcept;

        /**
         * @brief 数字PD
         *
         * @param [in] e: 输入, error
         *
         * @return value_t: 输出
         *
         */
        value_t pd_transfer(value_t e) noexcept;

        /**
         * @brief 数字PID
         *
         * @param [in] e: 输入, error
         *
         * @return value_t: 输出
         *
         */
        value_t pid_transfer(value_t e) noexcept;

    private:
        //! 饱和误差
        value_t sat_err = 0;

        //! 积分值
        value_t i_sum = 0;
    };
}
#endif // __INCLUDE_CONTROL_MC_PID_H__
