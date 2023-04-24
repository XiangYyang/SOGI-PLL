// SPDX-License-Identifier: LGPL-2.1

/**
 * @file    mc_spll.h
 * @brief   数字锁相环
 * @author  向阳 (hinata.hoshino@foxmail.com)
 * @date    2022-07-27
 * 
 * @copyright Copyright (C) 向阳 2022
 */
#if !defined(__INCLUDE_CONTROL_MC_SPLL_H__)
#define __INCLUDE_CONTROL_MC_SPLL_H__
#include <tuple>
#include "mc_pid.h"
#include <stddef.h>
#include <stdint.h>
#include "mc_config.h"
namespace control
{
    class SPLL final
    {
    public:
        //! SPLL工作周期
        static constexpr value_t Ti = T;

        //! SPLL期望频率(Hz)
        static constexpr value_t TARGET_FREQ = 50;

        //! 计算1个周期有多少个点
        static constexpr size_t N_SAMPLE = static_cast<size_t>((1 / Ti) / TARGET_FREQ);

        SPLL();
        ~SPLL() = default;

        SPLL(SPLL&&) = delete;
        SPLL& operator=(SPLL&&) = delete;

        SPLL(const SPLL &) = delete;
        SPLL& operator=(const SPLL &) = delete;

        /**
         * @brief 复位
         * 
         */
        void reset();

        /**
         * @brief 进行一次计算
         * 
         * @note  按照Ti等间隔调用本方法, 实现PLL
         * 
         */
        void transfer_1phase(value_t val);

        /**
         * @brief 判断PLL是否已经锁定
         *
         * @param [in] th: 比较阈值
         * 
         */
        bool is_lock(value_t th = 1e-2f) const;

        /**
         * @brief 取得当前信号的频率
         *
         * @return value_t: 频率
         * 
         */
        inline value_t freq() const noexcept
        {
            return omega / constant_value<value_t>::TAU;
        }

        /**
         * @brief 取得当前信号的相位
         *
         * @return value_t: 相位
         * 
         */
        inline value_t phase() const noexcept
        {
            return cur_phase;
        }
    private:
        //! PI
        PID pid;

        //! 是否启动环路
        bool launch_loop;

        //! 采样点索引, 最大到N_SAMPLE / 4
        uint16_t sample_index;

        //! 角速度
        value_t omega;

        //! 角度积分值, 作为相位wt
        value_t cur_phase;

        //! 自动归零: 最大值和最小值
        value_t auto_offset_min;
        value_t auto_offset_max;

        //! SOGI的积分值
        value_t sogi_s1;
        value_t sogi_s2;


        //! 上次的环路error
        value_t last_error;
    
        /**
         * @brief 按照历史最大值和最小值进行归中
         * @param [in] inp: 输入
         *
         * @return value_t: 归中后的值
         * 
         */
        value_t auto_offset(value_t inp);
    };
}
#endif // __INCLUDE_CONTROL_MC_SPLL_H__
