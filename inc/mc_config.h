// SPDX-License-Identifier: LGPL-2.1

/**
 * @file    mc_config.h
 * @brief   控制系统配置
 * @author  向阳 (hinata.hoshino@foxmail.com)
 * @date    2022-07-27
 * 
 * @copyright Copyright (C) 向阳 2022
 */
#if !defined(__INCLUDE_CONTROL_MC_CONFIG_H__)
#define __INCLUDE_CONTROL_MC_CONFIG_H__
#include <float.h>
namespace control
{
    //! 运算的值类型
    using value_t = float;

    //! 控制周期
    constexpr value_t T = 1 / 12800.0f;

    //! 常量值表
    template<typename T>
    struct constant_value;

    
    template<>
    struct constant_value<value_t>
    {
        //! 零
        static constexpr value_t ZERO = 0;

        //! 圆周率
        static constexpr value_t PI = 3.14159265358979323f;

        //! 1/2 * 圆周率
        static constexpr value_t HALF_PI = PI / 2;
    
        //! 2 * 圆周率
        static constexpr value_t TAU = PI * 2;

        //! 最大值
        static constexpr value_t MAX = FLT_MAX;
    
        //! 最小
        static constexpr value_t MIN = FLT_MIN;
    };

}
#endif // __INCLUDE_CONTROL_MC_SPLL_H__
