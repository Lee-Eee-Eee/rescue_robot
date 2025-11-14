/*
 * Copyright (c) 2006-2021, XRobot Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2023-04-25     zhb          first version
 */

#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>

#if defined(BSP_USING_BEEP)

#define BEEP_PWM_DEV_NAME       "pwm1"
#define BEEP_PWM_DEV_CHANNEL    4

struct rt_device_pwm *pwm_device = RT_NULL;

// 使能蜂鸣器对应的 PWM 通道
void beep_on(void)
{
    rt_pwm_enable(pwm_device, BEEP_PWM_DEV_CHANNEL); 
}

// 失能蜂鸣器对应的 PWM 通道
void beep_off(void)
{
    rt_pwm_disable(pwm_device, BEEP_PWM_DEV_CHANNEL);
}

void beep_set(uint16_t freq, uint8_t volume)
{
    rt_uint32_t period, pulse;

    /* 将频率转化为周期 周期单位:ns 频率单位:HZ */
    period = 1000000000 / freq;  //unit:ns 1/HZ*10^9 = ns

    /* 根据声音大小计算占空比 蜂鸣器低电平触发 */
    pulse = period - period / 100 * volume;

    /* 利用 PWM API 设定 周期和占空比 */
    rt_pwm_set(pwm_device, BEEP_PWM_DEV_CHANNEL, period, pulse); // channel, period, pulse
}

int beep_port_init(void)
{
    /* 查找PWM设备 */
    pwm_device = (struct rt_device_pwm *)rt_device_find(BEEP_PWM_DEV_NAME);
    if (pwm_device == RT_NULL)
    {
        return -RT_ERROR;
    }
    return 0;
}
INIT_COMPONENT_EXPORT(beep_port_init);

#endif

