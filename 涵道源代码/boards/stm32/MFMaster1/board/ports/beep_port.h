#ifndef _BEEP_PORT_H_
#define _BEEP_PORT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdint.h>

// int beep_init(void);                         //蜂鸣器初始化
void beep_on(void);                             //蜂鸣器开
void beep_off(void);                            //蜂鸣器关
void beep_set(uint16_t freq, uint8_t volume);   //蜂鸣器设定

#ifdef __cplusplus
}
#endif

#endif /* _BEEP_PORT_H_ */
