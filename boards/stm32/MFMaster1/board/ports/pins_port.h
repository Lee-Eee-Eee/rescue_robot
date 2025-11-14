#ifndef _PINS_PORT_H_
#define _PINS_PORT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include "drv_gpio.h"

/* 按键引脚 */
#define KEY1_PIN        GET_PIN(E, 8)
#define KEY2_PIN        GET_PIN(E, 10)
#define KEY3_PIN        GET_PIN(E, 12)
#define KEY4_PIN        GET_PIN(E, 7)
#define KEY5_PIN        GET_PIN(E, 15)

/* LED引脚 */
#define LEDR_PIN        GET_PIN(E, 9)
#define LEDG_PIN        GET_PIN(E, 13)
#define LEDB_PIN        GET_PIN(E, 11)

/* 蜂鸣器引脚 */
#define BEEP_PIN        GET_PIN(E, 14)

/* SPI片选引脚 */
#define FLASH_CS_PIN    GET_PIN(C, 13)
#define BMI088A_CS_PIN  GET_PIN(C, 0)
#define BMI088G_CS_PIN  GET_PIN(C, 5)

/* USB电源控制引脚 */
#define USB_PWR_PIN     GET_PIN(E, 6)

/* LCD引脚 */
#define LCD_CS_PIN      GET_PIN(A, 8)
#define LCD_DC_PIN      GET_PIN(E, 2)
#define LCD_RESET_PIN   GET_PIN(E, 3)

/* RS485使能引脚 */
#define RS485_EN_PIN    GET_PIN(C, 8)

/* 外设引脚 */
#define SG1_PWR_PIN     GET_PIN(D, 0)
#define SG1_IO_PIN      GET_PIN(C, 12)
#define SG2_PWR_PIN     GET_PIN(C, 11)
#define SG2_IO_PIN      GET_PIN(C, 10)
#define SG3_PWR_PIN     GET_PIN(D, 5)
#define SG3_IO_PIN      GET_PIN(D, 6)
#define SG4_PWR_PIN     GET_PIN(D, 7)
#define SG4_IO_PIN      GET_PIN(B, 3)
#define SG5_PWR_PIN     GET_PIN(D, 1)
#define SG5_IO_PIN      GET_PIN(D, 2)
#define SG6_PWR_PIN     GET_PIN(D, 3)
#define SG6_IO_PIN      GET_PIN(D, 4)

/* 输入引脚 */
#define IN1_PIN         GET_PIN(D, 11)
#define IN2_PIN         GET_PIN(D, 12)
#define IN3_PIN         GET_PIN(D, 13)
#define IN4_PIN         GET_PIN(D, 14)

#ifdef __cplusplus
}
#endif

#endif /* _PINS_PORT_H_ */