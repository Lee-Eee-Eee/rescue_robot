/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LCD_DC_Pin GPIO_PIN_2
#define LCD_DC_GPIO_Port GPIOE
#define LCD_RST_Pin GPIO_PIN_3
#define LCD_RST_GPIO_Port GPIOE
#define USB_PWR_Pin GPIO_PIN_6
#define USB_PWR_GPIO_Port GPIOE
#define FLASH_CS_Pin GPIO_PIN_13
#define FLASH_CS_GPIO_Port GPIOC
#define ACCEL_CS_Pin GPIO_PIN_0
#define ACCEL_CS_GPIO_Port GPIOC
#define ACCEL_INT_Pin GPIO_PIN_1
#define ACCEL_INT_GPIO_Port GPIOC
#define GYRO_INT_Pin GPIO_PIN_4
#define GYRO_INT_GPIO_Port GPIOC
#define GYRO_CS_Pin GPIO_PIN_5
#define GYRO_CS_GPIO_Port GPIOC
#define KEY4_Pin GPIO_PIN_7
#define KEY4_GPIO_Port GPIOE
#define KEY1_Pin GPIO_PIN_8
#define KEY1_GPIO_Port GPIOE
#define LED_R_Pin GPIO_PIN_9
#define LED_R_GPIO_Port GPIOE
#define KEY2_Pin GPIO_PIN_10
#define KEY2_GPIO_Port GPIOE
#define LED_G_Pin GPIO_PIN_11
#define LED_G_GPIO_Port GPIOE
#define KEY3_Pin GPIO_PIN_12
#define KEY3_GPIO_Port GPIOE
#define LED_B_Pin GPIO_PIN_13
#define LED_B_GPIO_Port GPIOE
#define BEEP_Pin GPIO_PIN_14
#define BEEP_GPIO_Port GPIOE
#define KEY5_Pin GPIO_PIN_15
#define KEY5_GPIO_Port GPIOE
#define RS485_EN_Pin GPIO_PIN_8
#define RS485_EN_GPIO_Port GPIOC
#define LCD_CS_Pin GPIO_PIN_8
#define LCD_CS_GPIO_Port GPIOA
#define GYRO_TEMP_Pin GPIO_PIN_15
#define GYRO_TEMP_GPIO_Port GPIOA
#define SG2_IO_Pin GPIO_PIN_10
#define SG2_IO_GPIO_Port GPIOC
#define SG2_PWR_Pin GPIO_PIN_11
#define SG2_PWR_GPIO_Port GPIOC
#define SG1_IO_Pin GPIO_PIN_12
#define SG1_IO_GPIO_Port GPIOC
#define SG1_PWR_Pin GPIO_PIN_0
#define SG1_PWR_GPIO_Port GPIOD
#define SG5_PWR_Pin GPIO_PIN_1
#define SG5_PWR_GPIO_Port GPIOD
#define SG5_IO_Pin GPIO_PIN_2
#define SG5_IO_GPIO_Port GPIOD
#define SG6_PWR_Pin GPIO_PIN_3
#define SG6_PWR_GPIO_Port GPIOD
#define SG6_IO_Pin GPIO_PIN_4
#define SG6_IO_GPIO_Port GPIOD
#define SG3_PWR_Pin GPIO_PIN_5
#define SG3_PWR_GPIO_Port GPIOD
#define SG3_IO_Pin GPIO_PIN_6
#define SG3_IO_GPIO_Port GPIOD
#define SG4_PWR_Pin GPIO_PIN_7
#define SG4_PWR_GPIO_Port GPIOD
#define SG4_IO_Pin GPIO_PIN_3
#define SG4_IO_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
