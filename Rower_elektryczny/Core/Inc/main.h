/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "stm32h7xx_hal.h"

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

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LCD_RESET_Pin GPIO_PIN_2
#define LCD_RESET_GPIO_Port GPIOE
#define GT911_INT_Pin GPIO_PIN_3
#define GT911_INT_GPIO_Port GPIOE
#define LCD_BL_Pin GPIO_PIN_6
#define LCD_BL_GPIO_Port GPIOH
#define GT911_RST_Pin GPIO_PIN_7
#define GT911_RST_GPIO_Port GPIOH

/* USER CODE BEGIN Private defines */
typedef struct {
    volatile int32_t rpm;
    volatile float current_motor;
    volatile float duty_cycle;
    volatile float amp_hours;
    volatile float amp_hours_chg;
    volatile float watt_hours;
    volatile float watt_hours_chg;
    volatile float temp_fet;
    volatile float temp_motor;
    volatile float current_in;
    volatile float pid_pos;
    volatile int32_t tacho_value;
    volatile float v_in;
} VescData_t;

// 2. Udostępnienie zmiennych dla C++
extern VescData_t vescL;
extern VescData_t vescM;
extern VescData_t vescR;
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
