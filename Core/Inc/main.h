/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "stm32u5xx_hal.h"

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
#define button_user_Pin GPIO_PIN_13
#define button_user_GPIO_Port GPIOC
#define LCD_SCK_Pin GPIO_PIN_5
#define LCD_SCK_GPIO_Port GPIOA
#define LCD_SDI_Pin GPIO_PIN_7
#define LCD_SDI_GPIO_Port GPIOA
#define spi1_sd_cs_Pin GPIO_PIN_12
#define spi1_sd_cs_GPIO_Port GPIOF
#define spi1_tft_cs_Pin GPIO_PIN_14
#define spi1_tft_cs_GPIO_Port GPIOD
#define tft_bl_pwm_Pin GPIO_PIN_15
#define tft_bl_pwm_GPIO_Port GPIOD
#define led_red_Pin GPIO_PIN_2
#define led_red_GPIO_Port GPIOG
#define led_grn_Pin GPIO_PIN_7
#define led_grn_GPIO_Port GPIOC
#define led_blu_Pin GPIO_PIN_7
#define led_blu_GPIO_Port GPIOB
#define tft_rst_Pin GPIO_PIN_8
#define tft_rst_GPIO_Port GPIOB
#define tft_dc_sel_out_Pin GPIO_PIN_9
#define tft_dc_sel_out_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
