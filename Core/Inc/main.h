/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
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
#include "stm32f0xx_hal.h"

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
#define XTAL_2_Pin GPIO_PIN_0
#define XTAL_2_GPIO_Port GPIOF
#define XTAL_1_Pin GPIO_PIN_1
#define XTAL_1_GPIO_Port GPIOF
#define Unknown_12_Pin GPIO_PIN_0
#define Unknown_12_GPIO_Port GPIOA
#define Unknown_13_Pin GPIO_PIN_1
#define Unknown_13_GPIO_Port GPIOA
#define Unknown_14_Pin GPIO_PIN_2
#define Unknown_14_GPIO_Port GPIOA
#define Unknown_1_Pin GPIO_PIN_3
#define Unknown_1_GPIO_Port GPIOA
#define ESC_VCC_Pin GPIO_PIN_4
#define ESC_VCC_GPIO_Port GPIOA
#define Not_Connected_1_Pin GPIO_PIN_5
#define Not_Connected_1_GPIO_Port GPIOA
#define LED_Red_Pin GPIO_PIN_6
#define LED_Red_GPIO_Port GPIOA
#define LED_Bue_Pin GPIO_PIN_7
#define LED_Bue_GPIO_Port GPIOA
#define Tail_Motor_Pin GPIO_PIN_0
#define Tail_Motor_GPIO_Port GPIOB
#define Unused_Plug_Pin GPIO_PIN_1
#define Unused_Plug_GPIO_Port GPIOB
#define Not_Connected_2_Pin GPIO_PIN_2
#define Not_Connected_2_GPIO_Port GPIOB
#define RF_CSn_Pin GPIO_PIN_8
#define RF_CSn_GPIO_Port GPIOA
#define Test_Pad_Pin GPIO_PIN_9
#define Test_Pad_GPIO_Port GPIOA
#define Not_Connected_3_Pin GPIO_PIN_10
#define Not_Connected_3_GPIO_Port GPIOA
#define Not_Connected_4_Pin GPIO_PIN_11
#define Not_Connected_4_GPIO_Port GPIOA
#define RF_GDO0_Pin GPIO_PIN_12
#define RF_GDO0_GPIO_Port GPIOA
#define Debug_Data_Pin GPIO_PIN_13
#define Debug_Data_GPIO_Port GPIOA
#define Debug_Clock_Pin GPIO_PIN_14
#define Debug_Clock_GPIO_Port GPIOA
#define RF_GDO2_Pin GPIO_PIN_15
#define RF_GDO2_GPIO_Port GPIOA
#define RF_SO_GDO1_Pin GPIO_PIN_3
#define RF_SO_GDO1_GPIO_Port GPIOB
#define RF_SCLK_Pin GPIO_PIN_4
#define RF_SCLK_GPIO_Port GPIOB
#define RF_SI_Pin GPIO_PIN_5
#define RF_SI_GPIO_Port GPIOB
#define Gyro_Clock_Pin GPIO_PIN_6
#define Gyro_Clock_GPIO_Port GPIOB
#define Gyro_Data_Pin GPIO_PIN_7
#define Gyro_Data_GPIO_Port GPIOB
#define Button_Pin GPIO_PIN_8
#define Button_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
