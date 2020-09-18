/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdbool.h>
#include "mpu6050.h"
#include "pid_control.h"
#include "SEGGER_RTT.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_SPI1_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */

  // Gyro Interface
  MPU6050_t mpu_handle;
  if (MPU6050_Init(&mpu_handle, &hi2c1, MPU6050_Device_0, MPU6050_Accelerometer_2G, MPU6050_Gyroscope_250s) != MPU6050_Result_Ok)
  {
    Error_Handler();
  }

#if 0
  if (MPU6050_DMP_LoadFirmware(&mpu_handle) != MPU6050_Result_Ok)
  {
    Error_Handler();
  }

  if (MPU6050_DMP_SetState(&mpu_handle, true) != MPU6050_Result_Ok)
  {
    Error_Handler();
  }
#endif

  float gyro_integral = 0.0f;
  uint32_t gyro_tick = 0;

  // Init Gyro Status LED
  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);

  // Tail Control
  const PID_Terms tail_control = { 1.0f / 33.0f, 0.0f, 0.0f };
  PID_History tail_control_history = { 0.0f, 0.0f };
  uint32_t tail_tick = 0;

  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
  uint32_t tail_max_output = 1000;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

//#define TEST_ALL 1
#define TEST_LOG 1

#if TEST_ALL
  while (1)
  {
    // Can be used for debug or maybe to cutoff tail motor ?
    const bool button_pressed = (HAL_GPIO_ReadPin(Button_GPIO_Port, Button_Pin) == GPIO_PIN_RESET);

    if (MPU6050_ReadGyroscope(&mpu_handle) != MPU6050_Result_Ok)
    {
      Error_Handler();
    }
    else
    {
      uint32_t previous_tick = gyro_tick;
      gyro_tick = HAL_GetTick();
      uint32_t delta_tick = gyro_tick - previous_tick;

      gyro_integral -= mpu_handle.Gyroscope_Z * delta_tick;
    }

    // Gyroscope outputs from -32768 to 32767
    // We discard negative values, and divide the output by 33 to get an approximate output from 0 to 1000
    // (in reality 0 to 993 because of rounding)
    uint16_t gyro_led_output = mpu_handle.Gyroscope_Z < 0 ? 0 : (mpu_handle.Gyroscope_Z / 33);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, gyro_led_output);

    // Update PID for tail
    float tail_setpoint = 0;
    float tail_processvariable = gyro_integral;

    uint32_t previous_tick = tail_tick;
    tail_tick = HAL_GetTick();
    uint32_t delta_tick = tail_tick - previous_tick;

    float tail_output = PID_Compute(&tail_control, &tail_control_history, delta_tick, tail_setpoint, tail_processvariable);
    //__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, tail_output > 0.0 ? (uint32_t)tail_output : 0);
  }
#endif

#if TEST_LOG
  char r;

  SEGGER_RTT_WriteString(0, "SEGGER Real-Time-Terminal Sample\r\n");
  SEGGER_RTT_ConfigUpBuffer(0, NULL, NULL, 0, SEGGER_RTT_MODE_NO_BLOCK_SKIP);
  do {
    r = SEGGER_RTT_WaitKey();
    SEGGER_RTT_Write(0, &r, 1);
    r++;
  } while (1);
#endif
    /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSE;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_I2C1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_SYSCLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  HAL_GPIO_WritePin(LED_Red_GPIO_Port, LED_Red_Pin, GPIO_PIN_SET);
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
