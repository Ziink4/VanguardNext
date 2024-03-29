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
#include "log.h"
#include "cc2500.h"
#include "sfhss.h"
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
  SEGGER_RTT_Init();
  log_init();
  LOG_LOGD("HAL init done");
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  LOG_LOGD("System clock init done");
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_SPI1_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
  LOG_LOGD("Peripheral init done");
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

#define TEST_ALL 1
//#define TEST_ECHO 1
//#define TEST_CC2500 1

#if TEST_ALL
  // Gyro Interface
  MPU6050_t mpu_handle;
  LOG_LOGD("Gyro init...");
  if (MPU6050_Init(&mpu_handle, &hi2c1, MPU6050_Device_0, MPU6050_Accelerometer_2G, MPU6050_Gyroscope_250s) != MPU6050_Result_Ok)
  {
    LOG_LOGE("Gyro init failed");
    Error_Handler();
  }

  if (MPU6050_DMP_LoadFirmware(&mpu_handle) != MPU6050_Result_Ok)
  {
    LOG_LOGE("Gyro firmware loading failed");
    Error_Handler();
  }

  if (MPU6050_DMP_SetState(&mpu_handle, true) != MPU6050_Result_Ok)
  {
    LOG_LOGE("Gyro firmware init failed");
    Error_Handler();
  }

  LOG_LOGD("Gyro init done");

  float gyro_integral = 0.0f;
  uint32_t gyro_tick = 0;

  // Init Gyro Status LED
  LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH1N);

  // Tail Control
  const PID_Terms tail_control = { 1.0f / 33.0f, 0.0f, 0.0f };
  PID_History tail_control_history = { 0.0f, 0.0f };
  uint32_t tail_tick = 0;
  uint32_t tail_max_output = 1000;

  LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH2N);

  // TIM1 Enable
  LL_TIM_EnableAllOutputs(TIM1);
  LL_TIM_EnableCounter(TIM1);

  while (1)
  {
    // Can be used for debug or maybe to cutoff tail motor ?
    const bool button_pressed = (HAL_GPIO_ReadPin(Button_GPIO_Port, Button_Pin) == GPIO_PIN_RESET);

    if (MPU6050_ReadAll(&mpu_handle) != MPU6050_Result_Ok)
    {
      LOG_LOGE("Gyro read failed");
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
    if (mpu_handle.Gyroscope_Z == -32768 || mpu_handle.Gyroscope_Z == 32767)
    {
      LOG_LOGE("DATA: %6d %6d %6d %6d %6d %6d", mpu_handle.Gyroscope_X, mpu_handle.Gyroscope_Y, mpu_handle.Gyroscope_Z, mpu_handle.Accelerometer_X, mpu_handle.Accelerometer_Y, mpu_handle.Accelerometer_Z);
    }
    else
    {
      LOG_LOGI("DATA: %6d %6d %6d %6d %6d %6d", mpu_handle.Gyroscope_X, mpu_handle.Gyroscope_Y, mpu_handle.Gyroscope_Z, mpu_handle.Accelerometer_X, mpu_handle.Accelerometer_Y, mpu_handle.Accelerometer_Z);
    }

    uint16_t gyro_led_output = mpu_handle.Gyroscope_Z < 0 ? 0 : (mpu_handle.Gyroscope_Z / 33);
    LL_TIM_OC_SetCompareCH1(TIM1, gyro_led_output);

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

#if TEST_ECHO
  char r = 'Z';

  LOG_LOGI("Running echo test sample...");

  do {
    LOG_LOGI("Waiting for a key press...");
    r = SEGGER_RTT_WaitKey();
    r++;
    LOG_LOGI(&r);
  } while (1);
#endif

#if TEST_CC2500
  CC2500CTX cc2500_ctx;

  LOG_LOGD("CC2500 init...");

  if (!cc2500_init(&cc2500_ctx, RF_CSn_GPIO_Port, RF_CSn_Pin, &hspi1))
  {
	  Error_Handler();
  }

  LOG_LOGD("CC2500 init done");

  LOG_LOGD("SFHSS init...");

  SFHSSCTX sfhss_ctx;
  sfhss_init(&sfhss_ctx, &cc2500_ctx);
  sfhss_calibrate(&sfhss_ctx);

  LOG_LOGD("S-FHSS init done");

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
  LL_FLASH_SetLatency(LL_FLASH_LATENCY_0);
  while(LL_FLASH_GetLatency() != LL_FLASH_LATENCY_0)
  {
  }
  LL_RCC_HSE_Enable();

   /* Wait till HSE is ready */
  while(LL_RCC_HSE_IsReady() != 1)
  {

  }
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_HSE);

   /* Wait till System clock is ready */
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_HSE)
  {

  }
  LL_SetSystemCoreClock(16000000);

   /* Update the time base */
  if (HAL_InitTick (TICK_INT_PRIORITY) != HAL_OK)
  {
    Error_Handler();
  }
  LL_RCC_SetI2CClockSource(LL_RCC_I2C1_CLKSOURCE_SYSCLK);
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
  LOG_LOGE("HAL error handler reached");

  while(1)
  {
    HAL_GPIO_WritePin(LED_Red_GPIO_Port, LED_Red_Pin, GPIO_PIN_SET);
    HAL_Delay(100);
    HAL_GPIO_WritePin(LED_Red_GPIO_Port, LED_Red_Pin, GPIO_PIN_RESET);
    HAL_Delay(100);
  }
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
  LOG_LOGE("Assertion failed: file %s on line %d\r\n", file, line);
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
