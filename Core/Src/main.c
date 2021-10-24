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
#include "MPU6050/mpu6050.h"
#include "pid_control.h"
#include "Log/log.h"
#include "CC2500/cc2500.h"
#include "SFHSS/sfhss.h"
#include "SEGGER_RTT/SEGGER_RTT.h"
#include "mltypes.h"
#include "MPU6050/eMD6/eMPL/inv_mpu.h"
#include "MPU6050/eMD6/eMPL/inv_mpu_dmp_motion_driver.h"
#include "MPU6050/eMD6/mllite/invensense.h"

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
#define TEST_INV_MPU 1
//#define TEST_MPU 1
//#define TEST_ECHO 1
//#define TEST_CC2500 1
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

#if TEST_INV_MPU
  struct int_param_s int_param;
  inv_error_t result = mpu_init(&int_param);
  if (result) {
    LOG_LOGE("Could not initialize gyro");
  }
  else
  {
    LOG_LOGI("Gyro init done");
  }

  /* Get/set hardware configuration. Start gyro. */
  /* Wake up all sensors. */
  int r;
  r = mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL);
  /* Push both gyro and accel data into the FIFO. */
  r |= mpu_configure_fifo(INV_XYZ_GYRO | INV_XYZ_ACCEL);

#define DEFAULT_MPU_HZ  (20)
  r |= mpu_set_sample_rate(DEFAULT_MPU_HZ);
  /* Read back configuration in case it was set improperly. */
  unsigned char accel_fsr,  new_temp = 0;
  unsigned short gyro_rate, gyro_fsr;
  unsigned long timestamp;

  r |= mpu_get_sample_rate(&gyro_rate);
  r |= mpu_get_gyro_fsr(&gyro_fsr);
  r |= mpu_get_accel_fsr(&accel_fsr);

  if (r)
  {
    LOG_LOGE("Could not configure sensors");
  }
  else
  {
    LOG_LOGI("Sensor configuration done");
  }

  /* Sync driver configuration with MPL. */
  /* Sample rate expected in microseconds. */
  inv_set_gyro_sample_rate(1000000L / gyro_rate);
  inv_set_accel_sample_rate(1000000L / gyro_rate);

  /* Set chip-to-body orientation matrix.
   * Set hardware units to dps/g's/degrees scaling factor.
   */

  /* Platform-specific information. Kinda like a boardfile. */
  struct platform_data_s {
    signed char orientation[9];
  };

  static struct platform_data_s gyro_pdata = {
      .orientation = { 1, 0, 0,
          0, 1, 0,
          0, 0, 1}
  };

  inv_set_gyro_orientation_and_scale(
      inv_orientation_matrix_to_scalar(gyro_pdata.orientation),
      (long)gyro_fsr<<15);
  inv_set_accel_orientation_and_scale(
      inv_orientation_matrix_to_scalar(gyro_pdata.orientation),
      (long)accel_fsr<<15);

  r = dmp_load_motion_driver_firmware();
  r |= dmp_set_orientation(inv_orientation_matrix_to_scalar(gyro_pdata.orientation));

  unsigned short dmp_features = DMP_FEATURE_6X_LP_QUAT | DMP_FEATURE_SEND_RAW_ACCEL | DMP_FEATURE_SEND_CAL_GYRO |
      DMP_FEATURE_GYRO_CAL;
  r |= dmp_enable_feature(dmp_features);
  r |= dmp_set_fifo_rate(DEFAULT_MPU_HZ);
  r |= mpu_set_dmp_state(1);
  unsigned char dmp_on = 1;

  if (r)
  {
    LOG_LOGE("Could not configure DMP");
  }
  else
  {
    LOG_LOGI("DMP configuration done");
  }

  unsigned char lp_accel_mode = 0;
#define ACCEL_ON        (0x01)
#define GYRO_ON         (0x02)
  unsigned char sensors = ACCEL_ON | GYRO_ON;
  volatile unsigned char new_gyro = 0;
  unsigned long next_temp_ms = 0;
  unsigned int report = 0;

  while(1)
  {
    unsigned long sensor_timestamp;
    int new_data = 0;

    timestamp = HAL_GetTick();

    /* Temperature data doesn't need to be read with every gyro sample.
     * Let's make them timer-based like the compass reads.
     */
    if (timestamp > next_temp_ms)
    {
#define TEMP_READ_MS    (500)
      next_temp_ms = timestamp + TEMP_READ_MS;
      new_temp = 1;
    }

    HAL_Delay(500);
    new_gyro = 1;

    if (!sensors || !new_gyro)
    {
      continue;
    }

    if (new_gyro && lp_accel_mode)
    {
      short accel_short[3];
      long accel[3];
      mpu_get_accel_reg(accel_short, &sensor_timestamp);
      accel[0] = (long)accel_short[0];
      accel[1] = (long)accel_short[1];
      accel[2] = (long)accel_short[2];
      inv_build_accel(accel, 0, sensor_timestamp);
      new_data = 1;
      new_gyro = 0;
    }
    else if (new_gyro && dmp_on)
    {
      short gyro[3], accel_short[3], sensors;
      unsigned char more;
      long accel[3], quat[4], temperature;
      /* This function gets new data from the FIFO when the DMP is in
       * use. The FIFO can contain any combination of gyro, accel,
       * quaternion, and gesture data. The sensors parameter tells the
       * caller which data fields were actually populated with new data.
       * For example, if sensors == (INV_XYZ_GYRO | INV_WXYZ_QUAT), then
       * the FIFO isn't being filled with accel data.
       * The driver parses the gesture data to determine if a gesture
       * event has occurred; on an event, the application will be notified
       * via a callback (assuming that a callback function was properly
       * registered). The more parameter is non-zero if there are
       * leftover packets in the FIFO.
       */
      if (dmp_read_fifo(gyro, accel_short, quat, &sensor_timestamp, &sensors, &more))
      {
        LOG_LOGE("Failed to read DMP FIFO");
      }

      if (!more)
      {
        new_gyro = 0;
      }

      if (sensors & INV_XYZ_GYRO)
      {
        /* Push the new data to the MPL. */
        inv_build_gyro(gyro, sensor_timestamp);
        new_data = 1;
        if (new_temp)
        {
          new_temp = 0;
          /* Temperature only used for gyro temp comp. */
          mpu_get_temperature(&temperature, &sensor_timestamp);
          inv_build_temp(temperature, sensor_timestamp);
        }
      }
      if (sensors & INV_XYZ_ACCEL)
      {
        accel[0] = (long)accel_short[0];
        accel[1] = (long)accel_short[1];
        accel[2] = (long)accel_short[2];
        inv_build_accel(accel, 0, sensor_timestamp);
        new_data = 1;
      }
      if (sensors & INV_WXYZ_QUAT)
      {
        inv_build_quat(quat, 0, sensor_timestamp);
        new_data = 1;
      }
    }
    else if (new_gyro)
    {
      short gyro[3], accel_short[3];
      unsigned char sensors, more;
      long accel[3], temperature;
      /* This function gets new data from the FIFO. The FIFO can contain
       * gyro, accel, both, or neither. The sensors parameter tells the
       * caller which data fields were actually populated with new data.
       * For example, if sensors == INV_XYZ_GYRO, then the FIFO isn't
       * being filled with accel data. The more parameter is non-zero if
       * there are leftover packets in the FIFO. The HAL can use this
       * information to increase the frequency at which this function is
       * called.
       */
      new_gyro = 0;
      mpu_read_fifo(gyro, accel_short, &sensor_timestamp,
          &sensors, &more);
      if (more)
        new_gyro = 1;
      if (sensors & INV_XYZ_GYRO) {
        /* Push the new data to the MPL. */
        inv_build_gyro(gyro, sensor_timestamp);
        new_data = 1;
        if (new_temp) {
          new_temp = 0;
          /* Temperature only used for gyro temp comp. */
          mpu_get_temperature(&temperature, &sensor_timestamp);
          inv_build_temp(temperature, sensor_timestamp);
        }
      }
      if (sensors & INV_XYZ_ACCEL)
      {
        accel[0] = (long)accel_short[0];
        accel[1] = (long)accel_short[1];
        accel[2] = (long)accel_short[2];
        inv_build_accel(accel, 0, sensor_timestamp);
        new_data = 1;
      }
    }

    if (new_data)
    {
      if (inv_execute_on_data())
      {
        LOG_LOGE("Failed to process data");
      }

      int8_t accuracy;
      float float_data[3] = {0};
      if (inv_get_sensor_type_linear_acceleration(float_data, &accuracy, (inv_time_t*)&timestamp))
      {
        LOG_LOGI("Linear Accel: %7.5f %7.5f %7.5f\r\n", float_data[0], float_data[1], float_data[2]);
       }

      if (inv_get_sensor_type_gyroscope_raw(float_data, &accuracy, (inv_time_t*)&timestamp))
      {
        LOG_LOGI("Gravity Vector: %7.5f %7.5f %7.5f\r\n", float_data[0], float_data[1], float_data[2]);
      }
    }
  }

#endif


#if TEST_MPU
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
