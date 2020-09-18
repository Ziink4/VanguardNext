/**
 * |----------------------------------------------------------------------
 * | Copyright (c) 2016 Tilen MAJERLE
 * |  
 * | Permission is hereby granted, free of charge, to any person
 * | obtaining a copy of this software and associated documentation
 * | files (the "Software"), to deal in the Software without restriction,
 * | including without limitation the rights to use, copy, modify, merge,
 * | publish, distribute, sublicense, and/or sell copies of the Software, 
 * | and to permit persons to whom the Software is furnished to do so, 
 * | subject to the following conditions:
 * | 
 * | The above copyright notice and this permission notice shall be
 * | included in all copies or substantial portions of the Software.
 * | 
 * | THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * | EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
 * | OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE
 * | AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
 * | HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
 * | WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING 
 * | FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 * | OTHER DEALINGS IN THE SOFTWARE.
 * |----------------------------------------------------------------------
 */
#include "mpu6050.h"


MPU6050_Result_t MPU6050_Init(MPU6050_t *DataStruct, I2C_HandleTypeDef *hi2c, MPU6050_Device_t DeviceNumber, MPU6050_Accelerometer_t AccelerometerSensitivity, MPU6050_Gyroscope_t GyroscopeSensitivity)
{
  // Store I2C Handle
  DataStruct->Handle = hi2c;

  // Format I2C address
  DataStruct->Address = MPU6050_I2C_ADDR | (uint8_t)DeviceNumber;

  // Check if device is connected
  if (HAL_I2C_IsDeviceReady(hi2c, DataStruct->Address, 2, MPU6050_I2C_TIMEOUT) != HAL_OK)
  {
      return MPU6050_Result_DeviceNotConnected;
  }

  // Check who am I
  uint8_t who_am_i;
  if (HAL_I2C_Mem_Read(hi2c, DataStruct->Address, MPU6050_WHO_AM_I, I2C_MEMADD_SIZE_8BIT, &who_am_i, sizeof(who_am_i), MPU6050_I2C_TIMEOUT) != HAL_OK)
  {
      return MPU6050_Result_Error;
  }

  if (who_am_i != MPU6050_I_AM)
  {
      return MPU6050_Result_DeviceInvalid;
  }

  // Wakeup MPU6050
  uint8_t pwr_mgmt_1 = 0x00;
  if (HAL_I2C_Mem_Write(hi2c, DataStruct->Address, MPU6050_PWR_MGMT_1, I2C_MEMADD_SIZE_8BIT, &pwr_mgmt_1, sizeof(pwr_mgmt_1), MPU6050_I2C_TIMEOUT) != HAL_OK)
  {
      return MPU6050_Result_Error;
  }

  // Set sample rate to 1kHz
  if (MPU6050_SetDataRate(DataStruct, MPU6050_DataRate_1KHz) != MPU6050_Result_Ok)
  {
      return MPU6050_Result_Error;
  }

  // Config accelerometer
  if (MPU6050_SetAccelerometer(DataStruct, AccelerometerSensitivity) != MPU6050_Result_Ok)
  {
      return MPU6050_Result_Error;
  }

  // Config accelerometer
  if (MPU6050_SetGyroscope(DataStruct, GyroscopeSensitivity) != MPU6050_Result_Ok)
  {
      return MPU6050_Result_Error;
  }

  return MPU6050_Result_Ok;
}

MPU6050_Result_t MPU6050_SetGyroscope(MPU6050_t *DataStruct, MPU6050_Gyroscope_t GyroscopeSensitivity)
{
  // Read current gyroscope configuration register
  uint8_t gyro_config;
  if (HAL_I2C_Mem_Read(DataStruct->Handle, DataStruct->Address, MPU6050_GYRO_CONFIG, I2C_MEMADD_SIZE_8BIT, &gyro_config, sizeof(gyro_config), MPU6050_I2C_TIMEOUT) != HAL_OK)
  {
      return MPU6050_Result_Error;
  }

  // Mask out FS_SEL and set new sensitivity
  gyro_config = (gyro_config & 0xE7) | (uint8_t)GyroscopeSensitivity << 3;

  // Write modified register
  if (HAL_I2C_Mem_Write(DataStruct->Handle, DataStruct->Address, MPU6050_GYRO_CONFIG, I2C_MEMADD_SIZE_8BIT, &gyro_config, sizeof(gyro_config), MPU6050_I2C_TIMEOUT) != HAL_OK)
  {
      return MPU6050_Result_Error;
  }

  switch (GyroscopeSensitivity)
  {
    case MPU6050_Gyroscope_250s:
      DataStruct->Gyro_Mult = (float)1 / MPU6050_GYRO_SENS_250;
      break;
    case MPU6050_Gyroscope_500s:
      DataStruct->Gyro_Mult = (float)1 / MPU6050_GYRO_SENS_500;
      break;
    case MPU6050_Gyroscope_1000s:
      DataStruct->Gyro_Mult = (float)1 / MPU6050_GYRO_SENS_1000;
      break;
    case MPU6050_Gyroscope_2000s:
      DataStruct->Gyro_Mult = (float)1 / MPU6050_GYRO_SENS_2000;
    default:
      break;
  }

  return MPU6050_Result_Ok;
}

MPU6050_Result_t MPU6050_SetAccelerometer(MPU6050_t *DataStruct, MPU6050_Accelerometer_t AccelerometerSensitivity)
{
  // Read current accelerometer configuration register
  uint8_t accel_config;
  if (HAL_I2C_Mem_Read(DataStruct->Handle, DataStruct->Address, MPU6050_ACCEL_CONFIG, I2C_MEMADD_SIZE_8BIT, &accel_config, sizeof(accel_config), MPU6050_I2C_TIMEOUT) != HAL_OK)
  {
      return MPU6050_Result_Error;
  }
  // Mask out AFS_SEL and set new sensitivity
  accel_config = (accel_config & 0xE7) | (uint8_t)AccelerometerSensitivity << 3;

  // Write modified register
  if (HAL_I2C_Mem_Write(DataStruct->Handle, DataStruct->Address, MPU6050_ACCEL_CONFIG, I2C_MEMADD_SIZE_8BIT, &accel_config, sizeof(accel_config), MPU6050_I2C_TIMEOUT) != HAL_OK)
  {
      return MPU6050_Result_Error;
  }

  // Set sensitivities for multiplying gyro and accelerometer data
  switch (AccelerometerSensitivity)
  {
    case MPU6050_Accelerometer_2G:
      DataStruct->Acce_Mult = (float)1 / MPU6050_ACCE_SENS_2;
      break;
    case MPU6050_Accelerometer_4G:
      DataStruct->Acce_Mult = (float)1 / MPU6050_ACCE_SENS_4;
      break;
    case MPU6050_Accelerometer_8G:
      DataStruct->Acce_Mult = (float)1 / MPU6050_ACCE_SENS_8;
      break;
    case MPU6050_Accelerometer_16G:
      DataStruct->Acce_Mult = (float)1 / MPU6050_ACCE_SENS_16;
    default:
      break;
  }

  return MPU6050_Result_Ok;
}

MPU6050_Result_t MPU6050_SetDataRate(MPU6050_t *DataStruct, uint8_t rate)
{
  // Set data sample rate
  if (HAL_I2C_Mem_Write(DataStruct->Handle, DataStruct->Address, MPU6050_SMPLRT_DIV, I2C_MEMADD_SIZE_8BIT, &rate, sizeof(rate), MPU6050_I2C_TIMEOUT) != HAL_OK)
  {
      return MPU6050_Result_Error;
  }

  return MPU6050_Result_Ok;
}

MPU6050_Result_t MPU6050_EnableInterrupts(MPU6050_t *DataStruct, bool dmp_enabled)
{
  uint8_t int_enable = dmp_enabled ? MPU6050_INT_EN_DMP_INT : MPU6050_INT_EN_DATA_RDY;
  if (HAL_I2C_Mem_Write(DataStruct->Handle, DataStruct->Address, MPU6050_INT_ENABLE, I2C_MEMADD_SIZE_8BIT, &int_enable, sizeof(int_enable), MPU6050_I2C_TIMEOUT) != HAL_OK)
  {
      return MPU6050_Result_Error;
  }

  return MPU6050_Result_Ok;
}

MPU6050_Result_t MPU6050_DisableInterrupts(MPU6050_t *DataStruct)
{
  uint8_t int_enable = 0x00;
  if (HAL_I2C_Mem_Write(DataStruct->Handle, DataStruct->Address, MPU6050_INT_ENABLE, I2C_MEMADD_SIZE_8BIT, &int_enable, sizeof(int_enable), MPU6050_I2C_TIMEOUT) != HAL_OK)
  {
      return MPU6050_Result_Error;
  }

  return MPU6050_Result_Ok;
}

/*
// TODO : Use interrupts
MPU6050_Result_t MPU6050_EnableInterrupts(MPU6050_t* DataStruct)
{
  uint8_t temp;

  // Enable interrupts for data ready and motion detect
  TM_I2C_Write(MPU6050_I2C, DataStruct->Address, MPU6050_INT_ENABLE, 0x21);

  // Clear IRQ flag on any read operation
  TM_I2C_Read(MPU6050_I2C, DataStruct->Address, MPU6050_INT_PIN_CFG, &temp);
  temp |= 0x10;
  TM_I2C_Write(MPU6050_I2C, DataStruct->Address, MPU6050_INT_PIN_CFG, temp);

  return MPU6050_Result_Ok;
}

MPU6050_Result_t MPU6050_ReadInterrupts(MPU6050_t *DataStruct, MPU6050_Interrupt_t *InterruptsStruct)
{
  uint8_t read;

  // Reset structure
  InterruptsStruct->Status = 0;

  // Read interrupts status register
  if (TM_I2C_Read(MPU6050_I2C, DataStruct->Address, MPU6050_INT_STATUS, &read) != TM_I2C_Result_Ok)
  {
      return MPU6050_Result_Error;
  }

  // Fill value
  InterruptsStruct->Status = read;

  return MPU6050_Result_Ok;
}
*/

MPU6050_Result_t MPU6050_ReadAccelerometer(MPU6050_t *DataStruct)
{
  // Read accelerometer data
  uint8_t accelerometer_data[6];
  if (HAL_I2C_Mem_Read(DataStruct->Handle, DataStruct->Address, MPU6050_ACCEL_XOUT_H, I2C_MEMADD_SIZE_8BIT, accelerometer_data, sizeof(accelerometer_data), MPU6050_I2C_TIMEOUT) != HAL_OK)
  {
      return MPU6050_Result_Error;
  }

  // Format
  DataStruct->Accelerometer_X = (int16_t)(accelerometer_data[0] << 8 | accelerometer_data[1]);
  DataStruct->Accelerometer_Y = (int16_t)(accelerometer_data[2] << 8 | accelerometer_data[3]);
  DataStruct->Accelerometer_Z = (int16_t)(accelerometer_data[4] << 8 | accelerometer_data[5]);

  return MPU6050_Result_Ok;
}

MPU6050_Result_t MPU6050_ReadGyroscope(MPU6050_t *DataStruct)
{
  // Read gyroscope data
  uint8_t gyroscope_data[6];
  if (HAL_I2C_Mem_Read(DataStruct->Handle, DataStruct->Address, MPU6050_GYRO_XOUT_H, I2C_MEMADD_SIZE_8BIT, gyroscope_data, sizeof(gyroscope_data), MPU6050_I2C_TIMEOUT) != HAL_OK)
  {
      return MPU6050_Result_Error;
  }

  // Format
  DataStruct->Gyroscope_X = (int16_t)(gyroscope_data[0] << 8 | gyroscope_data[1]);
  DataStruct->Gyroscope_Y = (int16_t)(gyroscope_data[2] << 8 | gyroscope_data[3]);
  DataStruct->Gyroscope_Z = (int16_t)(gyroscope_data[4] << 8 | gyroscope_data[5]);

  return MPU6050_Result_Ok;
}

MPU6050_Result_t MPU6050_ReadTemperature(MPU6050_t *DataStruct)
{
  // Read temperature
  uint8_t temperature_data[2];
  if (HAL_I2C_Mem_Read(DataStruct->Handle, DataStruct->Address, MPU6050_TEMP_OUT_H, I2C_MEMADD_SIZE_8BIT, temperature_data, sizeof(temperature_data), MPU6050_I2C_TIMEOUT) != HAL_OK)
  {
      return MPU6050_Result_Error;
  }

  // Format temperature
  DataStruct->Temperature = (float)(((int16_t)(temperature_data[0] << 8 | temperature_data[1])) / 340.0f + 36.53f);

  return MPU6050_Result_Ok;
}

MPU6050_Result_t MPU6050_ReadAll(MPU6050_t *DataStruct)
{
  // Read full raw data, 14bytes
  uint8_t data[14];
  if (HAL_I2C_Mem_Read(DataStruct->Handle, DataStruct->Address, MPU6050_ACCEL_XOUT_H, I2C_MEMADD_SIZE_8BIT, data, sizeof(data), MPU6050_I2C_TIMEOUT) != HAL_OK)
  {
      return MPU6050_Result_Error;
  }

  // Format accelerometer data
  DataStruct->Accelerometer_X = (int16_t)(data[0] << 8 | data[1]);
  DataStruct->Accelerometer_Y = (int16_t)(data[2] << 8 | data[3]);
  DataStruct->Accelerometer_Z = (int16_t)(data[4] << 8 | data[5]);

  // Format temperature
  DataStruct->Temperature = (float)(((int16_t)(data[6] << 8 | data[7])) / 340.0f + 36.53f);

  // Format gyroscope data
  DataStruct->Gyroscope_X = (int16_t)(data[8] << 8 | data[9]);
  DataStruct->Gyroscope_Y = (int16_t)(data[10] << 8 | data[11]);
  DataStruct->Gyroscope_Z = (int16_t)(data[12] << 8 | data[13]);

  return MPU6050_Result_Ok;
}
