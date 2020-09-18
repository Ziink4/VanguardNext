/*
 * mpu6050_dmp.c
 *
 *  Created on: 11 juil. 2020
 *      Author: Florian
 */

#include <sys/param.h>
#include <string.h>
#include <stdbool.h>

#include "mpu6050.h"
#include "mpu6050_dmp.h"

/**
 *  @brief      Write to the DMP memory.
 *  This function prevents I2C writes past the bank boundaries. The DMP memory
 *  is only accessible when the chip is awake.
 *  @param[in]  mem_addr    Memory location (bank << 8 | start address)
 *  @param[in]  length      Number of bytes to write.
 *  @param[in]  data        Bytes to write to memory.
 *  @return     0 if successful.
 */
MPU6050_Result_t MPU6050_DMP_WriteMemory(MPU6050_t *DataStruct, uint16_t mem_addr, uint16_t length, const uint8_t *data)
{
  const uint8_t tmp[2] = {
      (uint8_t) (mem_addr >> 8),
      (uint8_t) (mem_addr & 0xFF)
  };

  // Check bank boundaries
  if (tmp[1] + length > MPU6050_DMP_BANK_SIZE)
  {
    return MPU6050_Result_Error;
  }

  if (HAL_I2C_Mem_Write(DataStruct->Handle, DataStruct->Address, MPU6050_BANK_SEL, I2C_MEMADD_SIZE_8BIT, tmp, sizeof(tmp), MPU6050_I2C_TIMEOUT) != HAL_OK)
  {
    return MPU6050_Result_Error;
  }

  if (HAL_I2C_Mem_Write(DataStruct->Handle, DataStruct->Address, MPU6050_MEM_R_W, I2C_MEMADD_SIZE_8BIT, data, length, MPU6050_I2C_TIMEOUT) != HAL_OK)
  {
    return MPU6050_Result_Error;
  }

  return MPU6050_Result_Ok;
}

/**
 *  @brief      Read from the DMP memory.
 *  This function prevents I2C reads past the bank boundaries. The DMP memory
 *  is only accessible when the chip is awake.
 *  @param[in]  mem_addr    Memory location (bank << 8 | start address)
 *  @param[in]  length      Number of bytes to read.
 *  @param[out] data        Bytes read from memory.
 *  @return     0 if successful.
 */
MPU6050_Result_t MPU6050_DMP_ReadMemory(MPU6050_t *DataStruct, uint16_t mem_addr, uint16_t length, uint8_t *data)
{
  const uint8_t tmp[2] =
  {
      (uint8_t) (mem_addr >> 8),
      (uint8_t) (mem_addr & 0xFF)
  };

  // Check bank boundaries
  if (tmp[1] + length > MPU6050_DMP_BANK_SIZE)
  {
    return MPU6050_Result_Error;
  }

  // Select bank
  if (HAL_I2C_Mem_Write(DataStruct->Handle, DataStruct->Address, MPU6050_BANK_SEL, I2C_MEMADD_SIZE_8BIT, tmp, sizeof(tmp), MPU6050_I2C_TIMEOUT) != HAL_OK)
  {
    return MPU6050_Result_Error;
  }

  if (HAL_I2C_Mem_Read(DataStruct->Handle, DataStruct->Address, MPU6050_MEM_R_W, I2C_MEMADD_SIZE_8BIT, data, length, MPU6050_I2C_TIMEOUT) != HAL_OK)
  {
    return MPU6050_Result_Error;
  }

  return MPU6050_Result_Ok;
}

/**
 *  @brief      Load and verify DMP image.
 *  @param[in]  length      Length of DMP image.
 *  @param[in]  firmware    DMP code.
 *  @param[in]  start_addr  Starting address of DMP code memory.
 *  @param[in]  sample_rate Fixed sampling rate used when DMP is enabled.
 *  @return     0 if successful.
 */
MPU6050_Result_t MPU6050_DMP_LoadFirmware(MPU6050_t *DataStruct)
{
  // Must divide evenly into st.hw->bank_size to avoid bank crossings
  uint8_t cur[MPU6050_DMP_FIRMWARE_LOAD_CHUNK];
  uint16_t this_write;

  for (uint16_t firmware_offset = 0; firmware_offset < MPU6050_DMP_FIRMWARE_SIZE; firmware_offset += this_write)
  {
    this_write = MIN(MPU6050_DMP_FIRMWARE_LOAD_CHUNK, MPU6050_DMP_FIRMWARE_SIZE - firmware_offset);

    if (MPU6050_DMP_WriteMemory(DataStruct, firmware_offset, this_write, &MPU6050_DMP_FIRMWARE[firmware_offset]) != MPU6050_Result_Ok)
    {
      return MPU6050_Result_Error;
    }

    if (MPU6050_DMP_ReadMemory(DataStruct, firmware_offset, this_write, cur) != MPU6050_Result_Ok)
    {
      return MPU6050_Result_Error;
    }

    // FIXME : Fix memory comparison not correct
    /*
    if (memcmp(MPU6050_DMP_FIRMWARE + firmware_offset, cur, this_write))
    {
      return MPU6050_Result_Error;
    }
    */
  }

  // Set program start address
  const uint8_t tmp[2] =
  {
      (uint8_t) (MPU6050_DMP_FIRMWARE_START_ADDR >> 8),
      (uint8_t) (MPU6050_DMP_FIRMWARE_START_ADDR & 0xFF)
  };

  if (HAL_I2C_Mem_Write(DataStruct->Handle, DataStruct->Address, MPU6050_PRGM_START_H, I2C_MEMADD_SIZE_8BIT, tmp, sizeof(tmp), MPU6050_I2C_TIMEOUT) != HAL_OK)
  {
    return MPU6050_Result_Error;
  }

  return MPU6050_Result_Ok;
}

/**
 *  @brief  Reset FIFO read/write pointers.
 *  @return 0 if successful.
 */
MPU6050_Result_t MPU6050_DMP_ResetFIFO(MPU6050_t *DataStruct, bool dmp_enabled, bool int_enabled, uint8_t fifo_enable)
{
  uint8_t data = 0;

  if (MPU6050_DisableInterrupts(DataStruct) != MPU6050_Result_Ok)
  {
    return MPU6050_Result_Error;
  }

  if (HAL_I2C_Mem_Write(DataStruct->Handle, DataStruct->Address, MPU6050_FIFO_EN, I2C_MEMADD_SIZE_8BIT, &data, sizeof(data), MPU6050_I2C_TIMEOUT) != HAL_OK)
  {
    return MPU6050_Result_Error;
  }

  if (HAL_I2C_Mem_Write(DataStruct->Handle, DataStruct->Address, MPU6050_USER_CTRL, I2C_MEMADD_SIZE_8BIT, &data, sizeof(data), MPU6050_I2C_TIMEOUT) != HAL_OK)
  {
    return MPU6050_Result_Error;
  }

  if (dmp_enabled)
  {
    data = MPU6050_USER_CTRL_FIFO_RST | MPU6050_USER_CTRL_DMP_RST;
    if (HAL_I2C_Mem_Write(DataStruct->Handle, DataStruct->Address, MPU6050_USER_CTRL, I2C_MEMADD_SIZE_8BIT, &data, sizeof(data), MPU6050_I2C_TIMEOUT) != HAL_OK)
    {
      return MPU6050_Result_Error;
    }

    HAL_Delay(50);

    data = MPU6050_USER_CTRL_DMP_EN | MPU6050_USER_CTRL_FIFO_EN;
    if (HAL_I2C_Mem_Write(DataStruct->Handle, DataStruct->Address, MPU6050_USER_CTRL, I2C_MEMADD_SIZE_8BIT, &data, sizeof(data), MPU6050_I2C_TIMEOUT) != HAL_OK)
    {
      return MPU6050_Result_Error;
    }

    data = int_enabled ? MPU6050_INT_EN_DMP_INT : 0;
    if (HAL_I2C_Mem_Write(DataStruct->Handle, DataStruct->Address, MPU6050_INT_ENABLE, I2C_MEMADD_SIZE_8BIT, &data, sizeof(data), MPU6050_I2C_TIMEOUT) != HAL_OK)
    {
      return MPU6050_Result_Error;
    }

    data = 0;
    if (HAL_I2C_Mem_Write(DataStruct->Handle, DataStruct->Address, MPU6050_FIFO_EN, I2C_MEMADD_SIZE_8BIT, &data, sizeof(data), MPU6050_I2C_TIMEOUT) != HAL_OK)
    {
      return MPU6050_Result_Error;
    }
  }
  else
  {
    data = MPU6050_USER_CTRL_FIFO_RST;
    if (HAL_I2C_Mem_Write(DataStruct->Handle, DataStruct->Address, MPU6050_USER_CTRL, I2C_MEMADD_SIZE_8BIT, &data, sizeof(data), MPU6050_I2C_TIMEOUT) != HAL_OK)
    {
      return MPU6050_Result_Error;
    }

    data = MPU6050_USER_CTRL_FIFO_EN;
    if (HAL_I2C_Mem_Write(DataStruct->Handle, DataStruct->Address, MPU6050_USER_CTRL, I2C_MEMADD_SIZE_8BIT, &data, sizeof(data), MPU6050_I2C_TIMEOUT) != HAL_OK)
    {
      return MPU6050_Result_Error;
    }

    HAL_Delay(50);

    data = int_enabled ? MPU6050_INT_EN_DATA_RDY : 0;
    if (HAL_I2C_Mem_Write(DataStruct->Handle, DataStruct->Address, MPU6050_INT_ENABLE, I2C_MEMADD_SIZE_8BIT, &data, sizeof(data), MPU6050_I2C_TIMEOUT) != HAL_OK)
    {
      return MPU6050_Result_Error;
    }

    if (HAL_I2C_Mem_Write(DataStruct->Handle, DataStruct->Address, MPU6050_FIFO_EN, I2C_MEMADD_SIZE_8BIT, &fifo_enable, sizeof(fifo_enable), MPU6050_I2C_TIMEOUT) != HAL_OK)
    {
      return MPU6050_Result_Error;
    }
  }

  return MPU6050_Result_Ok;
}

MPU6050_Result_t MPU6050_DMP_SetState(MPU6050_t *DataStruct, bool enabled)
{
  if (enabled)
  {
    /* Disable data ready interrupt. */
    if (MPU6050_DisableInterrupts(DataStruct) != MPU6050_Result_Ok)
    {
      return MPU6050_Result_Error;
    }

    /* Keep constant sample rate, FIFO rate controlled by DMP. */
    if (MPU6050_SetDataRate(DataStruct, MPU6050_DataRate_1KHz) != MPU6050_Result_Ok)
    {
        return MPU6050_Result_Error;
    }

    /* Remove FIFO elements. */
    uint8_t tmp = 0;
    if (HAL_I2C_Mem_Write(DataStruct->Handle, DataStruct->Address, MPU6050_FIFO_EN, I2C_MEMADD_SIZE_8BIT, &tmp, sizeof(tmp), MPU6050_I2C_TIMEOUT) != HAL_OK)
    {
      return MPU6050_Result_Error;
    }

    /* Enable DMP interrupt. */
    if (MPU6050_EnableInterrupts(DataStruct, true) != MPU6050_Result_Ok)
    {
      return MPU6050_Result_Error;
    }

    if (MPU6050_DMP_ResetFIFO(DataStruct, true, true, 0) != MPU6050_Result_Ok)
    {
      return MPU6050_Result_Error;
    }
  }
  else
  {
    /* Disable DMP interrupt. */
    if (MPU6050_DisableInterrupts(DataStruct) != MPU6050_Result_Ok)
    {
      return MPU6050_Result_Error;
    }

    /* Restore FIFO settings. */
    if (MPU6050_DMP_ResetFIFO(DataStruct, false, false, MPU6050_FIFO_EN_ALL) != MPU6050_Result_Ok)
    {
      return MPU6050_Result_Error;
    }
  }

  return MPU6050_Result_Ok;
}
