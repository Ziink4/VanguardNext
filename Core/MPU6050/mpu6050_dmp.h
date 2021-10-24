/*
 * mpu6050_dmp.h
 *
 *  Created on: 11 juil. 2020
 *      Author: Florian
 */

#ifndef __MPU6050_DMP_H
#define __MPU6050_DMP_H

#define MPU6050_DMP_FEATURE_TAP             0x001
#define MPU6050_DMP_FEATURE_ANDROID_ORIENT  0x002
#define MPU6050_DMP_FEATURE_LP_QUAT         0x004
#define MPU6050_DMP_FEATURE_PEDOMETER       0x008
#define MPU6050_DMP_FEATURE_6X_LP_QUAT      0x010
#define MPU6050_DMP_FEATURE_GYRO_CAL        0x020
#define MPU6050_DMP_FEATURE_SEND_RAW_ACCEL  0x040
#define MPU6050_DMP_FEATURE_SEND_RAW_GYRO   0x080
#define MPU6050_DMP_FEATURE_SEND_CAL_GYRO   0x100

#define MPU6050_DMP_SAMPLING_RATE       1000
#define MPU6050_DMP_BANK_SIZE           256

#define MPU6050_DMP_FIRMWARE_SIZE       3062
#define MPU6050_DMP_FIRMWARE_START_ADDR 0x400
#define MPU6050_DMP_FIRMWARE_LOAD_CHUNK 16

MPU6050_Result_t MPU6050_DMP_WriteMemory(MPU6050_t *DataStruct, uint16_t mem_addr, uint16_t length, const uint8_t *data);

MPU6050_Result_t MPU6050_DMP_ReadMemory(MPU6050_t *DataStruct, uint16_t mem_addr, uint16_t length, uint8_t *data);

MPU6050_Result_t MPU6050_DMP_LoadFirmware(MPU6050_t *DataStruct);

MPU6050_Result_t MPU6050_DMP_ResetFIFO(MPU6050_t *DataStruct, bool dmp_enabled, bool int_enabled, uint8_t fifo_enable);

MPU6050_Result_t MPU6050_DMP_SetState(MPU6050_t *DataStruct, bool enabled);

#endif /* __MPU6050_DMP_H */
