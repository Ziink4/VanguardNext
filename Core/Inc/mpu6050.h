/**
 * @author  Tilen MAJERLE
 * @email   tilen@majerle.eu
 * @website http://stm32f4-discovery.net
 * @link    http://stm32f4-discovery.net/2015/10/hal-library-30-mpu6050-for-stm32fxxx
 * @version v1.0
 * @ide     Keil uVision
 * @license MIT
 * @brief   MPU6050 library for STM32Fxxx devices
 *
@verbatim
   ----------------------------------------------------------------------
    Copyright (c) 2016 Tilen MAJERLE

    Permission is hereby granted, free of charge, to any person
    obtaining a copy of this software and associated documentation
    files (the "Software"), to deal in the Software without restriction,
    including without limitation the rights to use, copy, modify, merge,
    publish, distribute, sublicense, and/or sell copies of the Software, 
    and to permit persons to whom the Software is furnished to do so, 
    subject to the following conditions:

    The above copyright notice and this permission notice shall be
    included in all copies or substantial portions of the Software.

    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
    EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
    OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE
    AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
    HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
    WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING 
    FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
    OTHER DEALINGS IN THE SOFTWARE.
   ----------------------------------------------------------------------
@endverbatim
 */
#ifndef __MPU6050_H
#define __MPU6050_H

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief    MPU6050 library for STM32Fxxx - http://stm32f4-discovery.net/2015/10/hal-library-30-mpu6050-for-stm32fxxx
 *
 * \par Features
 *
 * Library supports basic operation with MPU6050 device:
 *
\verbatim
- Read accelerometer, gyroscope and temperature data,
- Set custom output data rate for measurements
- Enable/disable interrupts
- Up to 2 MPU devices at a time
\endverbatim
 *
 * \par MPU6050 interrupts
 *
 * When you enable interrupts using @ref MPU6050_EnableInterrupts function,
 * "DataReady" and "MotionDetected" interrupts are enabled. 
 *
 * MPU pin for interrupt detection on STM device is rising edge and triggers on any interrupt.
 *
 * You can read interrupts status register to detect which interrupt happened using @ref MPU6050_ReadInterrupts function.
 *
 * \par MPU6050 data rate
 *
 * Device can output data at specific rate. It has 8-bit register with custom value to set data rate you need.
 *
 * Equation for data rate is below:
\f[
  DataRate =
    \frac{8 MHz}{REGVAL + 1}
\f]
 * where:
 *  - 8 Mhz is Gyro internal output used for data rate
 *  - REGVAL is a value to be used in @ref MPU6050_SetDataRate function
 *
 * \note  There are already some predefined constants in library for some "standard" data rates
 *
 * \par Default pinout
 * 
@verbatim
MPU6050     STM32Fxxx     Descrption
 
SCL         PB6           Clock line for I2C
SDA         PB7           Data line for I2C
IRQ         -             User selectable pin if needed. Interrupts for STM must be manually enabled by user.
VCC         3.3V
GND         GND
AD0         -             If pin is low, I2C address is 0xD0, if pin is high, the address is 0xD2
@endverbatim
 *
 * To change default pinout for I2C, you need to open defines.h file and copy/edit some defines:
 *
\code
//Set I2C used
MPU6050_I2C               I2C1  
//Set I2C pins used
MPU6050_I2C_PINSPACK      TM_I2C_PinsPack_1
\endcode
 *
 * \par Changelog
 *
@verbatim
 Version 1.0
  - First release
@endverbatim
 *
 * \par Dependencies
 *
@verbatim
 - STM32Fxxx HAL
@endverbatim
 */

#include <stdint.h>
#include <stdbool.h>
#include "stm32f0xx_hal.h"

// Default I2C address
#define MPU6050_I2C_ADDR           0xD0
#define MPU6050_I2C_TIMEOUT        1000

// Who I am register value
#define MPU6050_I_AM               0x68

// MPU6050 registers
#define MPU6050_AUX_VDDIO          0x01
#define MPU6050_SMPLRT_DIV         0x19
#define MPU6050_CONFIG             0x1A
#define MPU6050_GYRO_CONFIG        0x1B
#define MPU6050_ACCEL_CONFIG       0x1C
#define MPU6050_MOTION_THRESH      0x1F
#define MPU6050_FIFO_EN            0x23
#define MPU6050_INT_PIN_CFG        0x37
#define MPU6050_INT_ENABLE         0x38
#define MPU6050_INT_STATUS         0x3A
#define MPU6050_ACCEL_XOUT_H       0x3B
#define MPU6050_ACCEL_XOUT_L       0x3C
#define MPU6050_ACCEL_YOUT_H       0x3D
#define MPU6050_ACCEL_YOUT_L       0x3E
#define MPU6050_ACCEL_ZOUT_H       0x3F
#define MPU6050_ACCEL_ZOUT_L       0x40
#define MPU6050_TEMP_OUT_H         0x41
#define MPU6050_TEMP_OUT_L         0x42
#define MPU6050_GYRO_XOUT_H        0x43
#define MPU6050_GYRO_XOUT_L        0x44
#define MPU6050_GYRO_YOUT_H        0x45
#define MPU6050_GYRO_YOUT_L        0x46
#define MPU6050_GYRO_ZOUT_H        0x47
#define MPU6050_GYRO_ZOUT_L        0x48
#define MPU6050_MOT_DETECT_STATUS  0x61
#define MPU6050_SIGNAL_PATH_RESET  0x68
#define MPU6050_MOT_DETECT_CTRL    0x69
#define MPU6050_USER_CTRL          0x6A
#define MPU6050_PWR_MGMT_1         0x6B
#define MPU6050_PWR_MGMT_2         0x6C
#define MPU6050_BANK_SEL           0x6D
#define MPU6050_MEM_START_ADDR     0x6E
#define MPU6050_MEM_R_W            0x6F
#define MPU6050_PRGM_START_H       0x70
#define MPU6050_FIFO_COUNTH        0x72
#define MPU6050_FIFO_COUNTL        0x73
#define MPU6050_FIFO_R_W           0x74
#define MPU6050_WHO_AM_I           0x75

// MPU6050_USER_CTRL fields
#define MPU6050_USER_CTRL_FIFO_EN  0x40
#define MPU6050_USER_CTRL_DMP_EN   0x80
#define MPU6050_USER_CTRL_FIFO_RST 0x04
#define MPU6050_USER_CTRL_DMP_RST  0x08

// MPU6050_INT_ENABLE fields
#define MPU6050_INT_EN_DATA_RDY    0x01
#define MPU6050_INT_EN_DMP_INT     0x02
#define MPU6050_INT_EN_FIFO_OVERFLOW 0x10

// MPU6050_FIFO_EN fields
#define MPU6050_FIFO_EN_TEMP        0x80
#define MPU6050_FIFO_EN_X_GYRO      0x40
#define MPU6050_FIFO_EN_Y_GYRO      0x20
#define MPU6050_FIFO_EN_Z_GYRO      0x10
#define MPU6050_FIFO_EN_XYZ_GYRO    MPU6050_FIFO_EN_X_GYRO | MPU6050_FIFO_EN_Y_GYRO | MPU6050_FIFO_EN_Z_GYRO
#define MPU6050_FIFO_EN_XYZ_ACCEL   0x08
#define MPU6050_FIFO_EN_ALL         MPU6050_FIFO_EN_TEMP | MPU6050_FIFO_EN_XYZ_GYRO | MPU6050_FIFO_EN_XYZ_ACCEL

// Gyro sensitivities in degrees/s
#define MPU6050_GYRO_SENS_250      ((float) 131)
#define MPU6050_GYRO_SENS_500      ((float) 65.5)
#define MPU6050_GYRO_SENS_1000     ((float) 32.8)
#define MPU6050_GYRO_SENS_2000     ((float) 16.4)

// Acce sensitivities in g/s
#define MPU6050_ACCE_SENS_2        ((float) 16384)
#define MPU6050_ACCE_SENS_4        ((float) 8192)
#define MPU6050_ACCE_SENS_8        ((float) 4096)
#define MPU6050_ACCE_SENS_16       ((float) 2048)

/**
 * @brief  Data rates predefined constants
 */
#define MPU6050_DataRate_8KHz       0   //!< Sample rate set to 8 kHz
#define MPU6050_DataRate_4KHz       1   //!< Sample rate set to 4 kHz
#define MPU6050_DataRate_2KHz       3   //!< Sample rate set to 2 kHz
#define MPU6050_DataRate_1KHz       7   //!< Sample rate set to 1 kHz
#define MPU6050_DataRate_500Hz      15  //!< Sample rate set to 500 Hz
#define MPU6050_DataRate_250Hz      31  //!< Sample rate set to 250 Hz
#define MPU6050_DataRate_125Hz      63  //!< Sample rate set to 125 Hz
#define MPU6050_DataRate_100Hz      79  //!< Sample rate set to 100 Hz

/**
 * @brief  MPU6050 can have 2 different slave addresses, depends on it's input AD0 pin
 *         This feature allows you to use 2 different sensors with this library at the same time
 */
typedef enum _MPU6050_Device_t
{
  MPU6050_Device_0 = 0x00, //!< AD0 pin is set to low
  MPU6050_Device_1 = 0x02  //!< AD0 pin is set to high
} MPU6050_Device_t;

/**
 * @brief  MPU6050 result enumeration
 */
typedef enum _MPU6050_Result_t
{
  MPU6050_Result_Ok = 0x00,          //!< Everything OK
  MPU6050_Result_Error,              //!< Unknown error
  MPU6050_Result_DeviceNotConnected, //!< There is no device with valid slave address
  MPU6050_Result_DeviceInvalid       //!< Connected device with address is not MPU6050
} MPU6050_Result_t;

/**
 * @brief  Parameters for accelerometer range
 */
typedef enum _MPU6050_Accelerometer_t
{
  MPU6050_Accelerometer_2G = 0x00, //!< Range is +- 2G
  MPU6050_Accelerometer_4G = 0x01, //!< Range is +- 4G
  MPU6050_Accelerometer_8G = 0x02, //!< Range is +- 8G
  MPU6050_Accelerometer_16G = 0x03 //!< Range is +- 16G
} MPU6050_Accelerometer_t;

/**
 * @brief  Parameters for gyroscope range
 */
typedef enum _MPU6050_Gyroscope_t
{
  MPU6050_Gyroscope_250s = 0x00,  //!< Range is +- 250 degrees/s
  MPU6050_Gyroscope_500s = 0x01,  //!< Range is +- 500 degrees/s
  MPU6050_Gyroscope_1000s = 0x02, //!< Range is +- 1000 degrees/s
  MPU6050_Gyroscope_2000s = 0x03  //!< Range is +- 2000 degrees/s
} MPU6050_Gyroscope_t;

/**
 * @brief  Main MPU6050 structure
 */
typedef struct _MPU6050_t
{
  // Private
  I2C_HandleTypeDef *Handle; //!< I2C handle. Only for private use
  uint8_t Address;           //!< I2C address of device. Only for private use
  float Gyro_Mult;           //!< Gyroscope corrector from raw data to "degrees/s". Only for private use
  float Acce_Mult;           //!< Accelerometer corrector from raw data to "g". Only for private use
  // Public
  int16_t Accelerometer_X;   //!< Accelerometer value X axis
  int16_t Accelerometer_Y;   //!< Accelerometer value Y axis
  int16_t Accelerometer_Z;   //!< Accelerometer value Z axis
  int16_t Gyroscope_X;       //!< Gyroscope value X axis
  int16_t Gyroscope_Y;       //!< Gyroscope value Y axis
  int16_t Gyroscope_Z;       //!< Gyroscope value Z axis
  float Temperature;         //!< Temperature in degrees
} MPU6050_t;

/**
 * @brief  Interrupts union and structure
 */
typedef union _MPU6050_Interrupt_t
{
  struct
  {
    uint8_t DataReady:1;       //!< Data ready interrupt
    uint8_t reserved2:2;       //!< Reserved bits
    uint8_t Master:1;          //!< Master interrupt. Not enabled with library
    uint8_t FifoOverflow:1;    //!< FIFO overflow interrupt. Not enabled with library
    uint8_t reserved1:1;       //!< Reserved bit
    uint8_t MotionDetection:1; //!< Motion detected interrupt
    uint8_t reserved0:1;       //!< Reserved bit
  } F;
  uint8_t Status;
} MPU6050_Interrupt_t;

/**
 * @brief    Library Functions
 */

/**
 * @brief  Initializes MPU6050 and I2C peripheral
 * @param  *DataStruct: Pointer to empty @ref MPU6050_t structure
 * @param  DeviceNumber: MPU6050 has one pin, AD0 which can be used to set address of device.
 *          This feature allows you to use 2 different sensors on the same board with same library.
 *          If you set AD0 pin to low, then this parameter should be MPU6050_Device_0,
 *          but if AD0 pin is high, then you should use MPU6050_Device_1
 *          
 *          Parameter can be a value of @ref MPU6050_Device_t enumeration
 * @param  AccelerometerSensitivity: Set accelerometer sensitivity. This parameter can be a value of @ref MPU6050_Accelerometer_t enumeration
 * @param  GyroscopeSensitivity: Set gyroscope sensitivity. This parameter can be a value of @ref MPU6050_Gyroscope_t enumeration
 * @retval Initialization status:
 *            - MPU6050_Result_t: Everything OK
 *            - Other member: in other cases
 */
MPU6050_Result_t MPU6050_Init(MPU6050_t *DataStruct, I2C_HandleTypeDef *hi2c, MPU6050_Device_t DeviceNumber, MPU6050_Accelerometer_t AccelerometerSensitivity, MPU6050_Gyroscope_t GyroscopeSensitivity);

/**
 * @brief  Sets gyroscope sensitivity
 * @param  *DataStruct: Pointer to @ref MPU6050_t structure indicating MPU6050 device
 * @param  GyroscopeSensitivity: Gyro sensitivity value. This parameter can be a value of @ref MPU6050_Gyroscope_t enumeration
 * @retval Member of @ref MPU6050_Result_t enumeration
 */
MPU6050_Result_t MPU6050_SetGyroscope(MPU6050_t *DataStruct, MPU6050_Gyroscope_t GyroscopeSensitivity);

/**
 * @brief  Sets accelerometer sensitivity
 * @param  *DataStruct: Pointer to @ref MPU6050_t structure indicating MPU6050 device
 * @param  AccelerometerSensitivity: Gyro sensitivity value. This parameter can be a value of @ref MPU6050_Accelerometer_t enumeration
 * @retval Member of @ref MPU6050_Result_t enumeration
 */
MPU6050_Result_t MPU6050_SetAccelerometer(MPU6050_t *DataStruct, MPU6050_Accelerometer_t AccelerometerSensitivity);

/**
 * @brief  Sets output data rate
 * @param  *DataStruct: Pointer to @ref MPU6050_t structure indicating MPU6050 device
 * @param  rate: Data rate value. An 8-bit value for prescaler value
 * @retval Member of @ref MPU6050_Result_t enumeration
 */
MPU6050_Result_t MPU6050_SetDataRate(MPU6050_t *DataStruct, uint8_t rate);

/**
 * @brief  Enables interrupts
 * @param  *DataStruct: Pointer to @ref MPU6050_t structure indicating MPU6050 device
 * @retval Member of @ref MPU6050_Result_t enumeration
 */
MPU6050_Result_t MPU6050_EnableInterrupts(MPU6050_t *DataStruct, bool dmp_enabled);

/**
 * @brief  Disables interrupts
 * @param  *DataStruct: Pointer to @ref MPU6050_t structure indicating MPU6050 device
 * @retval Member of @ref MPU6050_Result_t enumeration
 */
MPU6050_Result_t MPU6050_DisableInterrupts(MPU6050_t *DataStruct);

/**
 * @brief  Reads and clears interrupts
 * @param  *DataStruct: Pointer to @ref MPU6050_t structure indicating MPU6050 device
 * @param  *InterruptsStruct: Pointer to @ref MPU6050_Interrupt_t structure to store status in
 * @retval Member of @ref MPU6050_Result_t enumeration
 */
MPU6050_Result_t MPU6050_ReadInterrupts(MPU6050_t *DataStruct, MPU6050_Interrupt_t *InterruptsStruct);

/**
 * @brief  Reads accelerometer data from sensor
 * @param  *DataStruct: Pointer to @ref MPU6050_t structure to store data to
 * @retval Member of @ref MPU6050_Result_t:
 *            - MPU6050_Result_Ok: everything is OK
 *            - Other: in other cases
 */
MPU6050_Result_t MPU6050_ReadAccelerometer(MPU6050_t *DataStruct);

/**
 * @brief  Reads gyroscope data from sensor
 * @param  *DataStruct: Pointer to @ref MPU6050_t structure to store data to
 * @retval Member of @ref MPU6050_Result_t:
 *            - MPU6050_Result_Ok: everything is OK
 *            - Other: in other cases
 */
MPU6050_Result_t MPU6050_ReadGyroscope(MPU6050_t *DataStruct);

/**
 * @brief  Reads temperature data from sensor
 * @param  *DataStruct: Pointer to @ref MPU6050_t structure to store data to
 * @retval Member of @ref MPU6050_Result_t:
 *            - MPU6050_Result_Ok: everything is OK
 *            - Other: in other cases
 */
MPU6050_Result_t MPU6050_ReadTemperature(MPU6050_t *DataStruct);

/**
 * @brief  Reads accelerometer, gyroscope and temperature data from sensor
 * @param  *DataStruct: Pointer to @ref MPU6050_t structure to store data to
 * @retval Member of @ref MPU6050_Result_t:
 *            - MPU6050_Result_Ok: everything is OK
 *            - Other: in other cases
 */
MPU6050_Result_t MPU6050_ReadAll(MPU6050_t *DataStruct);

#ifdef __cplusplus
}
#endif

#endif // __MPU6050_H
