/*
 * Copyright (c) 2015-2016, Freescale Semiconductor, Inc.
 * Copyright 2016-2017 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
/*! File: fxls8952_drv.h
* @brief The \b fxls8952_drv.h file describes the fxls8952 driver interface and structures.
*/

#ifndef FXLS8952_1_H_
#define FXLS8952_1_H_
#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>
#include "sensor_drv.h"
#include "fxls8952_drv.h"
#include "Driver_I2C.h"
/*******************************************************************************
 * Definitions
 ******************************************************************************/
extern fxls8952_i2c_sensorHandle_t Sensor_FXLS8952_1;
extern const RegisterReadList_t FXLS8952_ACCEL_DATA[];
extern const RegisterReadList_t FXLS8952_ACCEL_STATUS_DATA[];
/*******************************************************************************
 * API
 ******************************************************************************/
/*! @brief       The interface function to get the capability of the sensor.
 *  @details     TBD

 *  
 *  @return      ::FXLS8952_1_GetCapabilities() returns the status .
 *  
 *  @Constraints None
 *                        
 *  @Reentrant   Yes
 */
int32_t FXLS8952_1_GetCapabilities(void);

/*! @brief       The interface function to initialize the sensor.
 *  @details     This function initialize the sensor and sensor handle. 
 *  
 *  
 *  @return      ::FXLS8952_1_Initialize() returns the status .
 *  
 *  @Constraints None
 *                        
 *  @Reentrant   Yes
 */  
int32_t  FXLS8952_1_Initialize(ARM_DRIVER_I2C* pBus, uint16_t sAddress);
/*! @brief       The interface function to configure he sensor.
 *  @details     This function configure the sensor with requested ODR, Range and registers in the regsiter pair array. 

 *  @param[in]   pRegWriteList      pointer to the register list.
 *  @return      ::FXLS8952_1_Configure() returns the status .
 *  
 *  @Constraints None
 *                        
 *  @Reentrant   Yes
 */  
int32_t  FXLS8952_1_Configure(const RegisterWriteList_t *pRegWriteList);
/*! @brief       The interface function to start the sensor.
 *  @details     This function made sensor into active mode and ready to read the data.
 *  
 *  @return      ::FXLS8952_1_StartData() returns the status.
 *  
 *  @Constraints None
 *                        
 *  @Reentrant   Yes
 */  
int32_t  FXLS8952_1_StartData(void);
/*! @brief       The interface function to read the sensor data.
 *  @details     This function read the sensor data out from the device and returns raw data in a byte stream.

 *  @param[out]  pBuffer            buffer it holds raw sensor data.This buffer may be back to back databuffer based command read in the list.

 *  
 *  @return      ::FXLS8952_1_ReadData() returns the status .
 *  
 *  @Constraints None
 *                        
 *  @Reentrant   Yes
 */
int32_t  FXLS8952_1_ReadData(const RegisterReadList_t *pReadList, uint8_t *pBuffer );
/*! @brief       The interface function to stop the sensor..
 *  @details     This function made sensor into standby mode and in a power safe state.

 *  @param[in]   pSensorHandle      handle to the sensor.
  *  
 *  @return      ::fxls8952_EndData() returns the status .
 *  
 *  @Constraints None
 *                        
 *  @Reentrant   Yes
 */  
int32_t  FXLS8952_1_EndData(void );
/*! @brief       The interface function to calibrate the sensor data.
 *  @details     TBD.

 *  @param[in]   pCalData           pointer to the calibration data.
 *  
 *  @return      ::FXLS8952_1_Calibrate() returns the status .
 *  
 *  @Constraints None
 *                        
 *  @Reentrant   Yes
 */  
int32_t  FXLS8952_1_Calibrate(fxls8952_i2c_calData_t *pCalData );

/*! @brief       The interface function to shutdown the sensor.
 *  @details     TBD.
 *  
 *  @return      ::FXLS8952_1_Shutdown() returns the status .
 *  
 *  @Constraints None
 *                        
 *  @Reentrant   Yes
 */  
int32_t  FXLS8952_1_Shutdown(void );
/*! @brief       The interface function to control the power comsumption behavior.
 *  @details     TBD.

 *  @param[in]   pPowerState        power state.
 *  
 *  @return      ::FXLS8952_1_PowerControl() returns the status .
 *  
 *  @Constraints None
 *                        
 *  @Reentrant   Yes
 */  
int32_t  FXLS8952_1_PowerControl(void *pPowerState);
/*! @brief       The interface function to get the status 
 *  @details     TBD.

 *  
 *  @return      ::FXLS8952_1_GetStatus() returns the status .
 *  
 *  @Constraints None
 *                        
 *  @Reentrant   Yes
 */  
int32_t FXLS8952_1_GetStatus(void);
#endif //FXLS8952_FI_H_