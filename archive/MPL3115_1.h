/*
 * Copyright (c) 2015 - 2016, Freescale Semiconductor, Inc.
 * Copyright 2016-2017 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/*! File: MPL3115_1.h
* @brief The \b MPL3115_1.h file describes the MPL3115 driver interface and structures.
*/

#ifndef MPL3115_1_H_
#define MPL3115_1_H_

#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>
#include "sensor_drv.h"
#include "mpl3115_drv.h"
#include "Driver_I2C.h"
/*******************************************************************************
 * Definitions
 ******************************************************************************/
extern mpl3115_i2c_sensorHandle_t Sensor_MPL3115_1;
extern const RegisterReadList_t MPL3115_DATA_BUFFER[];
/*******************************************************************************
 * API
 ******************************************************************************/


/*! @brief       The interface function to get the capability of the sensor.
 *  @details     TBD

 *
 *  @return      ::MPL3115_1_GetCapabilities() returns the status .
 *
 *  @Constraints None
 *
 *  @Reentrant   Yes
 */
int32_t MPL3115_1_GetCapabilities(void);

/*! @brief       The interface function to initialize the sensor.
 *  @details     This function initialize the sensor and sensor handle.
 *
 *
 *  @param[in]   pBus           pointer to the CMSIS API compatible I2C bus object.
 *  @param[in]   sAddress       slave address of the device on the bus.
 *
 *  @return      ::MPL3115_1_Initialize() returns the status .
 *
 *  @Constraints None
 *
 *  @Reentrant   Yes
 */
int32_t  MPL3115_1_Initialize(ARM_DRIVER_I2C* pBus, uint16_t sAddress);
/*! @brief       The interface function to configure he sensor.
 *  @details     This function configure the sensor with requested ODR, Range and registers in the regsiter pair array.
 *
 *  @param[in]   pRegWriteList  pointer to the list of device registers and values to write during configuration.
 *
 *  @return      ::MPL3115_1_Configure() returns the status .
 *
 *  @Constraints None
 *
 *  @Reentrant   Yes
 */
int32_t  MPL3115_1_Configure(const RegisterWriteList_t *pRegWriteList);


/*! @brief       The interface function to start the sensor.
 *  @details     This function made sensor into active mode and ready to read the data.
 *
 *  @return      ::MPL3115_1_StartData() returns the status.
 *
 *  @Constraints None
 *
 *  @Reentrant   Yes
 */
int32_t  MPL3115_1_StartData(void);


/*! @brief       The interface function to read the sensor data.
 *  @details     This function read the sensor data out from the device and returns raw data in a byte stream.
 *
 *  @param[in]   pReadList      pointer to a list of device registers to read.
 *  @param[out]  pBuffer        buffer which holds the raw sensor data. This buffer may be back to back databuffer based command read in the list.
 *
 *  @return      ::MPL3115_1_ReadData() returns the status .
 *
 *  @Constraints None
 *
 *  @Reentrant   Yes
 */
int32_t  MPL3115_1_ReadData(const RegisterReadList_t *pReadList, uint8_t *pBuffer );


/*! @brief       The interface function to stop the sensor..
 *  @details     This function made sensor into standby mode and in a power safe state.
 *
 *  @return      ::MPL3115_EndData() returns the status .
 *
 *  @Constraints None
 *
 *  @Reentrant   Yes
 */
int32_t  MPL3115_1_EndData(void );


/*! @brief       The interface function to calibrate the sensor data.
 *  @details     TBD.

 *  @param[in]   pCalData           pointer to the calibration data.
 *
 *  @return      ::MPL3115_1_Calibrate() returns the status .
 *
 *  @Constraints None
 *
 *  @Reentrant   Yes
 */
int32_t  MPL3115_1_Calibrate(mpl3115_i2c_calData_t *pCalData );


/*! @brief       The interface function to shutdown the sensor.
 *  @details     TBD.
 *
 *  @return      ::MPL3115_1_Shutdown() returns the status .
 *
 *  @Constraints None
 *
 *  @Reentrant   Yes
 */
int32_t  MPL3115_1_Shutdown(void );


/*! @brief       The interface function to control the power comsumption behavior.
 *  @details     TBD.

 *  @param[in]   pPowerState        power state.
 *
 *  @return      ::MPL3115_1_PowerControl() returns the status .
 *
 *  @Constraints None
 *
 *  @Reentrant   Yes
 */
int32_t  MPL3115_1_PowerControl(void *pPowerState);


/*! @brief       The interface function to get the status
 *  @details     TBD.
 *
 *  @return      ::MPL3115_1_GetStatus() returns the status .
 *
 *  @Constraints None
 *
 *  @Reentrant   Yes
 */
int32_t MPL3115_1_GetStatus(void);

#endif //MPL3115_FI_H_