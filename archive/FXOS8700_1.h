/*
 * Copyright (c) 2015 - 2016, Freescale Semiconductor, Inc.
 * Copyright 2016-2017 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/*! File: FXOS8700_1.h
* @brief The \b FXOS8700_1.h file describes the fxos8700 driver interface and structures.
*/

#ifndef FXOS8700_1_H_
#define FXOS8700_1_H_

#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>
#include "fxos8700_drv.h"
#include "Driver_I2C.h"
/*******************************************************************************
 * Definitions
 ******************************************************************************/
extern fxos8700_i2c_sensorhandle_t Sensor_FXOS8700_1;

/*******************************************************************************
 * API
 ******************************************************************************/


/*! @brief       The interface function to get the capability of the sensor.
 *  @details     TBD
 *
 *  @return      ::FXOS8700_1_GetCapabilities() returns the status .
 *
 *  @Constraints None
 *
 *  @Reentrant   Yes
 */
int32_t FXOS8700_1_GetCapabilities(void);


/*! @brief       The interface function to initialize the sensor.
 *  @details     This function initialize the sensor and sensor handle.
 *
 *
 *  @param[in]   pBus           pointer to the CMSIS API compatible I2C bus object.
 *  @param[in]   sAddress       slave address of the device on the bus.
 *
 *  @return      ::FXOS8700_1_Initialize() returns the status .
 *
 *  @Constraints None
 *
 *  @Reentrant   Yes
 */
int32_t  FXOS8700_1_Initialize(ARM_DRIVER_I2C* pBus, uint16_t sAddress);


/*! @brief       The interface function to configure he sensor.
 *  @details     This function configure the sensor with requested ODR, Range and registers in the regsiter pair array.
 *
 *  @param[in]   pRegWriteList  pointer to the list of device registers and values to write during configuration.
 *
 *  @return      ::FXOS8700_1_Configure() returns the status .
 *
 *  @Constraints None
 *
 *  @Reentrant   Yes
 */
int32_t  FXOS8700_1_Configure(const registerwritelist_t *pRegWriteList);


/*! @brief       The interface function to start the sensor.
 *  @details     This function made sensor into active mode and ready to read the data.
 *
 *  @return      ::FXOS8700_1_StartData() returns the status.
 *
 *  @Constraints None
 *
 *  @Reentrant   Yes
 */
int32_t  FXOS8700_1_StartData(void);


/*! @brief       The interface function to read the sensor data.
 *  @details     This function read the sensor data out from the device and returns raw data in a byte stream.
 *
 *  @param[in]   pReadList      pointer to a list of device registers to read.
 *  @param[out]  pBuffer        buffer which holds the raw sensor data. This buffer may be back to back databuffer based command read in the list.
 *
 *  @return      ::FXOS8700_1_ReadData() returns the status .
 *
 *  @Constraints None
 *
 *  @Reentrant   Yes
 */
int32_t  FXOS8700_1_ReadData(const registerreadlist_t *pReadList, uint8_t *pBuffer );


/*! @brief       The interface function to stop the sensor..
 *  @details     This function made sensor into standby mode and in a power safe state.
 *
 *  @return      ::fxos8700_EndData() returns the status .
 *
 *  @Constraints None
 *
 *  @Reentrant   Yes
 */
int32_t  FXOS8700_1_EndData(void );


/*! @brief       The interface function to calibrate the sensor data.
 *  @details     TBD.

 *  @param[in]   pCalData           pointer to the calibration data.
 *
 *  @return      ::FXOS8700_1_Calibrate() returns the status .
 *
 *  @Constraints None
 *
 *  @Reentrant   Yes
 */
int32_t  FXOS8700_1_Calibrate(mma845x_i2c_caldata_t *pCalData );


/*! @brief       The interface function to shutdown the sensor.
 *  @details     TBD.
 *
 *  @return      ::FXOS8700_1_Shutdown() returns the status .
 *
 *  @Constraints None
 *
 *  @Reentrant   Yes
 */
int32_t  FXOS8700_1_Shutdown(void );


/*! @brief       The interface function to control the power comsumption behavior.
 *  @details     TBD.

 *  @param[in]   pPowerState        power state.
 *
 *  @return      ::FXOS8700_1_PowerControl() returns the status .
 *
 *  @Constraints None
 *
 *  @Reentrant   Yes
 */
int32_t  FXOS8700_1_PowerControl(void *pPowerState);


/*! @brief       The interface function to get the status
 *  @details     TBD.
 *
 *  @return      ::FXOS8700_1_GetStatus() returns the status .
 *
 *  @Constraints None
 *
 *  @Reentrant   Yes
 */
int32_t FXOS8700_1_GetStatus(void);

#endif //FXOS8700_1_H_
