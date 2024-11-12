/*
 * Copyright (c) 2015, Freescale Semiconductor, Inc.
 * Copyright 2016-2017 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/* Local Includes */
#include "issdk_sim.h"

///////////////////////////////////////////////////////////////////////////////
//  Definitions
///////////////////////////////////////////////////////////////////////////////
#define GYRO_SAMPLE_LEN 6          /* Length of Gyro Sample. */
#define GYRO_SAMPLE_OFFSET 0x01    /* The offset of gyro sample. */
#define FXAS21002_REGISTER_SIZE 22 /* Number of registers in sensor's register map. */

///////////////////////////////////////////////////////////////////////////////
//  Variables
///////////////////////////////////////////////////////////////////////////////
static uint16_t gSampleOffset = 0; /* The offset of the next sample to be sent. */
static uint16_t gSampleIndex = 0;  /* The index of the last sample added by host. */
/* Refer to Chapter 6 of FXAS21002 Data Sheet for default values and individulal register details.
 * Document Number: FXAS21002C */
uint8_t gRegisterMapArray21002[FXAS21002_REGISTER_SIZE] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                                                           0x00, 0x00, 0x00, 0x00, 0xD7, 0x00, 0x00, 0x00,
                                                           0x00, 0x01, 0x00, 0x00, 0x00, 0x00};

///////////////////////////////////////////////////////////////////////////////
//  Constants
///////////////////////////////////////////////////////////////////////////////
static const uint8_t cSensorSampleLen =
    GYRO_SAMPLE_LEN + SAMPLE_FLAG_LEN;         /* The total length of one set of sample in the pool. */
static const uint8_t cDataReadyAddress = 0x00; /* The offset of Status register. */
static const uint8_t cDataReadyValue = 0x0F;   /* The value of Data ready register to indicate new data is available. */
static uint32_t gSamplePoolSize;               /* The size of the sample pool buffer. */
static uint8_t *pSamplePoolBuffer;             /* The pool storing numerous sensor samples. */

///////////////////////////////////////////////////////////////////////////////
//  Functions
///////////////////////////////////////////////////////////////////////////////
/*! -----------------------------------------------------------------------
 *  @brief       The function to be executed when sensor data is ready.
 *               This function is a callback and does not block if a fresh sample is not available.
 *               It will simply send the next sample in the sample pool.
 *  @details     This function copies a new sample from the sample pool to the sensor registers on expiry of the ODR
 * Timer.
 *  @param[in]   void There is no input parameter.
 *  @return      void There is no return value.
 *  -----------------------------------------------------------------------*/
void ISSDK_Sim_FXAS21002_Data_Ready_Callback(void)
{
    memcpy(gRegisterMapArray21002 + GYRO_SAMPLE_OFFSET, pSamplePoolBuffer + gSampleOffset * cSensorSampleLen,
           cSensorSampleLen - SAMPLE_FLAG_LEN);
    pSamplePoolBuffer[gSampleOffset * cSensorSampleLen + cSensorSampleLen - SAMPLE_FLAG_LEN] = SAMPLE_READ_MARK;

    gSampleOffset = /* Update Offset to next sample. */
        (gSampleOffset == gSamplePoolSize / cSensorSampleLen - 1) ? 0 : gSampleOffset + 1;

    /* Sensor data Ready, now set STATUS Register. */
    gRegisterMapArray21002[cDataReadyAddress] = cDataReadyValue;
}

/*! -----------------------------------------------------------------------
 *  @brief       The function to get sensor specific parameters.
 *  @details     This function populates sensor specific parameters which are required by the simulator core.
 *  @param[in]   pSensorParams The structure to be populated with sensor parameters.
 *  @return      void There is no return value.
 *  -----------------------------------------------------------------------*/
void ISSDK_Sim_Get_FXAS21002_Parameters(issdk_sim_sensorparams_t *pSensorParams)
{
    pSensorParams->sensorName = "FXAS21002C";
    pSensorParams->pRegisterMap = gRegisterMapArray21002;
    pSensorParams->slaveAddress = FXAS21002_I2C_ADDRESS;
    pSensorParams->dataStartAddress = GYRO_SAMPLE_OFFSET;
    pSensorParams->dataReadyAddress = cDataReadyAddress;
    pSensorParams->maxRegisterIndex = FXAS21002_REGISTER_SIZE;
    pSensorParams->pSensorDataReadyCb = ISSDK_Sim_FXAS21002_Data_Ready_Callback;

    gSampleIndex = 0;
    gSampleOffset = 0;
    gSamplePoolSize = pSensorParams->sampleDepth * cSensorSampleLen;
    free(pSensorParams->pSamplePool); /* Free the previously allocated buffer(if any). */
    pSensorParams->pSamplePool = pSamplePoolBuffer =
        calloc(gSamplePoolSize, sizeof(uint8_t)); /* Allocate new memory.  */
}

/*! -----------------------------------------------------------------------
 *  @brief       The function to get sensor specific parameters.
 *  @details     This function populates sensor specific parameters which are required by the simulator core.
 *  @param[in]   pSample Pointer to the sample.
 *  @return      bool Success/Fail status of sample addition.
 *  -----------------------------------------------------------------------*/
bool ISSDK_Sim_Set_FXAS21002_Sample(uint8_t *pSample)
{
    uint16_t sampleDepth = gSamplePoolSize / cSensorSampleLen;

    if (pSamplePoolBuffer[gSampleIndex * cSensorSampleLen + cSensorSampleLen - SAMPLE_FLAG_LEN] !=
        SAMPLE_UNREAD_MARK) /* Check if next slot is read. */
    {
        memcpy(pSamplePoolBuffer + gSampleIndex * cSensorSampleLen, pSample, cSensorSampleLen - SAMPLE_FLAG_LEN);
        pSamplePoolBuffer[gSampleIndex * cSensorSampleLen + cSensorSampleLen - SAMPLE_FLAG_LEN] =
            SAMPLE_UNREAD_MARK; /* Mark Slot as unread. */

        /* Update the Index to next slot. */
        gSampleIndex = (gSampleIndex == sampleDepth - 1) ? 0 : (gSampleIndex + 1);
        return true;
    }
    else
    {
        return false;
    }
}
