/*
* Copyright (c) 2015 - 2016, Freescale Semiconductor, Inc.
* All rights reserved.
*
* SPDX-License-Identifier: BSD-3-Clause
*/

/*! \file driver_KSDK_NVM.c
    \brief middleware driver for NVM on Kinetis devices
*/
#include "anomaly_detection.h"
#include "driver_KSDK_NVM.h"
#include "fsl_flash.h"
#define ERROR 1
#define SUCCESS 0;
byte NVM_SetBlockFlash(uint8_t *Source, uint32_t Dest, uint16_t Count)
{
    status_t result;
    uint32_t pflashSectorSize = 0;
    flash_config_t flashDriver;                                            /* Flash driver Structure */
    byte retVal=0;

    /* Clean up Flash driver Structure*/
    memset(&flashDriver, 0, sizeof(flash_config_t));
    /* Setup flash driver structure for device and initialize variables. */
    result = FLASH_Init(&flashDriver);
    if (kStatus_FLASH_Success == result)
    {
        FLASH_GetProperty(&flashDriver, FLASH_SECTOR_SIZE_PROPERTY, &pflashSectorSize);
        result = FLASH_Erase(&flashDriver, Dest, pflashSectorSize,  FLASH_ERASE_KEY);

        if (kStatus_FLASH_Success == result) {
            result = FLASH_Program(&flashDriver, Dest, (uint32_t*) Source, Count);
            if (kStatus_FLASH_Success == result) {
              retVal=SUCCESS;
            } else {
              retVal=ERROR;
            }
        } else {
            retVal=ERROR;
        }
    } else {
        retVal=ERROR;
    }

    return(retVal);
}
