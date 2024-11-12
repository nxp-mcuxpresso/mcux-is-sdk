/*
 * Copyright (c) 2016, Freescale Semiconductor, Inc.
 * Copyright 2016-2017 NXP
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/**
 * @file fxos8700_orientation_detection.c
 * @brief The fxos8700_orientation_detection.c file implements the ISSDK FXOS8700 sensor
 *        driver example demonstration for Motion Activated Orientation detection.
 */

//-----------------------------------------------------------------------
// SDK Includes
//-----------------------------------------------------------------------
#include "board.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "fsl_debug_console.h"

//-----------------------------------------------------------------------
// CMSIS Includes
//-----------------------------------------------------------------------
#include "Driver_I2C.h"

//-----------------------------------------------------------------------
// ISSDK Includes
//-----------------------------------------------------------------------
#include "issdk_hal.h"
#include "gpio_driver.h"
#include "fxos8700_drv.h"
#include "systick_utils.h"

//-----------------------------------------------------------------------
// Macros
//-----------------------------------------------------------------------
#define A_FFMT_THS 0x15   /* Threshold Value. */
#define PL_COUNT 0x10     /* Threshold Value. */
#define ASLP_COUNTER 0x07 /* Auto Sleep after ~5s. */

//-----------------------------------------------------------------------
// Constants
//-----------------------------------------------------------------------
/*! @brief Register Start Motion Detect Mode Register Write List. */
const registerwritelist_t cFxos8700ConfigMotionDetect[] = {
    {FXOS8700_CTRL_REG1, FXOS8700_CTRL_REG1_DR_HYBRID_0P7813_HZ, FXOS8700_CTRL_REG1_DR_MASK},
    {FXOS8700_CTRL_REG2, FXOS8700_CTRL_REG2_SLPE_DISABLE, FXOS8700_CTRL_REG2_SLPE_MASK},
    {FXOS8700_CTRL_REG3,
     FXOS8700_CTRL_REG3_WAKE_LNDPRT_DIS | FXOS8700_CTRL_REG3_IPOL_ACTIVE_HIGH | FXOS8700_CTRL_REG3_PP_OD_PUSH_PULL,
     FXOS8700_CTRL_REG3_WAKE_LNDPRT_MASK | FXOS8700_CTRL_REG3_IPOL_MASK |
         FXOS8700_CTRL_REG3_PP_OD_MASK}, /*! Active Low, Push-Pull */
    {FXOS8700_CTRL_REG4, FXOS8700_CTRL_REG4_INT_EN_FFMT_EN, 0},
    {FXOS8700_CTRL_REG5, FXOS8700_CTRL_REG5_INT_CFG_FFMT_INT1, 0}, /*! INT1 Pin  */
    {FXOS8700_A_FFMT_CFG, FXOS8700_A_FFMT_CFG_OAE_MOTION | FXOS8700_A_FFMT_CFG_ZEFE_RAISE_EVENT |
                              FXOS8700_A_FFMT_CFG_YEFE_RAISE_EVENT | FXOS8700_A_FFMT_CFG_XEFE_RAISE_EVENT,
     FXOS8700_A_FFMT_CFG_OAE_MASK | FXOS8700_A_FFMT_CFG_ZEFE_MASK | FXOS8700_A_FFMT_CFG_YEFE_MASK |
         FXOS8700_A_FFMT_CFG_XEFE_MASK},
    {FXOS8700_A_FFMT_THS, A_FFMT_THS, FXOS8700_A_FFMT_THS_THS_MASK}, /* Threshold */
    {FXOS8700_ASLP_COUNT, 0x00, 0},
    {FXOS8700_PL_COUNT, 0x00, 0}, /* Threshold */
    {FXOS8700_PL_CFG, 0x00, 0},
    {FXOS8700_M_CTRL_REG1, FXOS8700_M_CTRL_REG1_M_HMS_HYBRID_MODE,
     FXOS8700_M_CTRL_REG1_M_HMS_MASK}, /*! Enable the Hybrid Mode. */
    {FXOS8700_M_CTRL_REG2, FXOS8700_M_CTRL_REG2_M_RST_CNT_DISABLE,
     FXOS8700_M_CTRL_REG2_M_RST_CNT_MASK}, /*! Automatic magnetic reset is disabled. */
    __END_WRITE_DATA__};

/*! @brief Register Start Orientation Detect Mode Register Write List. */
const registerwritelist_t cFxos8700ConfigOrientDetect[] = {
    {FXOS8700_CTRL_REG1, FXOS8700_CTRL_REG1_DR_HYBRID_3P125_HZ, FXOS8700_CTRL_REG1_DR_MASK},
    {FXOS8700_CTRL_REG2, FXOS8700_CTRL_REG2_SLPE_EN, FXOS8700_CTRL_REG2_SLPE_MASK},
    {FXOS8700_CTRL_REG3,
     FXOS8700_CTRL_REG3_WAKE_LNDPRT_EN | FXOS8700_CTRL_REG3_IPOL_ACTIVE_HIGH | FXOS8700_CTRL_REG3_PP_OD_PUSH_PULL,
     FXOS8700_CTRL_REG3_WAKE_LNDPRT_MASK | FXOS8700_CTRL_REG3_IPOL_MASK |
         FXOS8700_CTRL_REG3_PP_OD_MASK}, /*! Active Low, Push-Pull */
    {FXOS8700_CTRL_REG4, FXOS8700_CTRL_REG4_INT_EN_LNDPRT_EN | FXOS8700_CTRL_REG4_INT_EN_ASLP_EN, 0},
    {FXOS8700_CTRL_REG5, FXOS8700_CTRL_REG5_INT_CFG_LNDPRT_INT1 | FXOS8700_CTRL_REG5_INT_CFG_ASLP_INT1,
     0}, /*! INT1 Pin  */
    {FXOS8700_A_FFMT_CFG, 0x00, 0},
    {FXOS8700_A_FFMT_THS, 0x00, 0}, /* Threshold */
    {FXOS8700_ASLP_COUNT, ASLP_COUNTER, 0},
    {FXOS8700_PL_COUNT, PL_COUNT, 0}, /* Threshold */
    {FXOS8700_PL_CFG, FXOS8700_PL_CFG_DBCNTM_CLEAR_MODE | FXOS8700_PL_CFG_PL_EN_ENABLE,
     FXOS8700_PL_CFG_PL_EN_MASK | FXOS8700_PL_CFG_DBCNTM_MASK},
    {FXOS8700_M_CTRL_REG1, FXOS8700_M_CTRL_REG1_M_HMS_HYBRID_MODE,
     FXOS8700_M_CTRL_REG1_M_HMS_MASK}, /*! Enable the Hybrid Mode. */
    {FXOS8700_M_CTRL_REG2, FXOS8700_M_CTRL_REG2_M_RST_CNT_DISABLE,
     FXOS8700_M_CTRL_REG2_M_RST_CNT_MASK}, /*! Automatic magnetic reset is disabled. */
    __END_WRITE_DATA__};

/*! @brief Address of FFMT Source Register. */
const registerreadlist_t cFxos8700FFMTSrc[] = {{.readFrom = FXOS8700_A_FFMT_SRC, .numBytes = 1}, __END_READ_DATA__};

/*! @brief Address of INT Source Register. */
const registerreadlist_t cFxos8700INTSrc[] = {{.readFrom = FXOS8700_INT_SOURCE, .numBytes = 1}, __END_READ_DATA__};

/*! @brief Address of PL Status Register. */
const registerreadlist_t cFxos8700OutputPLStatus[] = {{.readFrom = FXOS8700_PL_STATUS, .numBytes = 1},
                                                      __END_READ_DATA__};

//-----------------------------------------------------------------------
// Global Variables
//-----------------------------------------------------------------------
volatile bool gFxos8700EventReady;

//-----------------------------------------------------------------------
// Functions
//-----------------------------------------------------------------------
/*! -----------------------------------------------------------------------
 *  @brief       This is the Sensor Data Ready ISR implementation.
 *  @details     This function sets the flag which indicates if a new sample(s) is available for reading.
 *  @param[in]   pUserData This is a void pointer to the instance of the user specific data structure for the ISR.
 *  @return      void  There is no return value.
 *  @constraints None
 *  @reeentrant  Yes
 *  -----------------------------------------------------------------------*/
void fxos8700_int_callback(void *pUserData)
{ /*! @brief Set flag to indicate Sensor has signalled data ready. */
    gFxos8700EventReady = true;
}

/*! -----------------------------------------------------------------------
 *  @brief       This is the The main function implementation.
 *  @details     This function invokes board initializes routines, then then brings up the sensor and
 *               finally enters an endless loop to continuously read available samples.
 *  @param[in]   void This is no input parameter.
 *  @return      void  There is no return value.
 *  @constraints None
 *  @reeentrant  No
 *  -----------------------------------------------------------------------*/
int main(void)
{
    int32_t status;
    bool motionDetect;
    uint8_t orientStatus;

    ARM_DRIVER_I2C *I2Cdrv = &I2C_S_DRIVER; // Now using the shield.h value!!!
    fxos8700_i2c_sensorhandle_t fxos8700Driver;
    GENERIC_DRIVER_GPIO *pGpioDriver = &Driver_GPIO_KSDK;

    /*! Initialize the MCU hardware. */
    BOARD_InitPins();
    BOARD_BootClockRUN();
    BOARD_SystickEnable();
    BOARD_InitDebugConsole();

    PRINTF("\r\n ISSDK FXOS8700 sensor driver example for Orientation and Motion Detection.\r\n");

    /*! Initialize FXOS8700 pin used by FRDM board */
    pGpioDriver->pin_init(&FXOS8700_INT1, GPIO_DIRECTION_IN, NULL, &fxos8700_int_callback, NULL);

    /*! Initialize the I2C driver. */
    status = I2Cdrv->Initialize(I2C_S_SIGNAL_EVENT);
    if (ARM_DRIVER_OK != status)
    {
        PRINTF("\r\n I2C Initialization Failed\r\n");
        return -1;
    }

    /*! Set the I2C Power mode. */
    status = I2Cdrv->PowerControl(ARM_POWER_FULL);
    if (ARM_DRIVER_OK != status)
    {
        PRINTF("\r\n I2C Power Mode setting Failed\r\n");
        return -1;
    }

    /*! Set the I2C bus speed. */
    status = I2Cdrv->Control(ARM_I2C_BUS_SPEED, ARM_I2C_BUS_SPEED_FAST);
    if (ARM_DRIVER_OK != status)
    {
        PRINTF("\r\n I2C Control Mode setting Failed\r\n");
        return -1;
    }

    /*! Initialize FXOS8700 sensor driver. */
    status = FXOS8700_I2C_Initialize(&fxos8700Driver, &I2C_S_DRIVER, I2C_S_DEVICE_INDEX, FXOS8700_I2C_ADDR,
                                     FXOS8700_WHO_AM_I_PROD_VALUE);
    if (SENSOR_ERROR_NONE != status)
    {
        PRINTF("\r\n Sensor Initialization Failed\r\n");
        return -1;
    }
    PRINTF("\r\n Successfully Initiliazed Sensor\r\n");

    /*!  Set the task to be executed while waiting for I2C transactions to complete. */
    FXOS8700_I2C_SetIdleTask(&fxos8700Driver, (registeridlefunction_t)SMC_SetPowerModeWait, SMC);

    for (;;) /* Forever loop for Motion Detection */
    {
        /* Apply FXOS8700 Configuration for Motion Detection. */
        status = FXOS8700_I2C_Configure(&fxos8700Driver, cFxos8700ConfigMotionDetect);
        if (SENSOR_ERROR_NONE != status)
        {
            PRINTF("\r\n Write FXOS8700 Motion Configuration Failed!\r\n");
            return -1;
        }

        motionDetect = true;
        gFxos8700EventReady = false;
        PRINTF("\r\n Waiting for Motion | MCU going to Deep Sleep Mode ...\r\n");

        for (;;) /* Loop for Orientation Detection */
        {        /* In ISR Mode we do not need to check Data Ready Register.
                  * The receipt of interrupt will indicate event is ready. */
            if (false == gFxos8700EventReady)
            { /* Loop, if new sample is not available. */
                SMC_SetPowerModeWait(SMC);
                continue;
            }
            else
            { /*! Clear the data ready flag, it will be set again by the ISR. */
                gFxos8700EventReady = false;
            }

            if (motionDetect)
            { /*! Read the FFMT Status from the FXOS8700. */
                status = FXOS8700_I2C_ReadData(&fxos8700Driver, cFxos8700FFMTSrc, &orientStatus);
                if (ARM_DRIVER_OK != status)
                {
                    PRINTF("\r\n Read FFMT Failed!\r\n");
                    return -1;
                }

                if ((orientStatus & FXOS8700_A_FFMT_SRC_EA_MASK) == 0x80)
                {
                    /*! Display that a Motion event has been detected. */
                    PRINTF("\r\n Motion detected ...\r\n");

                    /* Apply FXOS8700 Configuration for Orientation Detection. */
                    status = FXOS8700_I2C_Configure(&fxos8700Driver, cFxos8700ConfigOrientDetect);
                    if (SENSOR_ERROR_NONE != status)
                    {
                        PRINTF("\r\n Write FXOS8700 Orientation Configuration Failed!\r\n");
                        return -1;
                    }
                    motionDetect = false;
                }
            }
            else
            { /*! Read the Orientation Status from the FXOS8700. */
                status = FXOS8700_I2C_ReadData(&fxos8700Driver, cFxos8700OutputPLStatus, &orientStatus);
                if (ARM_DRIVER_OK != status)
                {
                    PRINTF("\r\n Read Orientation Failed!\r\n");
                    return -1;
                }

                if (((orientStatus & FXOS8700_PL_STATUS_NEWLP_MASK) == 0x80) &&
                    ((orientStatus & FXOS8700_PL_STATUS_LO_MASK) == 0x00))
                {
                    if ((orientStatus & FXOS8700_PL_STATUS_LAPO_MASK) == 0x00)
                    {
                        PRINTF("\r\n Portrait Up ...\r\n");
                        continue;
                    }
                    if ((orientStatus & FXOS8700_PL_STATUS_LAPO_MASK) == 0x02)
                    {
                        PRINTF("\r\n Portrait Down ...\r\n");
                        continue;
                    }
                    if ((orientStatus & FXOS8700_PL_STATUS_LAPO_MASK) == 0x04)
                    {
                        PRINTF("\r\n Landscape Right ...\r\n");
                        continue;
                    }
                    if ((orientStatus & FXOS8700_PL_STATUS_LAPO_MASK) == 0x06)
                    {
                        PRINTF("\r\n Landscape Left ...\r\n");
                        continue;
                    }
                }

                if (((orientStatus & FXOS8700_PL_STATUS_NEWLP_MASK) == 0x80) &&
                    ((orientStatus & FXOS8700_PL_STATUS_LO_MASK) == 0x40))
                {
                    if ((orientStatus & FXOS8700_PL_STATUS_BAFRO_MASK) == 0x00)
                    {
                        PRINTF("\r\n Front Side ...\r\n");
                        continue;
                    }
                    if ((orientStatus & FXOS8700_PL_STATUS_BAFRO_MASK) == 0x01)
                    {
                        PRINTF("\r\n Back Side ...\r\n");
                        continue;
                    }
                }

                status = FXOS8700_I2C_ReadData(&fxos8700Driver, cFxos8700INTSrc, &orientStatus);
                if (ARM_DRIVER_OK != status)
                {
                    PRINTF("\r\n Read INT SRC Failed!\r\n");
                    return -1;
                }

                if ((orientStatus & FXOS8700_INT_SOURCE_SRC_ASLP_MASK) == FXOS8700_INT_SOURCE_SRC_ASLP_MASK)
                {
                    break;
                }
            }
        }
    }
}
