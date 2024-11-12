/*
 * Copyright (c) 2016, Freescale Semiconductor, Inc.
 * Copyright 2016-2017 NXP
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef _is_sdk_sim_H_
#define _is_sdk_sim_H_

/* Standard C Includes */
#include <stdlib.h>

/* SDK Includes */
#include "fsl_lptmr.h"
#include "fsl_uart.h"
#include "fsl_i2c.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define ODR_TMR_BASEADDR LPTMR0 /* The LPTMR Instance to be used. */
#define ODR_TMR_IRQ LPTMR0_IRQn /* The IRQ number for the LPTMR instance. */
#define I2C_SLAVE_BASEADDR I2C0 /* The I2C Instance to be used. */
#define I2C_SLAVE_IRQ I2C0_IRQn /* The IRQ number for the I2C instance. */

#define CONFIG_MSG_TAG 0U       /* TAG for Configure Command from Host */
#define ADD_SAMPLE_MSG_TAG 1U   /* TAG for Add Sensor Sample Command from Host */
#define ACK_RSP_TAG 6U          /* TAG for Response (ASCII ACK symbol has code 6) */
#define NAK_RSP_TAG 15U         /* TAG for Response (ASCII NAK symbol has code 15)*/
#define SAMPLE_FLAG_LEN 1U      /* Size of Sample Read/Unread Flag. */
#define SAMPLE_READ_MARK 0x00   /* Sample Unread Marker. */
#define SAMPLE_UNREAD_MARK 0xFF /* Sample Read Marker. */
#define UART_CMD_EOL_COUNT 2U   /* CR and LF characters */
#define UART_RX_BUF_LEN 8U      /* UART RX MSG Length. */
                                /* The simulator only accepts/expects fixed size NULL padded messages.
                                 * Messages of any other length will lead to indeterminate results. */
#define ODR_MAX_OVERFLOWS 5U    /* This controls stopping the Timer in case no one is reading sensor data. */
#define I2C_SLAVE_RX_BUF_LEN 2U /* Choose this value as per master implementation. */

/* Set the ODR callback as LPTMR IRQ Handler. */
#define ISSDK_Sim_ODR_Callback LPTMR0_IRQHandler

/* Supported Sensors */
#define FXOS8700_I2C_ADDRESS 0x1E  /* The I2C slave address of FXOS8700. */
#define MPL3115_I2C_ADDRESS 0x60   /* The I2C slave address of MPL3115. */
#define FXAS21002_I2C_ADDRESS 0x20 /* The I2C slave address of FXAS21002. */

/*******************************************************************************
 * Types
 ******************************************************************************/
/*! @brief This structure will act as the interface between the sensor spectfic parameters and the simulator core. */
typedef struct
{
    uint8_t slaveAddress;             /* The I2C Slave address for this sensor. */
    uint8_t dataStartAddress;         /* The offset of the first sample. */
    uint8_t dataReadyAddress;         /* The offset of Data ready register. */
    uint16_t sampleDepth;             /* The depth of the Sample Pool. */
    uint16_t maxRegisterIndex;        /* The length of Sensor register map. */
    uint8_t *pRegisterMap;            /* Pointer to Register Map Array. */
    uint8_t *pSamplePool;             /* Pointer to Sensor Sample Pool. */
    char *sensorName;                 /* Part Name of the Sensor. */
    void (*pSensorDataReadyCb)(void); /* Pointer to the sensor's data ready callback function. */
} issdk_sim_sensorparams_t;

/*! @brief This structure will encapsulate all core configuration structures. */
typedef struct
{
    uart_handle_t uartHandle;       /* The UART device handle. */
    i2c_slave_handle_t slaveHandle; /* The I2C device handle. */
    lptmr_config_t lptmrConfig;     /* The LPTMR configuration. */
    i2c_slave_config_t slaveConfig; /* The I2C Slave Mode configuration. */
} issdk_sim_coreparams_t;

/*******************************************************************************
 * APIs
 ******************************************************************************/
/*! -----------------------------------------------------------------------
 *  @brief       The function to get MPL3115 specific parameters.
 *  @details     This function populates sensor specific parameters which are required by the simulator core.
 *  @param[in]   pSensorParams The structure to be populated with sensor parameters.
 *  @return      void There is no return value.
 *  -----------------------------------------------------------------------*/
void ISSDK_Sim_Get_MPL3115_Parameters(issdk_sim_sensorparams_t *pSensorParams);

/*! -----------------------------------------------------------------------
 *  @brief       The function to get sensor specific parameters.
 *  @details     This function populates sensor specific parameters which are required by the simulator core.
 *  @param[in]   pSample Pointer to the sample.
 *  @return      bool Success/Fail status of sample addition.
 *  -----------------------------------------------------------------------*/
bool ISSDK_Sim_Set_MPL3115_Sample(uint8_t *pSample);

/*! -----------------------------------------------------------------------
 *  @brief       The function to get FXOS8700 specific parameters.
 *  @details     This function populates sensor specific parameters which are required by the simulator core.
 *  @param[in]   pSensorParams The structure to be populated with sensor parameters.
 *  @return      void There is no return value.
 *  -----------------------------------------------------------------------*/
void ISSDK_Sim_Get_FXOS8700_Parameters(issdk_sim_sensorparams_t *pSensorParams);

/*! -----------------------------------------------------------------------
 *  @brief       The function to get sensor specific parameters.
 *  @details     This function populates sensor specific parameters which are required by the simulator core.
 *  @param[in]   pSample Pointer to the sample.
 *  @return      bool Success/Fail status of sample addition.
 *  -----------------------------------------------------------------------*/
bool ISSDK_Sim_Set_FXOS8700_Sample(uint8_t *pSample);

/*! -----------------------------------------------------------------------
 *  @brief       The function to get FXAS21002 specific parameters.
 *  @details     This function populates sensor specific parameters which are required by the simulator core.
 *  @param[in]   pSensorParams The structure to be populated with sensor parameters.
 *  @return      void There is no return value.
 *  -----------------------------------------------------------------------*/
void ISSDK_Sim_Get_FXAS21002_Parameters(issdk_sim_sensorparams_t *pSensorParams);

/*! -----------------------------------------------------------------------
 *  @brief       The function to get sensor specific parameters.
 *  @details     This function populates sensor specific parameters which are required by the simulator core.
 *  @param[in]   pSample Pointer to the sample.
 *  @return      bool Success/Fail status of sample addition.
 *  -----------------------------------------------------------------------*/
bool ISSDK_Sim_Set_FXAS21002_Sample(uint8_t *pSample);

/*! -----------------------------------------------------------------------
 *  @brief       The simulator hardware initialization function.
 *  @details     This function initializes the Pins, Clock and UART interfaces.
 *  @param[in]   void There is no input parameter.
 *  @return      void There is no return value.
 *  -----------------------------------------------------------------------*/
void ISSDK_Sim_HW_Init(void);

/*! -----------------------------------------------------------------------
 *  @brief       The simulator I2C Slave Mode initialization function.
 *  @details     This function initializes the I2C interface in Slave Mode and configures slave settings.
 *  @param[in]   void There is no input parameter.
 *  @return      void There is no return value.
 *  -----------------------------------------------------------------------*/
void ISSDK_Sim_I2C_Init(void);

/*! -----------------------------------------------------------------------
 *  @brief       The simulator UART Rx Process and Wait for Events function.
 *  @details     This function processes UART Rx Data and then puts the CPU into Wait Mode.
 *               The CPU will now wake-up when it receives either I2C, LPTMR or an UART interrupt and service it.
 *  @param[in]   void There is no input parameter.
 *  @return      void There is no return value.
 *  -----------------------------------------------------------------------*/
void ISSDK_Sim_ProcessAndWait(void);

#endif /* #ifndef _is_sdk_sim_H_ */
