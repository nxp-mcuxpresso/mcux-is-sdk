/*
 * Copyright (c) 2015, Freescale Semiconductor, Inc.
 * Copyright 2016-2017 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/* SDK Includes */
#include "board.h"
#include "fsl_smc.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "fsl_debug_console.h"

/* Local Includes */
#include "issdk_sim.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define ODR_SLAVE_ms 25U /* Selected ODR Rate in ms (integral values only)*/

/*******************************************************************************
 * Global Variables
 ******************************************************************************/
uint8_t gI2cRxBuff[I2C_SLAVE_RX_BUF_LEN];
uint8_t gUartRxBuff[UART_RX_BUF_LEN];
uart_transfer_t gUartXfer = {.data = gUartRxBuff, .dataSize = UART_RX_BUF_LEN + UART_CMD_EOL_COUNT};
issdk_sim_coreparams_t gCoreParams;
issdk_sim_sensorparams_t gSensorParams;
volatile bool bDutInActive = false;
volatile bool bUartCmdPending = false;
volatile bool bTimerCbPending = false;
volatile bool bUartRxProcessed = false;

/*******************************************************************************
 * Functions
 ******************************************************************************/
/*! -----------------------------------------------------------------------
 *  @brief       The I2C slave mode real time user callback.
 *  @details     This function is called automatically on receipt of I2C Events.
 *  @param[in]   base      The pointer to I2C base structure.
 *  @param[in]   xfer      The pointer to the context structure which triggered this callback.
 *  @param[in]   callParam The pointer to usae specific parameters' structure.
 *  @return      void There is no return value.
 *  -----------------------------------------------------------------------*/
static void ISSDK_Sim_I2C_Slave_Callback(I2C_Type *base, i2c_slave_transfer_t *xfer, void *callParam)
{
    uint8_t *registerAddress = (uint8_t *)callParam;
    uint8_t *dataOffset = (uint8_t *)callParam + 1;

    switch (xfer->event)
    { /*  Receive request */
        case kI2C_SlaveReceiveEvent:
            /* This slave is designed to expect multi byte writes (Address+Data+Data+Data...).
             * Here we set the availabe length of the slave's buffer to read.
             * Data will be read until a kI2C_SlaveCompletionEvent is processed to stop it. */
            *registerAddress = 0x00;
            xfer->data = registerAddress;
            xfer->dataSize = I2C_SLAVE_RX_BUF_LEN;
            break;

        /*  Switch to Transmit */
        case kI2C_SlaveRepeatedStartEvent:
            /* Terminate the receive transaction by clearing remaining data count. */
            xfer->dataSize = 0;
            break;

        /*  Transmit request */
        case kI2C_SlaveTransmitEvent:
            /* This slave is designed to expect multi byte reads (Data+Data+Data...).
             * Here we set the availabe length that the master can read (until end of registers).
             * The transaction will be terminated when a kI2C_SlaveCompletionEvent is processed. */
            xfer->dataSize = gSensorParams.maxRegisterIndex - *registerAddress;
            xfer->data = gSensorParams.pRegisterMap + *registerAddress;
            /* Sensor Data Read, now clear Data EVENT register.*/
            if (*registerAddress == gSensorParams.dataStartAddress)
            {
                gSensorParams.pRegisterMap[gSensorParams.dataReadyAddress] = 0x00;
                LED_GREEN_TOGGLE();
            }
            /* Start Data Ready Timer after first query from DUT. */
            if (bDutInActive && (*registerAddress == gSensorParams.dataReadyAddress))
            {
                LPTMR_StartTimer(ODR_TMR_BASEADDR);
                bDutInActive = false;
                LED_BLUE_OFF();
            }
            break;

        /*  Transfer done */
        case kI2C_SlaveCompletionEvent:
            /* Terminate the transaction by clearing remaining data count. */
            xfer->dataSize = 0;
            /* If this is Master writing to slave, copy read data to the appropriate register offset. */
            if (xfer->transferredCount && !(I2C_SlaveGetStatusFlags(base) & kI2C_TransferDirectionFlag))
            {
                memcpy(gSensorParams.pRegisterMap + *registerAddress, dataOffset, xfer->transferredCount - 1);
            }
            break;

        default:
            PRINTF("\r\n[SIM] Error: Un-handled event[%X].\r\n", xfer->event);
            break;
    }
}

/*! -----------------------------------------------------------------------
 *  @brief       The UART user callback.
 *  @details     This function is called automatically on receipt of UART Data.
 *  @param[in]   base     The pointer to UART base structure.
 *  @param[in]   handle   The pointer to UART interface.
 *  @param[in]   status   The event status value which triggered this callback.
 *  @param[in]   userData The pointer to usae specific data structure.
 *  @return      void There is no return value.
 *  -----------------------------------------------------------------------*/
void ISSDK_Sim_UART_Callback(UART_Type *base, uart_handle_t *handle, status_t status, void *userData)
{
    bUartCmdPending = true;
}

/*! -----------------------------------------------------------------------
 *  @brief       The LPTMR expiry callback.
 *  @details     This function is called automatically on expiry of LPTMR period.
 *  @param[in]   void There is no input parameter.
 *  @return      void There is no return value.
 *  -----------------------------------------------------------------------*/
void ISSDK_Sim_ODR_Callback(void)
{
    LPTMR_ClearStatusFlags(ODR_TMR_BASEADDR, kLPTMR_TimerCompareFlag);
    bTimerCbPending = true;
}

/*! -----------------------------------------------------------------------
 *  @brief       The simulator hardware initialization function.
 *  @details     This function initializes the Pins, Clock and UART interfaces.
 *  @param[in]   void There is no input parameter.
 *  @return      void There is no return value.
 *  -----------------------------------------------------------------------*/
void ISSDK_Sim_HW_Init(void)
{
    BOARD_InitPins();
    BOARD_BootClockRUN();

    BOARD_InitDebugConsole();
    UART_TransferCreateHandle((UART_Type *)BOARD_DEBUG_UART_BASEADDR, &gCoreParams.uartHandle, ISSDK_Sim_UART_Callback,
                              NULL);
    UART_TransferReceiveNonBlocking((UART_Type *)BOARD_DEBUG_UART_BASEADDR, &gCoreParams.uartHandle, &gUartXfer, NULL);

    NVIC_SetPriority(ODR_TMR_IRQ, 1);
    LPTMR_GetDefaultConfig(&gCoreParams.lptmrConfig);
    LPTMR_Init(ODR_TMR_BASEADDR, &gCoreParams.lptmrConfig);
    /* Set time period to 1ms less than actual value to compensate for observed LPTMR zero error. */
    LPTMR_SetTimerPeriod(ODR_TMR_BASEADDR, MSEC_TO_COUNT(ODR_SLAVE_ms - 1, CLOCK_GetFreq(kCLOCK_LpoClk)));
    LPTMR_EnableInterrupts(ODR_TMR_BASEADDR, kLPTMR_TimerInterruptEnable);
    EnableIRQ(ODR_TMR_IRQ);

    LED_RED_INIT(LOGIC_LED_ON);
    LED_BLUE_INIT(LOGIC_LED_OFF);
    LED_GREEN_INIT(LOGIC_LED_OFF);
    PRINTF("\r\n[SIM] Hardware Initialization Successful.\r\n");
}

/*! -----------------------------------------------------------------------
 *  @brief       The simulator I2C Slave Mode initialization function.
 *  @details     This function initializes the I2C interface in Slave Mode and configures slave settings.
 *  @param[in]   void There is no input parameter.
 *  @return      void There is no return value.
 *  -----------------------------------------------------------------------*/
void ISSDK_Sim_I2C_Init(void)
{
    NVIC_SetPriority(I2C_SLAVE_IRQ, 2);
    I2C_SlaveGetDefaultConfig(&gCoreParams.slaveConfig);
    gCoreParams.slaveConfig.enableWakeUp = true;
    gCoreParams.slaveConfig.slaveAddress = gSensorParams.slaveAddress;

    I2C_SlaveInit(I2C_SLAVE_BASEADDR, &gCoreParams.slaveConfig);
    I2C_SlaveTransferCreateHandle(I2C_SLAVE_BASEADDR, &gCoreParams.slaveHandle, ISSDK_Sim_I2C_Slave_Callback,
                                  gI2cRxBuff);
    I2C_SlaveTransferNonBlocking(I2C_SLAVE_BASEADDR, &gCoreParams.slaveHandle,
                                 kI2C_SlaveCompletionEvent | kI2C_SlaveRepeatedStartEvent);

    LPTMR_StopTimer(ODR_TMR_BASEADDR);

    LED_RED_OFF();
    LED_BLUE_ON();
    PRINTF("\r\n[SIM] Profile activated for [%s] @%dHz ODR.\r\n", gSensorParams.sensorName, 1000 / ODR_SLAVE_ms);
}

/*! -----------------------------------------------------------------------
 *  @brief       The Offline UART Rx Process Function.
 *  @details     This function is called on receipt of UART Data to process the UART payload.
 *  @param[in]   void There is no input parameter.
 *  @return      void There is no return value.
 *  -----------------------------------------------------------------------*/
void ISSDK_Sim_UART_ProcessRx(void)
{
    /*
    Configure Command Format:
    ------------------------------------------------------
    | TAG | SlaveAddress  | SampleDepth  | Padding | EOL |
    ------------------------------------------------------
    | 1B  |      1B       |     2B       |    4B   |  2B |
    ------------------------------------------------------

    Add Sample Command Format:
    ------------------------------------------------------
    | TAG |   Reserved   | Sample in HEX | Padding | EOL |
    ----------------------------------------------------
    | 1B  |      1B      |     xB        |    yB   |  2B |
    ------------------------------------------------------

    Command Received Ack/Nak Format:
    -------
    | TAG |
    -------
    | 1B  |
    -------
    */

    bool addSampleStatus;
    static uint32_t samplesReceived;

    bUartCmdPending = false;
    bUartRxProcessed = true;

    if (gUartXfer.data[0] == CONFIG_MSG_TAG)
    { /* Set the requested sensor's specific configuration and parameters. */
        samplesReceived = 0;
        gSensorParams.sampleDepth = (uint16_t)gUartXfer.data[2] << 8 | gUartXfer.data[3];
        switch (gUartXfer.data[1])
        {
            case FXOS8700_I2C_ADDRESS:
                LPTMR_StopTimer(ODR_TMR_BASEADDR);
                ISSDK_Sim_Get_FXOS8700_Parameters(&gSensorParams);
                break;
            case FXAS21002_I2C_ADDRESS:
                LPTMR_StopTimer(ODR_TMR_BASEADDR);
                ISSDK_Sim_Get_FXAS21002_Parameters(&gSensorParams);
                break;
            case MPL3115_I2C_ADDRESS:
                LPTMR_StopTimer(ODR_TMR_BASEADDR);
                ISSDK_Sim_Get_MPL3115_Parameters(&gSensorParams);
                break;
            default:
                PUTCHAR(NAK_RSP_TAG); /* Publish NAK to signal command failure. */
                return;
        }
        PUTCHAR(ACK_RSP_TAG); /* Publish ACK to signal command completion. */
        PRINTF("\r\n[SIM] Sample Depth set at [%d].\r\n", gSensorParams.sampleDepth);
        ISSDK_Sim_I2C_Init();
    }

    if (gUartXfer.data[0] == ADD_SAMPLE_MSG_TAG)
    {
        switch (gSensorParams.slaveAddress)
        {
            case FXOS8700_I2C_ADDRESS:
                addSampleStatus = ISSDK_Sim_Set_FXOS8700_Sample(&gUartXfer.data[2]);
                break;
            case FXAS21002_I2C_ADDRESS:
                addSampleStatus = ISSDK_Sim_Set_FXAS21002_Sample(&gUartXfer.data[2]);
                break;
            case MPL3115_I2C_ADDRESS:
                addSampleStatus = ISSDK_Sim_Set_MPL3115_Sample(&gUartXfer.data[2]);
                break;
            default:
                PUTCHAR(NAK_RSP_TAG); /* Publish NAK to signal command failure. */
                return;
        }

        if (addSampleStatus)
        {
            PUTCHAR(ACK_RSP_TAG); /* Publish ACK to signal command completion. */
            bUartCmdPending = false;
            bUartRxProcessed = true;
            samplesReceived++;
        }
        else
        { /* In case sample could not be inserted no ACK is sent.
           * Sim will keep trying repeatedly until there is space.
           * At some point the Host will eventually time out waiting for response. */
            bUartCmdPending = true;
            bUartRxProcessed = false;
            return;
        }

        if (samplesReceived == gSensorParams.sampleDepth / 2)
        { /* Start Sampling TIMER once sample buffer is half full.  */
            bDutInActive = true;
        }
    }
}

/*! -----------------------------------------------------------------------
 *  @brief       The simulator UART Rx Process and Wait for Events function.
 *  @details     This function processes UART Rx Data and then puts the CPU into Wait Mode.
 *               The CPU will then wake-up when it receives either I2C, LPTMR or an UART interrupt and service them.
 *  @param[in]   void There is no input parameter.
 *  @return      void There is no return value.
 *  -----------------------------------------------------------------------*/
void ISSDK_Sim_ProcessAndWait(void)
{
    SMC_SetPowerModeWait(SMC);

    if (bTimerCbPending)
    {
        bTimerCbPending = false;
        gSensorParams.pSensorDataReadyCb(); /* Call the offline function to fetch the next sample. */
    }

    if (bUartCmdPending)
    {
        ISSDK_Sim_UART_ProcessRx();
    }

    if (bUartRxProcessed)
    {
        bUartRxProcessed = false;
        UART_TransferReceiveNonBlocking((UART_Type *)BOARD_DEBUG_UART_BASEADDR, &gCoreParams.uartHandle, &gUartXfer,
                                        NULL);
    }
}
