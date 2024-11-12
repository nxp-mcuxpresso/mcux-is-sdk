/*
* Copyright (c) 2015 - 2016, Freescale Semiconductor, Inc.
* All rights reserved.
*
* SPDX-License-Identifier: BSD-3-Clause
*/

/*! \file control.c
    \brief Defines control sub-system

    This file contains a UART implementation of the control subsystem.  The
    command interpreter and streaming functions are contained in two separate
    files.  So you can easily swap those out with only minor changes here.
*/
#include "fsl_debug_console.h"
#include "board.h"
#include "pin_mux.h"
#include "fsl_uart.h"
#include "fsl_port.h"
#include "host_io_uart.h"
#include "anomaly_detection.h"
#include "host_interface_service.h"
#include "host_io_uart.h"
#include "control.h"

/*******************************************************************************
 * Global Variables
 ******************************************************************************/
extern uint8_t gUartRxBuff, gHostRxBuff[HOST_RX_BUF_LEN];
extern volatile bool bUartTxComplete, bUartRxPendingMsg;
extern host_rx_packet_t gHostRxPkt;
extern host_channel_params_t gHostChannelParams[MAX_HOST_STREAMS];
extern host_interface_handle_t gHostHandle;

// direct access to gbls here is the only place in the entire library where we cannot simply
// pass a pointer.  This is because it is needed by the UART interrupt handlers.  Since this
// only occurs here, in a subsystem which is defined to be application dependent, that is
// considered acceptable.
extern Globals gbls;
#define UART_RX_RING_BUFFER_SIZE 64
uint8_t gUartRingbuffer[UART_RX_RING_BUFFER_SIZE] = {0}; /* UART RX ring buffer. */
extern uart_handle_t HOST_S_CMSIS_HANDLE;                /* The uart device handle for configuring Ring buffer. */

void clearCommands(ControlSubsystem *pComm) {
    pComm->CLR=false;
    pComm->Delete=false;
    pComm->DeleteAll=false;
    pComm->Start=false;
    pComm->Stop=false;
 }
/// Initialize the control subsystem and all related hardware
int8_t initializeControlPort(
    ControlSubsystem *pComm  ///< pointer to the control subystem structure
)
{
    ARM_DRIVER_USART *pUartDriver = &HOST_S_DRIVER;
    int status;
    pComm->TrainRun=false;
    pComm->StreamEnable=false;
    if (pComm)
    {
        /*! Initialize the UART driver. */
        status = pUartDriver->Initialize(HOST_S_SIGNAL_EVENT);
        if (ARM_DRIVER_OK != status)
        {
            return -1;
        }
        /*! Set UART Power mode. */
        status = pUartDriver->PowerControl(ARM_POWER_FULL);
        if (ARM_DRIVER_OK != status)
        {
            return -1;
        }
        /*! Set UART Baud Rate. */
        status = pUartDriver->Control(ARM_USART_MODE_ASYNCHRONOUS, 230400);
        if (ARM_DRIVER_OK != status)
        {
            return -1;
        }

        /* UART on K66F does not have H/W FIFOs, as such for reliable UART Interrupt reception with simultaneous SPI we need
        * UART Rx Ring buffer.
        * Since CMSIS drivers do not provide in-built Ring Buffer suport, we need to call Lower SDK APIs do it explicity.
        * Care should be taken to include the appropriate device (uart/lpuart/lpsci) file and call the appropriate SDK API
        * based on the selected device handle. */
        UART_TransferStartRingBuffer(HOST_S_DEVICE_BASE, &HOST_S_CMSIS_HANDLE, gUartRingbuffer, UART_RX_RING_BUFFER_SIZE);


        HOST_Initialize(&gHostHandle, COMM_UART, (void *)pUartDriver, COMM_NONBLOCKING, HOST_SignalEvent_t, NULL);
        bUartTxComplete = true;
        bUartRxPendingMsg = false;
        HOST_Receive(&gHostHandle, &gUartRxBuff, NULL, 1, NULL);

        return(0);
    }
    else
    {
        return (1);
    }
}
