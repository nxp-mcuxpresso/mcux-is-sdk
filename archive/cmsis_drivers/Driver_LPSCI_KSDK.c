/*
 * Copyright (c) 2013-2016 ARM Limited. All rights reserved.
 * Copyright (c) 2016, Freescale Semiconductor, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the License); you may
 * not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an AS IS BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "Driver_USART.h"
#include "fsl_lpsci.h"
#include "board.h"

#define ARM_USART_DRV_VERSION ARM_DRIVER_VERSION_MAJOR_MINOR(2, 0) /* driver version */

/* Driver Version */
static const ARM_DRIVER_VERSION DriverVersion = {ARM_USART_API_VERSION, ARM_USART_DRV_VERSION};

enum
{
    UART_INSTANCE_0 = 0,
    UART_INSTANCE_1 = 1,
    UART_INSTANCE_2 = 2,
    UART_INSTANCE_3 = 3,
    TOTAL_NUMBER_INSTANCE
};

#define _UART_INSTANCE_BASE (UART0_Type *) BOARD_DEBUG_UART_BASEADDR
#define _UART_CLKSRC_ BOARD_DEBUG_UART_CLKSRC
#define RX_RING_BUFFER_SIZE 32

/* Driver Capabilities */
static const ARM_USART_CAPABILITIES DriverCapabilities = {
    1, /* supports UART (Asynchronous) mode */
    0, /* supports Synchronous Master mode */
    0, /* supports Synchronous Slave mode */
    0, /* supports UART Single-wire mode */
    0, /* supports UART IrDA mode */
    0, /* supports UART Smart Card mode */
    0, /* Smart Card Clock generator available */
    0, /* RTS Flow Control available */
    0, /* CTS Flow Control available */
    0, /* Transmit completed event: \ref ARM_USART_EVENT_TX_COMPLETE */
    0, /* Signal receive character timeout event: \ref ARM_USART_EVENT_RX_TIMEOUT */
    0, /* RTS Line: 0=not available, 1=available */
    0, /* CTS Line: 0=not available, 1=available */
    0, /* DTR Line: 0=not available, 1=available */
    0, /* DSR Line: 0=not available, 1=available */
    0, /* DCD Line: 0=not available, 1=available */
    0, /* RI Line: 0=not available, 1=available */
    0, /* Signal CTS change event: \ref ARM_USART_EVENT_CTS */
    0, /* Signal DSR change event: \ref ARM_USART_EVENT_DSR */
    0, /* Signal DCD change event: \ref ARM_USART_EVENT_DCD */
    0  /* Signal RI change event: \ref ARM_USART_EVENT_RI */
};
lpsci_handle_t g_uart0Handle;

volatile bool g_uart0_receiveComplete = false;
volatile bool g_uart0_transmitComplete = false;
uint8_t g_uart0_ringbuffer[RX_RING_BUFFER_SIZE] = {0}; /* RX ring buffer. */
ARM_USART_STATUS uart0_status = {0};
ARM_USART_MODEM_STATUS uart0_modem_status = {0};
ARM_USART_SignalEvent_t callBack = NULL;
//
//   Functions
//
/* UART user callback */
void KSDK_UART0_UserCallback(UART0_Type *base, lpsci_handle_t *handle, status_t status, void *userData)
{
    userData = userData;
    uint32_t event;
    if (kStatus_LPSCI_TxIdle == status)
    {
        g_uart0_transmitComplete = true;
        event = ARM_USART_EVENT_SEND_COMPLETE;
    }

    if (kStatus_LPSCI_RxIdle == status)
    {
        g_uart0_receiveComplete = true;
        event = ARM_USART_EVENT_RECEIVE_COMPLETE;
    }
    if (callBack != NULL)
    {
        callBack(event);
    }
}

ARM_DRIVER_VERSION KSDK_UART0_USART_GetVersion(void)
{
    return DriverVersion;
}

ARM_USART_CAPABILITIES KSDK_UART0_USART_GetCapabilities(void)
{
    return DriverCapabilities;
}

int32_t KSDK_UART0_USART_Initialize(ARM_USART_SignalEvent_t cb_event)
{
    lpsci_config_t config;
    LPSCI_GetDefaultConfig(&config);
    config.enableTx = true;
    config.enableRx = true;

    CLOCK_SetLpsci0Clock(1);
    LPSCI_Init(_UART_INSTANCE_BASE, &config, CLOCK_GetFreq(_UART_CLKSRC_));
    LPSCI_TransferCreateHandle(_UART_INSTANCE_BASE, &g_uart0Handle, KSDK_UART0_UserCallback, NULL);
    /* It is essential to use ring buffer to store packets and prevent rx overflow. */
    LPSCI_TransferStartRingBuffer(_UART_INSTANCE_BASE, &g_uart0Handle, g_uart0_ringbuffer, RX_RING_BUFFER_SIZE);
    callBack = cb_event;
    return ARM_DRIVER_OK;
}

int32_t KSDK_UART0_USART_Uninitialize(void)
{
    return ARM_DRIVER_OK;
}

int32_t KSDK_UART0_USART_PowerControl(ARM_POWER_STATE state)
{
    switch (state)
    {
        case ARM_POWER_OFF:
            break;

        case ARM_POWER_LOW:
            break;

        case ARM_POWER_FULL:
            break;

        default:
            return ARM_DRIVER_ERROR_UNSUPPORTED;
    }
    return ARM_DRIVER_OK;
}

int32_t KSDK_UART0_USART_Send(const void *data, uint32_t num)
{
    /* create the data stream structure. */
    lpsci_transfer_t xfer;
    xfer.data = (uint8_t *)data;
    xfer.dataSize = num;
    if (kStatus_Success != LPSCI_TransferSendNonBlocking(_UART_INSTANCE_BASE, &g_uart0Handle, &xfer))
    {
        return ARM_DRIVER_ERROR;
    }
    while (!g_uart0_transmitComplete)
    {
        /* wait for the receive completion */
    }
    g_uart0_transmitComplete = false;
    return ARM_DRIVER_OK;
}
int32_t KSDK_UART0_USART_Send_NonBlocking(const void *data, uint32_t num)
{
    /* create the data stream structure. */
    lpsci_transfer_t xfer;
    xfer.data = (uint8_t *)data;
    xfer.dataSize = num;
    if (kStatus_Success != LPSCI_TransferSendNonBlocking(_UART_INSTANCE_BASE, &g_uart0Handle, &xfer))
    {
        return ARM_DRIVER_ERROR;
    }
    return ARM_DRIVER_OK;
}

int32_t KSDK_UART0_USART_Receive(void *data, uint32_t num)
{
    lpsci_transfer_t recvxfer;
    recvxfer.data = data;
    recvxfer.dataSize = num;
    if (kStatus_Success != LPSCI_TransferReceiveNonBlocking(_UART_INSTANCE_BASE, &g_uart0Handle, &recvxfer, NULL))
    {
        return ARM_DRIVER_ERROR;
    }
    while (!g_uart0_receiveComplete)
    {
        /* wait for the receive completion */
    }
    g_uart0_receiveComplete = false;
    return ARM_DRIVER_OK;
}

int32_t KSDK_UART0_USART_Receive_NonBlocking(void *data, uint32_t num)
{
    lpsci_transfer_t recvxfer;
    recvxfer.data = data;
    recvxfer.dataSize = num;
    if (kStatus_Success != LPSCI_TransferReceiveNonBlocking(_UART_INSTANCE_BASE, &g_uart0Handle, &recvxfer, NULL))
    {
        return ARM_DRIVER_ERROR;
    }
    return ARM_DRIVER_OK;
}
int32_t KSDK_UART0_USART_Transfer(const void *data_out, void *data_in, uint32_t num)
{
    return ARM_DRIVER_OK;
}

uint32_t KSDK_UART0_USART_GetTxCount(void)
{
    return ARM_DRIVER_OK;
}

uint32_t KSDK_UART0_USART_GetRxCount(void)
{
    return ARM_DRIVER_OK;
}

int32_t KSDK_UART0_USART_Control(uint32_t control, uint32_t arg)
{
    return ARM_DRIVER_OK;
}

ARM_USART_STATUS KSDK_UART0_USART_GetStatus(void)
{
    return uart0_status;
}

int32_t KSDK_UART0_USART_SetModemControl(ARM_USART_MODEM_CONTROL control)
{
    return ARM_DRIVER_OK;
}

ARM_USART_MODEM_STATUS KSDK_UART0_USART_GetModemStatus(void)
{
    return uart0_modem_status;
}

void KSDK_UART0_USART_SignalEvent(uint32_t event)
{
    // function body
}

// End USART Interface

ARM_DRIVER_USART Driver_USART_UART0_KSDK_Blocking = {
    KSDK_UART0_USART_GetVersion,      KSDK_UART0_USART_GetCapabilities, KSDK_UART0_USART_Initialize,
    KSDK_UART0_USART_Uninitialize,    KSDK_UART0_USART_PowerControl,    KSDK_UART0_USART_Send,
    KSDK_UART0_USART_Receive,         KSDK_UART0_USART_Transfer,        KSDK_UART0_USART_GetTxCount,
    KSDK_UART0_USART_GetRxCount,      KSDK_UART0_USART_Control,         KSDK_UART0_USART_GetStatus,
    KSDK_UART0_USART_SetModemControl, KSDK_UART0_USART_GetModemStatus};

ARM_DRIVER_USART Driver_USART_UART0_KSDK_NonBlocking = {
    KSDK_UART0_USART_GetVersion,          KSDK_UART0_USART_GetCapabilities, KSDK_UART0_USART_Initialize,
    KSDK_UART0_USART_Uninitialize,        KSDK_UART0_USART_PowerControl,    KSDK_UART0_USART_Send_NonBlocking,
    KSDK_UART0_USART_Receive_NonBlocking, KSDK_UART0_USART_Transfer,        KSDK_UART0_USART_GetTxCount,
    KSDK_UART0_USART_GetRxCount,          KSDK_UART0_USART_Control,         KSDK_UART0_USART_GetStatus,
    KSDK_UART0_USART_SetModemControl,     KSDK_UART0_USART_GetModemStatus};
