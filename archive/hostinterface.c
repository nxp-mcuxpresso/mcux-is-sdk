/*
 * Copyright (c) 2015, Freescale Semiconductor, Inc.
 * Copyright 2016-2017 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "board.h"
#include "fsl_uart.h"

#include "pin_mux.h"
#include "clock_config.h"
#include <string.h>
#include "host_interface_service.h"
#include "data_format_json.h"
#include "Driver_USART.h"


/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
void Host_Blocking(void);
void  Host_NonBlocking(void);
/*******************************************************************************
 * Variables
 ******************************************************************************/
extern ARM_DRIVER_USART Driver_USART_UART0_KSDK_Blocking;
extern ARM_DRIVER_USART Driver_USART_UART0_KSDK_NonBlocking;
/*******************************************************************************
 * Code
 ******************************************************************************/
volatile uint32_t sendComplete = false;
volatile uint32_t recvComplete = false;
#define HOST_NON_BLOCKING 1

void  Host_CallBack(uint32_t event)
{
    switch(event)
    {
      case HOST_INTERFACE_EVENT_SEND_COMPLETE:
        sendComplete = true;
        break;
      case HOST_INTERFACE_EVENT_RECEIVE_COMPLETE:  
        recvComplete = true;
        break;
    }
}
/*!
 * @brief Main function
 */
int main(void)
{
#if HOST_NON_BLOCKING
    Host_NonBlocking();
#else
    Host_Blocking();
#endif   
   
}

void  Host_Blocking(void)
{
    BOARD_InitPins();
    BOARD_BootClockRUN();

    uint8_t recevBuff[50]={0};
    
    uint8_t sendBuff[100]={0};
    //char value [10]= {0};
    
    host_interface_handle_t hostHandle;
    // Initialize the Host
    HOST_Initialize(&hostHandle, COMM_UART, (void*)&Driver_USART_UART0_KSDK_Blocking, COMM_BLOCKING, NULL, NULL);
    // Format the packet for JSON
    JSON_Serialize ((char*)sendBuff, "TimeStamp", "12010", JSON_TYPE_OBJECT, false);
    JSON_Serialize ((char*)sendBuff, "Accel X", "2399", JSON_TYPE_OBJECT, false);
    JSON_Serialize ((char*)sendBuff, "Accel Y", "567", JSON_TYPE_OBJECT, false);
    // Complete the streaming by setting true for the end flag.
    JSON_Serialize ((char*)sendBuff, "Accel Z", "127", JSON_TYPE_OBJECT, true);
    // Send the data to the host
    HOST_Send(&hostHandle, sendBuff, strlen((char*)sendBuff));
              
    
    while(1)
    {
      // wait for host data block
      uint32_t recvSize;
      HOST_Receive(&hostHandle, (uint8_t*)&recevBuff,&recvSize, 1, JSON_BlockDataRead_BlockingCall);
      // send the recev data back in a JSON format
      memset(sendBuff, 0, sizeof(sendBuff));
     
      JSON_Serialize ((char*)sendBuff, "Recv", (char*)recevBuff, JSON_TYPE_OBJECT, true);
      HOST_Send(&hostHandle, sendBuff, strlen((char*)sendBuff));
      
    }  
}

void  Host_NonBlocking(void)
{
  
    BOARD_InitPins();
    BOARD_BootClockRUN();

    uint8_t recevBuff[50]={0};
    
    uint8_t sendBuff[100]={0};
    
    uint8_t state = 0;
    uint8_t buffIndex = 0;
    //char value [10]= {0};
    
    host_interface_handle_t hostHandle;
    // Initialize the Host
    HOST_Initialize(&hostHandle, COMM_UART, (void*)&Driver_USART_UART0_KSDK_NonBlocking, COMM_NONBLOCKING, Host_CallBack, NULL);
    // Format the packet for JSON
    JSON_Serialize ((char*)sendBuff, "TimeStamp", "12010", JSON_TYPE_OBJECT, false);
    JSON_Serialize ((char*)sendBuff, "Accel X", "2399", JSON_TYPE_OBJECT, false);
    JSON_Serialize ((char*)sendBuff, "Accel Y", "567", JSON_TYPE_OBJECT, false);
    // Complete the streaming by setting true for the end flag.
    JSON_Serialize ((char*)sendBuff, "Accel Z", "127", JSON_TYPE_OBJECT, true);
    // Send the data to the host
    HOST_Send(&hostHandle, sendBuff, strlen((char*)sendBuff));
    
    while(!sendComplete);
    sendComplete = false;
      
    while(1)
    {
      // wait for host data block
      uint32_t recvSize;

      uint8_t data;
      HOST_Receive(&hostHandle, (uint8_t*)&data,&recvSize, 1, NULL);
      while(!recvComplete);
      recvComplete = false;   
      if( DATA_FORMAT_JSON_OK == JSON_Get_Stream_NonBlockingCall((uint8_t*)&recevBuff, data, &state, &buffIndex))
      {
          // Received one stream.
                // send the recev data back in a JSON format
        memset(sendBuff, 0, sizeof(sendBuff));
        //sprintf((char*)value, "%d", (uint32_t)recevBuff);       
        JSON_Serialize ((char*)sendBuff, "Recv", (char*)recevBuff, JSON_TYPE_OBJECT, true);
        HOST_Send(&hostHandle, sendBuff, strlen((char*)sendBuff));
        while(!sendComplete);
        sendComplete = false;
        state = 0;
        buffIndex = 0;
      }
      
    } 
}
