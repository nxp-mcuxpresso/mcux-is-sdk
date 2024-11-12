/*! *********************************************************************************
 * \addtogroup Wireless UART Application
 * @{
 ********************************************************************************** */
/*!
 * Copyright (c) 2015, Freescale Semiconductor, Inc.
 * Copyright 2016-2017 NXP
 *
 * ile
 *
 * This file is the source file for the ISSDK based Wireless BLE Applications.
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/************************************************************************************
 *************************************************************************************
 * Include
 *************************************************************************************
 ************************************************************************************/
/* Standard C Includes */
#include <stdio.h>
#include <stdarg.h>

/* Framework / Drivers */
#include "RNG_Interface.h"
#include "Keyboard.h"
#include "LED.h"
#include "TimersManager.h"
#include "Panic.h"
#include "SerialManager.h"
#include "MemManager.h"
#include "board.h"

/* BLE Host Stack */
#include "gatt_interface.h"
#include "gatt_server_interface.h"
#include "gatt_client_interface.h"
#include "gatt_database.h"
#include "gap_interface.h"
#include "gatt_db_app_interface.h"
#include "gatt_db_handles.h"

/* Profile / Services */
#include "wireless_uart_interface.h"
#include "battery_interface.h"
/* Wrappers */
#include "ble_conn_manager.h"
#include "ble_service_discovery.h"

#include "board.h"
#include "ApplMain.h"
#include "wireless_uart.h"

/* ISSDK Interface Include */
#include "issdk_ble_interface.h"

/************************************************************************************
 *************************************************************************************
 * Private macros
 *************************************************************************************
 ************************************************************************************/

#define mAppUartBufferSize_c   			gAttMaxWriteDataSize_d(gAttMaxMtu_c) /* Local Buffer Size */


#define mBatteryLevelReportInterval_c   (10)	/* battery level report interval in seconds  */
/************************************************************************************
 *************************************************************************************
 * Private type definitions
 *************************************************************************************
 ************************************************************************************/
typedef enum appEvent_tag{
    mAppEvt_PeerConnected_c,
    mAppEvt_PairingComplete_c,
    mAppEvt_ServiceDiscoveryComplete_c,
    mAppEvt_ServiceDiscoveryFailed_c,
    mAppEvt_GattProcComplete_c,
    mAppEvt_GattProcError_c
}appEvent_t;

typedef enum appState_tag
{
    mAppIdle_c, 
    mAppExchangeMtu_c,	
    mAppServiceDisc_c,
    mAppRunning_c
} appState_t;

typedef struct appPeerInfo_tag
{
    deviceId_t  deviceId;
    bool_t      isBonded;
    wucConfig_t clientInfo;
    appState_t  appState;
} appPeerInfo_t;

typedef struct advState_tag
{
    bool_t advOn;
} advState_t;
/************************************************************************************
 *************************************************************************************
 * Private memory declarations
 *************************************************************************************
 ************************************************************************************/

static appPeerInfo_t mPeerInformation;
static gapRole_t mGapRole;

#if gAppUseBonding_d
static bool_t mRestoringBondedLink = FALSE;
#endif

/* Adv Parmeters */
static advState_t mAdvState;
static bool_t   mScanningOn = FALSE;

static uint16_t mCharMonitoredHandles[1] = { value_uart_stream};

/* Service Data*/
static wusConfig_t mWuServiceConfig;
static basConfig_t mBasServiceConfig = {service_battery, 0};

static tmrTimerID_t mAppTimerId;
static tmrTimerID_t mBatteryMeasurementTimerId;
#ifdef USE_DEBUG_CONSOLE
static uint8_t gAppSerMgrIf;
#endif
static uint16_t mAppUartBufferSize = mAppUartBufferSize_c;
/************************************************************************************
 *************************************************************************************
 * Private functions prototypes
 *************************************************************************************
 ************************************************************************************/

/* Gatt and Att callbacks */
static void BleApp_AdvertisingCallback(gapAdvertisingEvent_t* pAdvertisingEvent);
static void BleApp_ScanningCallback (gapScanningEvent_t* pScanningEvent);
static void BleApp_ConnectionCallback
(
    deviceId_t peerDeviceId,
    gapConnectionEvent_t* pConnectionEvent
);
static void BleApp_GattServerCallback 
(
   deviceId_t deviceId,
   gattServerEvent_t* pServerEvent
);

static void BleApp_GattClientCallback 
(
    deviceId_t              serverDeviceId,
    gattProcedureType_t     procedureType,
    gattProcedureResult_t   procedureResult,
    bleResult_t             error
);

static void BleApp_ServiceDiscoveryCallback
(
    deviceId_t deviceId,
    servDiscEvent_t* pEvent
);

static void BleApp_Config (void);
static void BleApp_Advertise (void);

void BleApp_StateMachineHandler
(
    deviceId_t peerDeviceId,
    uint8_t event
);

static void BleApp_StoreServiceHandles
(
    deviceId_t 	     peerDeviceId,
    gattService_t   *pService
);
static bool_t BleApp_CheckScanEvent (gapScannedDevice_t* pData);

/* Timer Callbacks */
static void ScaningTimerCallback (void *);
static void BatteryMeasurementTimerCallback (void *);
#ifdef USE_DEBUG_CONSOLE
/* Uart Tx Callbacks*/
static void Uart_TxCallBack(void *pBuffer);
#endif
static void BleApp_ReceivedUartStream(uint8_t *pStream, uint16_t streamLength);
/************************************************************************************
*************************************************************************************
* Public functions
*************************************************************************************
************************************************************************************/

/*! *********************************************************************************
* \brief    Initializes application specific functionality before the BLE stack init.
*
********************************************************************************** */
void BleApp_Init(void)
{
    /* Initialize application support for drivers */
    BOARD_InitAdc();
#ifdef USE_DEBUG_CONSOLE
    /* UI */
    SerialManager_Init();

    /* Register Serial Manager interface */
    Serial_InitInterface(&gAppSerMgrIf, APP_SERIAL_INTERFACE_TYPE, APP_SERIAL_INTERFACE_INSTANCE);

    Serial_SetBaudRate(gAppSerMgrIf, gUARTBaudRate115200_c);
#endif
    /* Initialize ISSDK Services */
    ISSDK_Initialize();
}

/*! *********************************************************************************
 * \brief    Starts the BLE application.
 *
 * \param[in]    mGapRole    GAP Start Role (Central or Peripheral).
 ********************************************************************************** */
void BleApp_Start (gapRole_t mGapRole)
{
    switch (mGapRole)
    {
        case gGapCentral_c:
        {
            gPairingParameters.localIoCapabilities = gIoKeyboardDisplay_c;
            App_StartScanning(&gScanParams, BleApp_ScanningCallback, TRUE);
            break;
        }
        case gGapPeripheral_c:
        {
            gPairingParameters.localIoCapabilities = gIoDisplayOnly_c;
            BleApp_Advertise();
            break;
        }
        default:
            break;
    }
}

/*! *********************************************************************************
* \brief        Handles keyboard events.
*
* \param[in]    events    Key event structure.
********************************************************************************** */
void BleApp_HandleKeys(key_event_t events)
{
    switch (events)
    {
        case gKBD_EventPressPB1_c:
        {
            LED_StopFlashingAllLeds();
            Led1Flashing();

            BleApp_Start(mGapRole);
            break;
        }
        case gKBD_EventLongPB1_c:
        {
            if (mPeerInformation.deviceId != gInvalidDeviceId_c)
                Gap_Disconnect(mPeerInformation.deviceId);
            break;
        }
        case gKBD_EventPressPB2_c:
        case gKBD_EventLongPB2_c:
            break;
    default:
        break;
    }
}

/*! *********************************************************************************
* \brief        Handles BLE generic callback.
*
* \param[in]    pGenericEvent    Pointer to gapGenericEvent_t.
********************************************************************************** */
void BleApp_GenericCallback (gapGenericEvent_t* pGenericEvent)
{
    /* Call BLE Conn Manager */
    BleConnManager_GenericEvent(pGenericEvent);
    
    switch (pGenericEvent->eventType)
    {
        case gInitializationComplete_c:    
        {
            BleApp_Config();
        }
        break;    
        
        case gAdvertisingParametersSetupComplete_c:
        {
            App_StartAdvertising(BleApp_AdvertisingCallback, BleApp_ConnectionCallback);
        }
        break;         

        default: 
            break;
    }
}

/************************************************************************************
*************************************************************************************
* Private functions
*************************************************************************************
************************************************************************************/

/*! *********************************************************************************
* \brief        Configures BLE Stack after initialization. Usually used for
*               configuring advertising, scanning, white list, services, et al.
*
********************************************************************************** */
static void BleApp_Config()
{
    /* Configure as GAP Dual Role */
    BleConnManager_GapDualRoleConfig();
        
    /* Register for callbacks */
    App_RegisterGattServerCallback(BleApp_GattServerCallback);
    App_RegisterGattClientProcedureCallback(BleApp_GattClientCallback);
    GattServer_RegisterHandlesForWriteNotifications(NumberOfElements((mCharMonitoredHandles)), mCharMonitoredHandles);
    BleServDisc_RegisterCallback(BleApp_ServiceDiscoveryCallback);
	
    /* By default, always start node as GAP central */
    mAdvState.advOn = FALSE;
    mPeerInformation.appState = mAppIdle_c;
    mScanningOn = FALSE;
    
    mGapRole = gGapCentral_c;

    /* Start services */
    Wus_Start(&mWuServiceConfig);

    mBasServiceConfig.batteryLevel = BOARD_GetBatteryLevel();
    Bas_Start(&mBasServiceConfig);

    /* Allocate application timer */
    mAppTimerId = TMR_AllocateTimer();
    mBatteryMeasurementTimerId = TMR_AllocateTimer();

}

/*! *********************************************************************************
* \brief        Configures GAP Advertise parameters. Advertise will satrt after
*               the parameters are set.
*
********************************************************************************** */
static void BleApp_Advertise(void)
{
    /* Set advertising parameters*/
    Gap_SetAdvertisingParameters(&gAdvParams);
}

/*! *********************************************************************************
 * \brief        Handles BLE Scanning callback from host stack.
 *
 * \param[in]    pScanningEvent    Pointer to gapScanningEvent_t.
 ********************************************************************************** */
static void BleApp_ScanningCallback (gapScanningEvent_t* pScanningEvent)
{
    switch (pScanningEvent->eventType)
    {
        case gDeviceScanned_c:
        {
            if (BleApp_CheckScanEvent(&pScanningEvent->eventData.scannedDevice))
            {        
                gConnReqParams.peerAddressType = pScanningEvent->eventData.scannedDevice.addressType;
                FLib_MemCpy(gConnReqParams.peerAddress, 
                            pScanningEvent->eventData.scannedDevice.aAddress,
                            sizeof(bleDeviceAddress_t));
                
                Gap_StopScanning();
#if gAppUsePrivacy_d
                gConnReqParams.usePeerIdentityAddress = pScanningEvent->eventData.scannedDevice.advertisingAddressResolved;
#endif
                App_Connect(&gConnReqParams, BleApp_ConnectionCallback);
            }
        }        
        break;
        
        case gScanStateChanged_c:
        {
            mScanningOn = !mScanningOn;
            
            /* Node starts scanning */
            if (mScanningOn)
            { 
                /* Start advertising timer */
                TMR_StartLowPowerTimer(mAppTimerId, 
                           gTmrLowPowerSecondTimer_c,
                           TmrSeconds(gScanningTime_c),
                           ScaningTimerCallback, NULL);  

                Led1Flashing();
            }
            /* Node is not scanning */
            else
            {                
                TMR_StopTimer(mAppTimerId);
                
                Led1Flashing();
                Led2Flashing();
                Led3Flashing();
                Led4Flashing();
            }
        }
        break;
    case gScanCommandFailed_c:
    {
        panic(0, 0, 0, 0);
        break;
    }
    default:
        break;
    }
}

/*! *********************************************************************************
* \brief        Handles BLE Advertising callback from host stack.
*
* \param[in]    pAdvertisingEvent    Pointer to gapAdvertisingEvent_t.
********************************************************************************** */
static void BleApp_AdvertisingCallback (gapAdvertisingEvent_t* pAdvertisingEvent)
{
    switch (pAdvertisingEvent->eventType)
    {
        case gAdvertisingStateChanged_c:
        {
            mAdvState.advOn = !mAdvState.advOn;
            LED_StopFlashingAllLeds();
            Led1Flashing();

            if(!mAdvState.advOn)
            {
                Led2Flashing();
                Led3Flashing();
                Led4Flashing();
            }
        }
        break;

        case gAdvertisingCommandFailed_c:
        {
            panic(0,0,0,0);
        }
        break;

        default:
            break;
    }
}

/*! *********************************************************************************
* \brief        Handles BLE Connection callback from host stack.
*
* \param[in]    peerDeviceId        Peer device ID.
* \param[in]    pConnectionEvent    Pointer to gapConnectionEvent_t.
********************************************************************************** */
static void BleApp_ConnectionCallback (deviceId_t peerDeviceId, gapConnectionEvent_t* pConnectionEvent)
{
    switch (pConnectionEvent->eventType)
    {
        case gConnEvtConnected_c:
        {
            mPeerInformation.deviceId = peerDeviceId;

            /* Advertising stops when connected */
            mAdvState.advOn = FALSE;

            /* Subscribe client*/
            Wus_Subscribe(peerDeviceId);
            Bas_Subscribe(peerDeviceId);

            /* UI */
            LED_StopFlashingAllLeds();
            Led1On();

            /* Stop Advertising Timer*/
            mAdvState.advOn = FALSE;
            TMR_StopTimer(mAppTimerId);
            
            /* Start battery measurements */
            TMR_StartLowPowerTimer(mBatteryMeasurementTimerId, gTmrLowPowerIntervalMillisTimer_c,
                       TmrSeconds(mBatteryLevelReportInterval_c), BatteryMeasurementTimerCallback, NULL);

#if gAppUsePairing_d
            
           
#if gAppUseBonding_d            
            Gap_CheckIfBonded(peerDeviceId, &mPeerInformation.isBonded);
            
            if ((mPeerInformation.isBonded) &&
                (gBleSuccess_c == Gap_LoadCustomPeerInformation(peerDeviceId,
                    (void*) &mPeerInformation.clientInfo, 0, sizeof (wucConfig_t))))
            {
                mRestoringBondedLink = TRUE;
                /* Restored custom connection information. Encrypt link */
                Gap_EncryptLink(peerDeviceId);
            }
            else
#endif /* gAppUseBonding_d*/ 
            {
                if (mGapRole == gGapCentral_c)
                {
//                    Gap_Pair(peerDeviceId, &gPairingParameters);
                }
            }
#endif /* gAppUsePairing_d */            
            BleApp_StateMachineHandler(mPeerInformation.deviceId, mAppEvt_PeerConnected_c);
        }
        break;

        case gConnEvtDisconnected_c:
        {
            mPeerInformation.appState = mAppIdle_c;
            mAppUartBufferSize        = mAppUartBufferSize_c;  
            /* Unsubscribe client */
            Wus_Unsubscribe();
            Bas_Unsubscribe(); 
            
            TMR_StopTimer(mBatteryMeasurementTimerId);
            ISSDK_Configure(ISSDK_STANDBY_MODE); /* Stop Sensor on Disconnect */
            /* Reset Service Discovery to be sure*/
            BleServDisc_Stop(peerDeviceId);

            /* UI */
            LED_TurnOffAllLeds();
            LED_StartFlash(LED_ALL);

            BleApp_Start(mGapRole);

        }
        break;

#if gAppUsePairing_d		
        case gConnEvtPairingComplete_c:
        {
            if (pConnectionEvent->eventData.pairingCompleteEvent.pairingSuccessful)
            {
                BleApp_StateMachineHandler(peerDeviceId, mAppEvt_PairingComplete_c);
            }
        }
        break;
#endif /* gAppUsePairing_d */

    default:
        break;
    }
    
    /* Connection Manager to handle Host Stack interactions */
    if (mGapRole == gGapCentral_c)
    {
        BleConnManager_GapCentralEvent(peerDeviceId, pConnectionEvent);
    }
    else if (mGapRole == gGapPeripheral_c)
    {
        BleConnManager_GapPeripheralEvent(peerDeviceId, pConnectionEvent);
    }
}

static void BleApp_ServiceDiscoveryCallback(deviceId_t peerDeviceId, servDiscEvent_t* pEvent)
{
    switch(pEvent->eventType)
    {
        case gServiceDiscovered_c:
        {
            BleApp_StoreServiceHandles(peerDeviceId, pEvent->eventData.pService);
        }
        break;

        case gDiscoveryFinished_c:
        {
            if (pEvent->eventData.success)
            {
                BleApp_StateMachineHandler(peerDeviceId, mAppEvt_ServiceDiscoveryComplete_c);
            }
            else
            {
                BleApp_StateMachineHandler(peerDeviceId, mAppEvt_ServiceDiscoveryFailed_c);
            }
        }
        break;

        default:
        break;
    }
}

/*! *********************************************************************************
* \brief        Handles GATT client callback from host stack.
*
* \param[in]    serverDeviceId      GATT Server device ID.
* \param[in]    procedureType    	Procedure type.
* \param[in]    procedureResult    	Procedure result.
* \param[in]    error    			Callback result.
********************************************************************************** */
static void BleApp_GattClientCallback(
    deviceId_t              serverDeviceId,
    gattProcedureType_t     procedureType,
    gattProcedureResult_t   procedureResult,
    bleResult_t             error
)
{  
    if (procedureResult == gGattProcError_c)
    {    
        BleApp_StateMachineHandler(serverDeviceId, mAppEvt_GattProcError_c);
    }
    else if (procedureResult == gGattProcSuccess_c)
    {        
    	BleApp_StateMachineHandler(serverDeviceId, mAppEvt_GattProcComplete_c);
    }

    /* Signal Service Discovery Module */
    BleServDisc_SignalGattClientEvent(serverDeviceId, procedureType,procedureResult, error);
}

/*! *********************************************************************************
 * \brief        Handles GATT server callback from host stack.
 *
 * \param[in]    deviceId        Client peer device ID.
 * \param[in]    pServerEvent    Pointer to gattServerEvent_t.
 ********************************************************************************** */
static void BleApp_GattServerCallback (
                                       deviceId_t deviceId,
                                       gattServerEvent_t* pServerEvent)
{
    switch (pServerEvent->eventType)
    {
        case gEvtAttributeWrittenWithoutResponse_c:
        {
            if (pServerEvent->eventData.attributeWrittenEvent.handle == value_uart_stream)
            {
            	BleApp_ReceivedUartStream(pServerEvent->eventData.attributeWrittenEvent.aValue,
                            pServerEvent->eventData.attributeWrittenEvent.cValueLength);
            }
            break;
        }
        case gEvtMtuChanged_c:
        {
            /* update stream length with new MTU */
            Gatt_GetMtu(deviceId, &mAppUartBufferSize);
            mAppUartBufferSize = gAttMaxWriteDataSize_d(mAppUartBufferSize);
        }
        break;
    default:
        break;
    }
}


static bool_t MatchDataInAdvElementList (
                                         gapAdStructure_t *pElement,
                                         void *pData,
                                         uint8_t iDataLen)
{
    uint8_t i;

    for (i = 0; i < pElement->length; i += iDataLen)
    {
        if (FLib_MemCmp(pData, &pElement->aData[i], iDataLen))
        {
            return TRUE;
        }
    }
    return FALSE;
}

/*! *********************************************************************************
 * \brief        Checks Scan data for a device to connect.
 *
 * \param[in]    pData    Pointer to gapScannedDevice_t.
 ********************************************************************************** */
static bool_t BleApp_CheckScanEvent (gapScannedDevice_t* pData)
{
    uint8_t index = 0;
    bool_t foundMatch = FALSE;

    while (index < pData->dataLength)
    {
        gapAdStructure_t adElement;

        adElement.length = pData->data[index];
        adElement.adType = (gapAdType_t)pData->data[index + 1];
        adElement.aData = &pData->data[index + 2];

        /* Search for Temperature Custom Service */
        if ((adElement.adType == gAdIncomplete128bitServiceList_c)
            || (adElement.adType == gAdComplete128bitServiceList_c))
        {
            foundMatch = MatchDataInAdvElementList(&adElement,
                &uuid_service_wireless_uart, 16);
        }

        /* Move on to the next AD elemnt type */
        index += adElement.length + sizeof(uint8_t);
    }

    return foundMatch;
}

/*! *********************************************************************************
 * \brief        Stores handles used by the application.
 *
 * \param[in]    pService    Pointer to gattService_t.
 ********************************************************************************** */
static void BleApp_StoreServiceHandles (deviceId_t peerDeviceId, gattService_t *pService)
{
    /* Found Wireless UART Service */
    mPeerInformation.clientInfo.hService = pService->startHandle;

    if (pService->cNumCharacteristics > 0 &&
        pService->aCharacteristics != NULL)
    {
        /* Found Uart Characteristic */
        mPeerInformation.clientInfo.hUartStream =
            pService->aCharacteristics[0].value.handle;
    }
}

static void BleApp_SendUartStream(deviceId_t deviceId, uint8_t *pRecvStream, uint8_t streamSize)
{
    gattCharacteristic_t characteristic;

    characteristic.value.handle = mPeerInformation.clientInfo.hUartStream;

    GattClient_WriteCharacteristicValue(deviceId, &characteristic,
                                        streamSize, pRecvStream, TRUE,
                                        FALSE, FALSE, NULL);
}

void BleApp_StateMachineHandler(deviceId_t peerDeviceId, uint8_t event)
{
    switch (mPeerInformation.appState)
    {
        case mAppIdle_c:
        {
            if (event == mAppEvt_PeerConnected_c)
            {
                /* Let the central device initiate the Exchange MTU procedure*/
                if (mGapRole == gGapCentral_c)
                {
                    /* Moving to Exchange MTU State */
                    mPeerInformation.appState = mAppExchangeMtu_c;
                    GattClient_ExchangeMtu(peerDeviceId);
                }
                else
                {
                    /* Moving to Service Discovery State*/
                    mPeerInformation.appState = mAppServiceDisc_c;

                    /* Start Service Discovery*/
                    BleServDisc_FindService(peerDeviceId, 
                                            gBleUuidType128_c,
                                            (bleUuid_t*) &uuid_service_wireless_uart);                  
                }
            }
        }
        break;

        case mAppExchangeMtu_c:
        {
            if (event == mAppEvt_GattProcComplete_c)
            {
                /* update stream length with new MTU */
                Gatt_GetMtu(peerDeviceId, &mAppUartBufferSize);
                mAppUartBufferSize = gAttMaxWriteDataSize_d(mAppUartBufferSize);
				
                /* Moving to Service Discovery State*/
                mPeerInformation.appState = mAppServiceDisc_c;

                /* Start Service Discovery*/
                BleServDisc_FindService(peerDeviceId, 
                                        gBleUuidType128_c,
                                        (bleUuid_t*) &uuid_service_wireless_uart);
            }
            else if (event == mAppEvt_GattProcError_c) 
            {
                Gap_Disconnect(peerDeviceId);
            }
        }
        break;

        case mAppServiceDisc_c:
        {
            if (event == mAppEvt_ServiceDiscoveryComplete_c)
            {            	
                /* Moving to Running State*/
                mPeerInformation.appState = mAppRunning_c;
            }
            else if (event == mAppEvt_ServiceDiscoveryFailed_c)
            {
                Gap_Disconnect(peerDeviceId);
            }
        }
        break;

        case mAppRunning_c:
                break;
        default:
                break;
    }
}

/*! *********************************************************************************
 * \brief        Handles scanning timer callback.
 *
 * \param[in]    pParam        Calback parameters.
 ********************************************************************************** */
static void ScaningTimerCallback (void * pParam)
{
    /* Stop scanning and start advertising */
    Gap_StopScanning();
    
    mGapRole = gGapPeripheral_c;
    gPairingParameters.localIoCapabilities = gIoDisplayOnly_c;

    BleApp_Advertise();
}

static void BleApp_ReceivedUartStream(uint8_t *pStream, uint16_t streamLength)
{
    bool result;

    /* Fetch Commands in Host String  */
    result = ISSDK_ProcessHostCommand(pStream, streamLength);
    if (false == result)
    { /* Print Error if string does not match any known Command */
        BleApp_WriteToUART("ERROR : Command Not Supported : [%.*s]!\r\n", streamLength - 2, pStream);
    }
}

#ifdef USE_DEBUG_CONSOLE
/*! *********************************************************************************
* \brief        Handles UART Transmit callback.
*
* \param[in]    pData        Parameters.
********************************************************************************** */
static void Uart_TxCallBack(void *pBuffer)
{
    MEM_BufferFree(pBuffer);
}

#endif
/*! *********************************************************************************
* \brief        Handles battery measurement timer callback.
*
* \param[in]    pParam        Calback parameters.
********************************************************************************** */
static void BatteryMeasurementTimerCallback(void * pParam)
{
    mBasServiceConfig.batteryLevel = BOARD_GetBatteryLevel();
    Bas_RecordBatteryMeasurement(mBasServiceConfig.serviceHandle, mBasServiceConfig.batteryLevel);
}

/*! *********************************************************************************
* \brief        Handles writing of messages over BLE.
*
* \param[in]    format        VA_ARG format.
* \param[in]    ...           VA_ARGS.
********************************************************************************** */
void BleApp_WriteToHost(char *format, ...)
{
    va_list ap;
    uint8_t n;
    char *pStr = MEM_BufferAlloc(mAppUartBufferSize_c);

    if (pStr != NULL)
    {
        va_start(ap, format);
        n = vsnprintf(pStr, mAppUartBufferSize_c, format, ap);
        BleApp_SendUartStream(mPeerInformation.deviceId, (uint8_t*)pStr, n);
        MEM_BufferFree(pStr); /* Free Buffer */
    }
}

/*! *********************************************************************************
* \brief        Handles writing of messages to UART.
*
* \param[in]    format        VA_ARG format.
* \param[in]    ...           VA_ARGS.
********************************************************************************** */
void BleApp_WriteToUART(char *format, ...)
{
    va_list ap;
    uint16_t m, n;
    char *pStr = MEM_BufferAlloc(mAppUartBufferSize_c);

    if (pStr != NULL)
    {
        va_start(ap, format);
        m = sprintf(pStr, ISSDK_DEBUG_SUFFIX);
        n = vsnprintf(pStr + m, mAppUartBufferSize_c - m, format, ap);
        if ((0 == strncmp(pStr + m, ISSDK_ERROR_SUFFIX, strlen(ISSDK_ERROR_SUFFIX))) ||
            (0 == strncmp(pStr + m, ISSDK_EVENT_SUFFIX, strlen(ISSDK_EVENT_SUFFIX))))
        {
            BleApp_SendUartStream(mPeerInformation.deviceId, (uint8_t*)pStr + m, n);
#ifdef USE_DEBUG_CONSOLE
            Serial_AsyncWrite(gAppSerMgrIf, (uint8_t*)pStr + m, n, Uart_TxCallBack, (uint8_t*)pStr);
        }
        else
        {
            Serial_AsyncWrite(gAppSerMgrIf, (uint8_t*)pStr, m + n, Uart_TxCallBack, (uint8_t*)pStr);
#else
            MEM_BufferFree(pStr);
#endif
        }
    }
}

/*! *********************************************************************************
 * @}
 ********************************************************************************** */
