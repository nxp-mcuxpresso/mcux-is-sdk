/*
* Copyright (c) 2015 - 2016, Freescale Semiconductor, Inc.
* All rights reserved.
*
* SPDX-License-Identifier: BSD-3-Clause
*/

/* FreeRTOS kernel includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "timers.h"
#include "event_groups.h"

// KSDK and ISSDK Headers
#include "fsl_debug_console.h"  // KSDK header file for the debug interface
#include "board.h"              // KSDK header file to define board configuration
#include "pin_mux.h"            // KSDK header file for pin mux initialization functions
#include "clock_config.h"       // KSDK header file for clock configuration
#include "fsl_port.h"           // KSDK header file for Port I/O control
#include "fsl_i2c.h"            // KSDK header file for I2C interfaces
#include "fxas21002.h"          // register address and bit field definitions
#include "fxos8700.h"           // register address and bit field definitions
#include "RTE_Device.h"
#include "fsl_i2c_cmsis.h"
#include "register_io_i2c.h"
#include "fsl_smc.h"

// Anomaly Detection Headers
#include "host_io_uart.h"
#include "anomaly_detection.h"  // top level magCal and sensor fusion interfaces
#include "control.h"  	        // Command/Streaming interface - application specific
#include "status.h"   	        // Sta:tus indicator interface - application specific
#include "drivers.h"  	        // NXP sensor drivers OR customer-supplied drivers
#include "output_stream.h"
#include "machineLearning_subsystem.h"

// Serial Audio Interface (SAI) headers
#include "sai_interrupt.h"

// from process_host_command.c :
bool process_host_command(
    uint8_t tag, uint8_t *hostCommand, uint8_t *hostResponse, size_t *hostMsgSize, size_t respBufferSize);

/// @brief Global data structures for sensor fusion
///
/// These structures are database for ControlSubsystem, StatusSubsystem, PhysicalSensor, and Globals.
/// Globals has the pointers for the other structures.
Globals gbls;                                   ///< This is the primary sensor fusion data structure
ControlSubsystem controlSubsystem;              ///< used for serial communications
StatusSubsystem statusSubsystem;                ///< provides visual (usually LED) status indicator
struct PhysicalSensor sensors[2];               ///< This implementation uses two physical sensors
EventGroupHandle_t event_group = NULL;

registerDeviceInfo_t i2cBusInfo = {
    .deviceInstance     = 0,
    .functionParam      = SMC,
    .idleFunction       = (registeridlefunction_t) SMC_SetPowerModeWait
};

/// @brief Global data structures for anomaly detection toolbox
///
/// In addition to the sensor fusion database, FeatureCollection, ModelCollection, FeatureInstance, ModelInstance, and union Model structures are linked to Globals with their pointers.
struct FeatureCollection featureCollection;             ///< FeatureCollection contains FeatureInstance pointers and control variables.
struct ModelCollection modelCollection;                 ///< ModelCollection contains ModelInstance pointers and control variables.
struct FeatureInstance featureInstance[MAX_FEATURE_INSTANCES];  ///< FeatureInstance contains feature information and control variables.
struct ModelInstance modelInstance[MAX_MODEL_INSTANCES];        ///< ModelInstance contains model information and control variables.
union Model mlModels[MAX_MODEL_INSTANCES];              ///< Union consists of GMM and OC-SVM.

/// @brief Tasks in FreeRTOS
///
/// Anomaly detection toolbox consists of three levels of tasks.
/// read_task() collects raw sensor data.
/// feature_task() computes features from the raw sensor data.
/// ml_task() performs machine learning algorithms.
/// Idle is the priority 0.
static void read_task(void *pvParameters);          ///< FreeRTOS Task definition, priority 1.
static void feature_task(void *pvParameters);       ///< FreeRTOS Task definition, priority 2.
static void ml_task(void *pvParameters);            ///< FreeRTOS Task definition, priority 3.
static void sai_task(void *pvParameters);            ///< FreeRTOS Task definition, priority 4.

/** @brief This is the main function for Anomaly Detection Toolbox (ADT) with machine learning algorithms.
*/
int main(void)
{

    BOARD_InitPins();                   // defined in pin_mux.c, initializes pkg pins
    BOARD_BootClockRUN();               // defined in clock_config.c, initializes clocks
    BOARD_InitDebugConsole();           // defined in board.c, initializes the OpenSDA port

    ARM_DRIVER_I2C* I2Cdrv = &I2C0_Interrupt_Driver;    // defined in the <shield>.h file
    I2Cdrv->Initialize(I2C0_SignalEvent_t);    // Initialize the KSDK driver for the I2C port
    I2Cdrv->PowerControl(ARM_POWER_FULL);      // Set the I2C Power mode.
    I2Cdrv->Control(ARM_I2C_BUS_SPEED, ARM_I2C_BUS_SPEED_FAST);   // Configure the I2C bus speed

    initializeControlPort(&controlSubsystem);                     // configure pins and ports for the control sub-system
    initializeStatusSubsystem(&statusSubsystem);                  // configure pins and ports for the status sub-system

    /* initialize machine learning related global structures */
    initFeatureCollection(&featureCollection, &featureInstance[0]);
    initModelCollection(&modelCollection, &modelInstance[0], &mlModels[0]);
    initGlobals(&gbls, &statusSubsystem, &controlSubsystem, &featureCollection, &modelCollection);  // includes the initialization of sensor fusion structures

    // "install" the sensors we will be using
    gbls.installSensor(&gbls, &sensors[0], BB_FXOS8700_I2C_ADDR, 1, (void*) I2Cdrv, &i2cBusInfo, FXOS8700_Init,  FXOS8700_Read);
    gbls.installSensor(&gbls, &sensors[1], BB_FXAS21002_I2C_ADDR, 1, (void*) I2Cdrv, &i2cBusInfo, FXAS21002_Init, FXAS21002_Read);
    gbls.initializeAD(&gbls);	        // This will initialize sensors and magnetic calibration
    
    // initialize Serial Audio Interface (SAI) for the use of Microphone / Speaker
    initializeSai(&gbls);
    
    event_group = xEventGroupCreate();
    xTaskCreate(read_task, "READ", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 4, NULL);
    xTaskCreate(feature_task, "FUSION", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 2, NULL);
    xTaskCreate(ml_task, "ML", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 1, NULL);
    xTaskCreate(sai_task, "SAI", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 3, NULL);

    gbls.setStatus(&gbls, NORMAL);                // If we got this far, let's set status state to NORMAL
    vTaskStartScheduler();                      // Start the RTOS scheduler
    gbls.setStatus(&gbls, HARD_FAULT);            // If we got this far, FreeRTOS does not have enough memory allocated
    for (;;) ;
}

/** @brief read_task() collects raw sensor data.
*/
static void read_task(void *pvParameters)
{
    uint16_t i=0;                       // general counter variable
    portTickType lastWakeTime;
    const portTickType frequency = 1;   // tick counter runs at the read rate
    lastWakeTime = xTaskGetTickCount();
    while (1)
    {
        for (i=1; i<=OVERSAMPLE_RATE; i++) {
            vTaskDelayUntil(&lastWakeTime, frequency);
            gbls.readSensors(&gbls, i);
            Host_IO_Receive(process_host_command, HOST_FORMAT_HDLC); // Check for incoming commands form Host.
        }
        xEventGroupSetBits(event_group, B0);
    }
}

/** @brief feature_task() computes features from the raw sensor data.
*
*       This function also streams out the anomaly detection restuls.
*/
static void feature_task(void *pvParameters)
{
    uint16_t i=0;  // general counter variable
    while (1)
    {
        xEventGroupWaitBits(event_group,    /* The event group handle. */
                            B0,             /* The bit pattern the event group is waiting for. */
                            pdTRUE,         /* BIT_0 and BIT_4 will be cleared automatically. */
                            pdFALSE,        /* Don't wait for both bits, either bit unblock task. */
                            portMAX_DELAY); /* Block indefinitely to wait for the condition to be met. */

        gbls.runAD(&gbls);                  // Run feature computation
        incrementFeatureBufferIndices(&gbls);
        addToFeatureBuffers(&gbls);
        incrementFeatureBufferIndicesModels(&gbls);
        streamFeatures();                   // Stream input features for enabled model. The stream packet includes the anomaly detection results.

        clearFIFOs(&gbls);                   // This is where the previous statistics are discarded from sensor structures

        gbls.loopcounter++;                 // The loop counter is used to "serialize" mag cal operations
        i=i+1;
        if (i>=4) {                         // Some status codes include a "blink" feature.  This loop
                i=0;                        // should cycle at least four times for that to operate correctly.
                gbls.updateStatus(&gbls);   // This is where pending status updates are made visible
        }
        gbls.queueStatus(&gbls, NORMAL);    // assume NORMAL status for next pass through the loop

        if ((gbls.loopcounter % modelCollection.training_rate) == 0)
            xEventGroupSetBits(event_group, B1);
    }

}

/** @brief ml_task() executes machine learning algorithms for anomaly detection.
*
*       process_ML_comman() controls the machine learning algorithms.
*/
static void ml_task(void *pvParameters)
{
    while (1)
    {
        xEventGroupWaitBits(event_group,    /* The event group handle. */
                            B1,             /* The bit pattern the event group is waiting for. */
                            pdTRUE,         /* BIT_0 and BIT_4 will be cleared automatically. */
                            pdFALSE,        /* Don't wait for both bits, either bit unblock task. */
                            portMAX_DELAY); /* Block indefinitely to wait for the condition to be met. */

        // execution of the control commands received from GUI
        // the commands below can be modified.
        process_ML_command(&gbls);

        xEventGroupSetBits(event_group, B2);
    }
}
    


int idx = 0;
static void sai_task(void *pvParameters)
{
    uint8_t adata[24000];
    uint8_t Rxdata[16];
    while(1)
    {
        xEventGroupWaitBits(event_group,    /* The event group handle. */
                            B2,             /* The bit pattern the event group is waiting for. */
                            pdTRUE,         /* BIT_0 and BIT_4 will be cleared automatically. */
                            pdFALSE,        /* Don't wait for both bits, either bit unblock task. */
                            portMAX_DELAY); /* Block indefinitely to wait for the condition to be met. */
        
        // SAI test
        
        SAI_RxListen(&Rxdata[0]);
        memcpy(&adata[idx],&Rxdata[0],16);
        idx += 16;
        if (idx >= 24000) 
        {
            idx = 0;
//            SAI_RxDisable();
        }
//        SAI_TxPlay();
    }

}

/// \endcode
