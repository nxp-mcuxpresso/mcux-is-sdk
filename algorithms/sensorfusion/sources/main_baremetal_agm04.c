/*
 * Copyright (c) 2016, Freescale Semiconductor, Inc.
 * Copyright 2016-2017 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/*! \file main_baremetal_agm04.c
    \brief Bare metal implementation of sensor fusion on FRDM-K64F/FRDM-STBC-AGM04
	       using MMA8652 (Accel), FXAS21002 (Gyro) and MAG3110 (Mag).

    \note This file shows one recommended way to incorporate sensor fusion capabilities
    into a bare metal project.
*/

// SDK and ISSDK Headers
#include "fsl_debug_console.h"  // SDK header file for the debug interface
#include "board.h"              // SDK header file to define board configuration
#include "pin_mux.h"            // SDK header file for pin mux initialization functions
#include "clock_config.h"       // SDK header file for clock configuration
#include "fsl_port.h"           // SDK header file for Port I/O control
#include "fsl_i2c.h"            // SDK header file for I2C interfaces
#include "fsl_i2c_cmsis.h"
#include "fsl_dspi_cmsis.h"
#include "register_io_i2c.h"
#include "fsl_pit.h"            // SDK header for the Periodic Interval Timer

// Sensor Fusion Headers
#include "sensor_fusion.h"      // top level magCal and sensor fusion interfaces
#include "control.h"  	        // Command/Streaming interface - application specific
#include "status.h"   	        // Status indicator interface - application specific
#include "drivers.h"  	        // NXP sensor drivers OR customer-supplied drivers
#include "driver_pit.h"         // Project-specific - PIT is used to control main() timing loop

// Global data structures
SensorFusionGlobals sfg;                ///< This is the primary sensor fusion data structure
ControlSubsystem controlSubsystem;      ///< used for serial communications
StatusSubsystem statusSubsystem;        ///< provides visual (usually LED) status indicator
struct PhysicalSensor sensors[3];              ///< This implementation uses three physical sensors

registerDeviceInfo_t i2cBusInfo = {
    .deviceInstance     = I2C_S_DEVICE_INDEX,
    .functionParam      = NULL,
    .idleFunction       = NULL
};

/// This is a FreeRTOS (dual task) implementation of the NXP sensor fusion demo build.
int main(void)
{
    int i; // loop index
    ARM_DRIVER_I2C* I2Cdrv = &I2C_S_DRIVER;       // defined in the <shield>.h file
    BOARD_InitPins();                   // defined in pin_mux.c, initializes pkg pins
    BOARD_BootClockRUN();               // defined in clock_config.c, initializes clocks
    BOARD_InitDebugConsole();           // defined in board.c, initializes the OpenSDA port

    I2Cdrv->Initialize(I2C_S_SIGNAL_EVENT);           // Initialize the SDK driver for the I2C port
    I2Cdrv->PowerControl(ARM_POWER_FULL);      // Set the I2C Power mode.
    I2Cdrv->Control(ARM_I2C_BUS_SPEED, ARM_I2C_BUS_SPEED_FAST);      // Configure the I2C bus speed

    initializeControlPort(&controlSubsystem);                           // configure pins and ports for the control sub-system
    initializeStatusSubsystem(&statusSubsystem);                        // configure pins and ports for the status sub-system
    initSensorFusionGlobals(&sfg, &statusSubsystem, &controlSubsystem); // Initialize sensor fusion structures
    // "install" the sensors we will be using
#if F_USING_ACCEL
    sfg.installSensor(&sfg, &sensors[0], MMA8652_I2C_ADDR,    1, (void*) I2Cdrv, &i2cBusInfo, MMA8652_Init,   MMA8652_Read);
#endif
#if F_USING_GYRO
    sfg.installSensor(&sfg, &sensors[1], FXAS21002_I2C_ADDR,  1, (void*) I2Cdrv, &i2cBusInfo, FXAS21002_Init, FXAS21002_Read);
#endif
#if F_USING_MAG
    sfg.installSensor(&sfg, &sensors[2], MAG3110_I2C_ADDR,    1, (void*) I2Cdrv, &i2cBusInfo, MAG3110_Init, MAG3110_Read);
#endif
    sfg.initializeFusionEngine(&sfg);	        // This will initialize sensors and magnetic calibration

    pit_init(1000000/FUSION_HZ);                // pitIsrFlag will be set true at FUSION_HZ periodic intervals

    sfg.setStatus(&sfg, NORMAL);                // If we got this far, let's set status state to NORMAL
    while (true)
    {
        if (true == pitIsrFlag) {               // Check whether occur interupt and toggle LED
            sfg.readSensors(&sfg, 1);           // Reads sensors, applies HAL and does averaging (if applicable)
            sfg.conditionSensorReadings(&sfg);  // magCal is run as part of this
            sfg.runFusion(&sfg);                // Run the actual fusion algorithms
            sfg.applyPerturbation(&sfg);        // apply debug perturbation (testing only)
            sfg.loopcounter++;                  // The loop counter is used to "serialize" mag cal operations
            i=i+1;
            if (i>=4) {                         // Some status codes include a "blink" feature.  This loop
                    i=0;                        // should cycle at least four times for that to operate correctly.
                    sfg.updateStatus(&sfg);     // This is where pending status updates are made visible
            }

            sfg.queueStatus(&sfg, NORMAL);      // assume NORMAL status for next pass through the loop
            sfg.pControlSubsystem->stream(&sfg, sUARTOutputBuffer);      // Send stream data to the Sensor Fusion Toolbox
            pitIsrFlag = false;                 // Reset the flag for the next cycle
        }
    }
}
/// \endcode