/*
 * Copyright (c) 2016, Freescale Semiconductor, Inc.
 * Copyright 2017 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/**
 * @file issdk_interface.h
 * @brief The issdk_interface.h file containes the ISSDK to BLE Interface prototypes.
*/

#ifndef _ISSDK_INTERFACE_H_
#define _ISSDK_INTERFACE_H_

/* ISSDK Sensor States */
enum {
    ISSDK_SENSOR_START,
    ISSDK_SENSOR_STOP,
    ISSDK_SENSOR_ALTITUDE_MODE,
    ISSDK_SENSOR_PRESSURE_MODE,
};

/* ISSDK Sensor Report Modes */
enum {
    ISSDK_REPORT_ALTITUDE,
    ISSDK_REPORT_PRESSURE,
};

#define ISSDK_REPORT_ALTITUDE_STR "ALTIMETER"
#define ISSDK_REPORT_PRESSURE_STR "BAROMETER"

/* ISSDK Sensor Interface functions */
void ISSDK_InitSensor(void);
void ISSDK_ReadPressure(void);
void ISSDK_ReadAltitide(void);
void ISSDK_ConfigureSensor(uint32_t arg);

/* BLE Driver Interface functions */
void BleApp_WriteToHost(char *format, ...);
void BleApp_WriteToUART(char *format, ...);

#endif /* _ISSDK_INTERFACE_H_ */
