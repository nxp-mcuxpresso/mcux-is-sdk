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
    ISSDK_START_SENSOR,
    ISSDK_STOP_SENSOR,
};

/* ISSDK Sensor Report Modes */
enum {
    ISSDK_REPORT_STEPS,
    ISSDK_REPORT_DATA,
};

#define ISSDK_REPORT_DATA_STR "DATA"
#define ISSDK_REPORT_STEPS_STR "STEPS"

/* ISSDK Sensor Interface functions */
void ISSDK_InitSensor(void);
void ISSDK_ReadSensor(void);
void ISSDK_ReadPedometer(void);
void ISSDK_ConfigureSensor(uint32_t arg);

/* BLE Driver Interface functions */
void BleApp_WriteToHost(char *format, ...);
void BleApp_WriteToUART(char *format, ...);

#endif /* _ISSDK_INTERFACE_H_ */
