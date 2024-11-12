/*
 * Copyright (c) 2016, Freescale Semiconductor, Inc.
 * Copyright 2017 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/**
 * @file issdk_ble_interface.h
 * @brief The issdk_ble_interface.h file containes the ISSDK to BLE Interface definitions.
*/

#ifndef _ISSDK_BLE_INTERFACE_H_
#define _ISSDK_BLE_INTERFACE_H_

#include <stdint.h>
#include <stdlib.h>

/*! @brief Sensor Operation Modes */
enum
{
    ISSDK_ACTIVE_MODE,
    ISSDK_STANDBY_MODE,
};

/*! @brief Sensor Report Devices */
enum
{
    ISSDK_REPORT_NONE,
    ISSDK_REPORT_ACCELERATION,
    ISSDK_REPORT_ROTATION,
    ISSDK_REPORT_MAGNETIC,
    ISSDK_REPORT_PRESSURE,
    ISSDK_REPORT_ALTITUDE,
    ISSDK_REPORT_TEMPERATURE,
    ISSDK_REPORT_ECOMPASS,
    ISSDK_REPORT_THRESHOLD,
    ISSDK_REPORT_FREEFALL,
    ISSDK_REPORT_ORIENTATION,
    ISSDK_REPORT_PEDOMETER,
    ISSDK_REPORT_CUSTOM,
};

/*! @brief Sensor Register Actions */
enum
{
    ISSDK_REGISTER_NONE,
    ISSDK_REGISTER_READ,
    ISSDK_REGISTER_WRITE,
};

/*! @brief Sensor Data Report Modes */
enum
{
    ISSDK_READ_RAW,
    ISSDK_READ_NORMAL,
    ISSDK_READ_STREAM,
};

/*! @brief This structure defines the register interface command container. */
struct _issdk_RLIcommand
{
    char tag[3];
    char _delimiter1;
    char device[1];
    char _delimiter2;
    char action[1];
    char _delimiter3;
    char offset[2];
    char _delimiter4;
    char bytes[2];
    char _padding[3];
};

/*! @brief This structure defines the data mode command container. */
struct _issdk_ModeCommand
{
    char tag[3];
    char _delimiter1;
    char mode[1];
    char _padding[3];
};

typedef union
{
    struct _issdk_RLIcommand rliCmd;
    struct _issdk_ModeCommand modeCmd;
} ISSDK_BLEcommand_t;

/* XXX: User Interface Commands
 * Sensor Operation Mode Commands:
 * Format = <TAG><D><MODE>
 * TAG    = 'CMD'           (3 Fixed Characters)
 * MODE   = 'STANDBY'       (7 Fixed Characters to Stop All Sensor Data)
 * MODE   = 'ACCELEROMETER' (13 Fixed Characters to Start Accelerometer Data)
 * MODE   = 'MAGNETOMETER'  (12 Fixed Characters to Start Magnetometer Data)
 * MODE   = 'GYROSCOPE'     (9 Fixed Characters to Start Gyroscope Data)
 * MODE   = 'BAROMETER'     (9 Fixed Characters to Start Barometer Data)
 * MODE   = 'ALTIMETER'     (9 Fixed Characters to Start Altimeter Data)
 * MODE   = 'THERMOMETER'   (11 Fixed Characters to Start Temperature Data)
 * MODE   = 'ECOMPASS'      (8 Fixed Characters to Start eCompass Data)
 * MODE   = 'THRESHOLD'     (9 Fixed Characters to Start Threshold detection)
 * MODE   = 'FREEFALL'      (8 Fixed Characters to Start Freefall detection)
 * MODE   = 'PEDOMETER'     (9 Fixed Characters to Pedometer Data capture)
 * MODE   = 'ORIENTATION'   (11 Fixed Characters to Start Orientation detection)
 * MODE   = 'CUSTOM'        (6 Fixed Characters to Start Custom detection)
 * D      = ' '             (Delimiter : Any Single Character like ' ' Space or ',' Comma or ':' Colon...)
 *
 * Sensor Data Format Commands:
 * Format = <TAG><D><MODE>
 * TAG    = 'CMD'           (3 Fixed Characters)
 * MODE   = 'NORMAL'        (6 Fixed Characters to view Converted Sensor Data)
 * MODE   = 'RAW'           (3 Fixed Characters to view Raw Sensor Data)
 * MODE   = 'STREAM'        (6 Fixed Characters to Start Streaming Data)
 * D      = ' '             (Delimiter : Any Single Character like ' ' Space or ',' Comma or ':' Colon...)
 *
 * Sensor Register Access Commands:
 * Format = <TAG><D><DEVICE><D><ACTION><D><OFFSET><D><BYTES>
 * TAG    = 'RLI'      (3 Fixed Characters)
 * DEVICE = 'A'        ('A' = ACCEL or 'M' = MAG or 'G' = GYRO or 'P' = PRESSURE)
 * ACTION = 'R'        ('R' = READ or 'W' = WRITE)
 * OFFSET = 'XX'       (00-7F in HEX for Register Start OFFSET)
 * BYTES  = 'YY'       (00-7F in HEX for Bytes to READ)
 * BYTES  = 'ZZZZZZZZ' (00-7F in HEX and up to 4 Bytes MAX to WRITE)
 * D      = ' '        (Delimiter : Any Single Character like ' ' Space or ',' Comma or ':' Colon...)
*/

/* Sensor User Interface Command Strings */
#define ISSDK_MODE_CMD_TAG "CMD"
#define ISSDK_REPORT_NONE_STR "STANDBY"
#define ISSDK_REPORT_ACCELERATION_STR "ACCELEROMETER"
#define ISSDK_REPORT_MAGNETIC_STR "MAGNETOMETER"
#define ISSDK_REPORT_ROTATION_STR "GYROSCOPE"
#define ISSDK_REPORT_PRESSURE_STR "BAROMETER"
#define ISSDK_REPORT_ALTITUDE_STR "ALTIMETER"
#define ISSDK_REPORT_TEMPERATURE_STR "THERMOMETER"
#define ISSDK_REPORT_ECOMPASS_STR "ECOMPASS"
#define ISSDK_REPORT_THRESHOLD_STR "THRESHOLD"
#define ISSDK_REPORT_FREEFALL_STR "FREEFALL"
#define ISSDK_REPORT_PEDOMETER_STR "PEDOMETER"
#define ISSDK_REPORT_ORIENTATION_STR "ORIENTATION"
#define ISSDK_REPORT_CUSTOM_STR "CUSTOM"
#define ISSDK_REPORT_CONV_DATA_STR "NORMAL"
#define ISSDK_REPORT_RAW_DATA_STR "RAW"
#define ISSDK_REPORT_STREAM_DATA_STR "STREAM"

/* Streaming Header Strings */
#define ISSDK_ACCEL_STREAM_HDR_STR "ST-3X-ACC"
#define ISSDK_MAG_STREAM_HDR_STR "ST-3X-MAG"
#define ISSDK_GYRO_STREAM_HDR_STR "ST-3X-GYR"
#define ISSDK_COMBO_STREAM_HDR_STR "ST-6X-COM"
#define ISSDK_PRESSURE_STREAM_HDR_STR "ST-1X-PRE"
#define ISSDK_ALT_STREAM_HDR_STR "ST-1X-ALT"
#define ISSDK_TEMP_STREAM_HDR_STR "ST-1X-TEM"

/* Sensor RLI Interface Command Strings */
#define ISSDK_REGISTER_CMD_TAG "RLI"
#define ISSDK_REGISTER_ACCEL_TAG "A"
#define ISSDK_REGISTER_GYRO_TAG "G"
#define ISSDK_REGISTER_MAG_TAG "M"
#define ISSDK_REGISTER_PRESSURE_TAG "P"
#define ISSDK_REGISTER_MODE_R_TAG "R"
#define ISSDK_REGISTER_MODE_W_TAG "W"

/* Sensor Console display Suffix Strings */
#define ISSDK_DEBUG_SUFFIX "DEBUG : "
#define ISSDK_ERROR_SUFFIX "ERROR : "
#define ISSDK_EVENT_SUFFIX "EVENT : "

/* Sensor RLI Command Length up to OFFSET sub-segment */
#define ISSDK_BLE_REG_CMD_OFFSET (12)

/*! @brief Macro to convert Float to Decimal for Streaming. */
#define ISSDK_GetDecimalForFloat(x) (int)(100.0 * x)

/*! @brief Sensor BLE Interface Write to Host function */
void BleApp_WriteToHost(char *format, ...);

/*! @brief Sensor BLE Interface Write to UART function */
void BleApp_WriteToUART(char *format, ...);

/*! @brief Function to Convert Float into Strings for display */
static inline void ISSDK_GetStringOfFloat(float f, char *s)
{
    int i = (int)f;
    int d = (int)(f * 100);

    i = abs(i);
    d = abs(d) - (i * 100);
    if (f > 0) /* Precision is 2 */
    {
        sprintf(s, "+%d.%02d", i, d);
    }
    else
    {
        sprintf(s, "-%d.%02d", i, d);
    }
}

/*! @brief       The interface function to initialize sensors.
 *  @details     This function initialize all the sensor associated with the shield board.
 *  @constraints This should be the first API to be called for Sensor Access.
 *               Application has to ensure that previous instances of these APIs have exited before invocation.
 *  @reeentrant  No
 *  @return      ::ISSDK_Initialize() returns void.
 */
void ISSDK_Initialize(void);

/*! @brief       The interface function to configure sensors.
 *  @details     This function configures the selected sensor with the requested operation.
 *  @constraints This can be called any number of times only after ISSDK_Initialize().
 *  @param[in]   opCode  The Code for the requested operation.
 *  @reeentrant  No
 *  @return      ::ISSDK_Configure() returns void.
 */
void ISSDK_Configure(uint8_t opCode);

/*! @brief       The interface function to parse Host Command Strings.
 *  @details     This function Parses Host Command Strings and performs the requested task.
 *  @constraints This can be called any number of times only after ISSDK_Initialize().
 *  @param[in]   *pCommand  The Buffer containing the Command String.
 *  @param[in]   commandLength  The Length of the Command String.
 *  @reeentrant  No
 *  @return      ::ISSDK_ProcessHostCommand() returns bool (true/false).
 */
bool ISSDK_ProcessHostCommand(uint8_t *pCommand, uint16_t commandLength);

#endif /* _ISSDK_BLE_INTERFACE_H_ */
