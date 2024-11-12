/*
* Copyright (c) 2015 - 2016, Freescale Semiconductor, Inc.
* All rights reserved.
*
* SPDX-License-Identifier: BSD-3-Clause
*/

#include <stdio.h>
#include "anomaly_detection.h"
#include "fusion.h"

/*! \file precisionAccelerometer.c
    \brief Implements accelerometer calibration routines
*/

// function resets the accelerometer buffer and accelerometer calibration
void fInitializeAccelCalibration(struct AccelCalibration *pthisAccelCal,
                                 struct AccelBuffer *pthisAccelBuffer,
                                 volatile int8 *AccelCalPacketOn)
{
}

void fUpdateAccelBuffer(struct AccelCalibration *pthisAccelCal,
                        struct AccelBuffer *pthisAccelBuffer,
                        struct AccelSensor *pthisAccel, volatile int8 *AccelCalPacketOn)
{
}

// function maps the accelerometer data fSum (g) onto precision calibrated and de-rotated data fGc (g), iGc (counts)
void fInvertAccelCal(struct AccelSensor *pthisAccel,
                     struct AccelCalibration *pthisAccelCal)
{
}

// function runs the precision accelerometer calibration
void fRunAccelCalibration(struct AccelCalibration *pthisAccelCal,
                          struct AccelBuffer *pthisAccelBuffer,
                          struct AccelSensor *pthisAccel)
{
}

// calculate the 4 element calibration from the available measurements
void fComputeAccelCalibration4(struct AccelBuffer *pthisAccelBuffer,
                               struct AccelCalibration *pthisAccelCal,
                               struct AccelSensor *pthisAccel)
{
}

// calculate the 7 element calibration from the available measurements
void fComputeAccelCalibration7(struct AccelBuffer *pthisAccelBuffer,
                               struct AccelCalibration *pthisAccelCal,
                               struct AccelSensor *pthisAccel)
{
}

// calculate the 10 element calibration from the available measurements
void fComputeAccelCalibration10(struct AccelBuffer *pthisAccelBuffer,
                                struct AccelCalibration *pthisAccelCal,
                                struct AccelSensor *pthisAccel)
{
}
