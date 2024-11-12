/*
* Copyright (c) 2015 - 2016, Freescale Semiconductor, Inc.
* All rights reserved.
*
* SPDX-License-Identifier: BSD-3-Clause
*/

/*! \file motionCheck.c
    \brief check to see if the board is moving.

    This function would normally be called from your fusion_tasks in your main.c.
    See main_freertos_two_tasks_power_cycling.c for example usage.
*/

// Sensor Fusion Headers
#include "sensor_fusion.h"      // top level magCal and sensor fusion interfaces

/// The motionCheck() function is not a sensor fusion function.  It is a function
/// that simply monitors an accelerometer or magnetometer tri-axial sensor output,
/// returning Boolean true if the sensor appears to be stationary, and false otherwise.
/// This function would normally be called from your fusion_tasks in your main().
bool motionCheck(
    float sample[3],            ///< processed triaxial sensor sample (accel or mag)
    float baseline[3],          ///< previous value to compare to
    float tolerance,            ///< how much tolerance you can stand
    uint32_t winLength,         ///< how many samples need to be stable to assert "noMotion"
    uint32_t *count             ///< how many samples so far we've been not moving
)
{
    float change[3];
    bool changed;
    change[CHX] = fabs(baseline[CHX] - sample[CHX]);
    change[CHY] = fabs(baseline[CHY] - sample[CHY]);
    change[CHZ] = fabs(baseline[CHZ] - sample[CHZ]);
    changed = (change[CHX]>tolerance) ||
              (change[CHY]>tolerance) ||
              (change[CHZ]>tolerance);
    if (changed) {
        baseline[CHX] = sample[CHX];
        baseline[CHY] = sample[CHY];
        baseline[CHZ] = sample[CHZ];
        *count = 0;
    } else {
        if ((*count) <= winLength) (*count) += 1;
    }
    return(*count > winLength);
}
