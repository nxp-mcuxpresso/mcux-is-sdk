/*
* Copyright (c) 2015 - 2016, Freescale Semiconductor, Inc.
* All rights reserved.
*
* SPDX-License-Identifier: BSD-3-Clause
*/


/*! \file fusion.h
    \brief Lower level sensor fusion interface

    This file can be used to "tune" the performance of specific algorithms
    within the sensor fusion library.  It also defines the lower level function
    definitions for specific algorithms.  Normally, the higher level hooks
    in anomaly_detection.h will be used, and those shown here will be left alone.
*/

#ifndef FUSION_H
#define FUSION_H

#include "anomaly_detection.h"

/// @name COMPUTE_1DOF_P_BASIC constants
///@{
#define FLPFSECS_1DOF_P_BASIC		1.5F            ///< pressure low pass filter time constant (s)
///@}

/// @name COMPUTE_3DOF_G_BASIC constants
///@{
#define FLPFSECS_3DOF_G_BASIC		1.0F            ///< tilt orientation low pass filter time constant (s)
///@}

/// @name COMPUTE_3DOF_B_BASIC constants
///@{
#define FLPFSECS_3DOF_B_BASIC		7.0F            ///< 2D eCompass orientation low pass filter time constant (s)
///@}

/// @name COMPUTE_6DOF_GB_BASIC constants
///@{
#define FLPFSECS_6DOF_GB_BASIC		7.0F            /// <3D eCompass orientation low pass filter time constant (s)
///@}

/// @name COMPUTE_6DOF_GY_KALMAN constants
///@{
#define FQVY_6DOF_GY_KALMAN			2E2     ///< gyro sensor noise variance units (deg/s)^2
#define FQVG_6DOF_GY_KALMAN			1.2E-3  ///< accelerometer sensor noise variance units g^2
#define FQWB_6DOF_GY_KALMAN			2E-2F   ///< gyro offset random walk units (deg/s)^2
#define FMIN_6DOF_GY_BPL			-7.0F   ///< minimum permissible power on gyro offsets (deg/s)
#define FMAX_6DOF_GY_BPL			7.0F    ///< maximum permissible power on gyro offsets (deg/s)
///@}

/// @name COMPUTE_9DOF_GBY_KALMAN constants
///@{
/// gyro sensor noise covariance units deg^2
/// increasing this parameter improves convergence to the geomagnetic field
#define FQVY_9DOF_GBY_KALMAN		2E2		///< gyro sensor noise variance units (deg/s)^2
#define FQVG_9DOF_GBY_KALMAN		1.2E-3	        ///< accelerometer sensor noise variance units g^2 defining minimum deviation from 1g sphere
#define FQVB_9DOF_GBY_KALMAN		5E0		///< magnetometer sensor noise variance units uT^2 defining minimum deviation from geomagnetic sphere.
#define FQWB_9DOF_GBY_KALMAN		2E-2F	        ///< gyro offset random walk units (deg/s)^2
#define FMIN_9DOF_GBY_BPL		-7.0F           ///< minimum permissible power on gyro offsets (deg/s)
#define FMAX_9DOF_GBY_BPL		7.0F            ///< maximum permissible power on gyro offsets (deg/s)
///@}

/// @name Fusion Function Prototypes
/// These functions comprise the core of the basic sensor fusion functions excluding
/// magnetic and acceleration calibration.  Parameter descriptions are not included here,
/// as details are provided in anomaly_detection.h.
///@{
void fInitializeFusion(Globals *gbls);
///@}

#endif   // #ifndef FUSION_H
