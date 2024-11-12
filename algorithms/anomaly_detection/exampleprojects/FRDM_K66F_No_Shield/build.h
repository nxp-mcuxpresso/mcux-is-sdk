/*
* Copyright (c) 2015 - 2016, Freescale Semiconductor, Inc.
* All rights reserved.
*
* SPDX-License-Identifier: BSD-3-Clause
*/

/*! \file build.h
    \brief Build configuration file

    This file contains only those parameters that directly relate to fusion
    implementation choices.  Board dependencies are in hal.h.  Consult the
    Sensor Fusion User Guide for guidance and details.
*/

#ifndef BUILD_H
#define BUILD_H


#define THISBUILD  		010     ///< define build number sent in debug packet for display purposes only

/// @name CoordinateSystemBitFields
/// These defines determine the frame of reference (x, y, z axes and Euler angles) standard
/// to be used for a particular build.  Change THISCOORDSYSTEM to whichever of NED, ANDROID
/// or WIN8 you prefer.
///@{
#define NED 0                           ///< identifier for NED (Aerospace) axes and angles
#define ANDROID 1                       ///< identifier for Android axes and angles
#define WIN8 2			        ///< identifier for Windows 8 axes and angles
#define THISCOORDSYSTEM NED		///< the coordinate system to be used
///@}

///@{
/// @name SensorBitFields
/// These bit-field values are used to declare which sensor types are used in the application.
/// Change bit-field values to 0x0000 for any features NOT USED
#define F_USING_ACCEL           0x0001  ///< nominally 0x0001 if an accelerometer is to be used, 0x0000 otherwise
#define F_USING_MAG             0x0000  ///< nominally 0x0002 if an magnetometer  is to be used, 0x0000 otherwise
#define F_USING_GYRO            0x0004  ///< nominally 0x0004 if a gyro           is to be used, 0x0000 otherwise
#define F_USING_PRESSURE        0x0000  ///< nominally 0x0008 if altimeter        is to be used, 0x0000 otherwise
#define F_USING_TEMPERATURE     0x0000  ///< nominally 0x0010 if temp sensor      is to be used, 0x0000 otherwise
#define F_ALL_SENSORS           0x001F  ///< refers to all applicable sensor types for the given physical unit

/// @name SensorParameters
/// FIFO sizes effect the size of the sensor data structures.  ODR refers to "Output Data Rate"
///@{
#define ACCEL_FIFO_SIZE 	200	///< FXOS8700 (accel), MMA8652, FXLS8952 all have 32 element FIFO
#define MAG_FIFO_SIZE 		200	///< FXOS8700 (mag), MAG3110 have no FIFO so equivalent to 1 element FIFO
#define GYRO_FIFO_SIZE 		400	///< FXAX21000, FXAS21002 have 32 element FIFO
#define ACCEL_ODR_HZ 		200    	///< (int) requested accelerometer ODR Hz (over-rides MAG_ODR_HZ for FXOS8700)
#define MAG_ODR_HZ 		200    	///< (int) requested magnetometer ODR Hz (over-ridden by ACCEL_ODR_HZ for FXOS8700)
#define GYRO_ODR_HZ 		400    	///< (int) requested gyroscope ODR Hz
#define TEMP_ODR_HZ             0       ///< (int) requested temp sensor ODR Hz
#define PRESSURE_ODR_HZ         0       ///< (int) requested pressure sensor ODR Hz
#define MICROPHONE_ODR_HZ       0       ///< (int) requested microphone ODR Hz
#define FUSION_HZ 		4	///< (int) actual rate of fusion algorithm execution and sensor FIFO reads
#define FAST_LOOP_HZ		20	///< Over Sample Ratio * FUSION_HZ when using no FIFO
#define OVERSAMPLE_RATE         FAST_LOOP_HZ/FUSION_HZ
///@}

#define INCLUDE_DEBUG_FUNCTIONS   // Comment this line to disable the ApplyPerturbation function
#define THIS_SHIELD 0
#endif // BUILD_H
