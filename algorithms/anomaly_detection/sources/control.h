/*
* Copyright (c) 2015 - 2016, Freescale Semiconductor, Inc.
* All rights reserved.
*
* SPDX-License-Identifier: BSD-3-Clause
*/

/*! \file control.h
    \brief Defines control sub-system

   Each sensor fusion application will probably have its own set of functions
   to control the fusion process and report results.  This file defines the
   programming interface that should be followed in order for the fusion functions
   to operate correctly out of the box.  The actual command interpreter is
   defined separately in DecodeCommandBytes.c.  The output streaming function
   is defined in output_stream.c. Via these three files, the NXP Sensor Fusion
   Library provides a default set of functions which are compatible with the
   Sensor Fusion Toolbox.  Use of the toolbox is highly recommended at least
   during initial development, as it provides many useful debug features.
   The NXP development team will typically require use of the toolbox as a
   pre-requisite for providing software support.
*/

// Requires anomaly_detection.h to occur first in the #include stackup
#ifndef SENSOR_FUSION_CONTROL_H_
#define SENSOR_FUSION_CONTROL_H_

/// \brief he ControlSubsystem encapsulates command and data streaming functions.
///
/// The ControlSubsystem encapsulates command and data streaming functions
/// for the library.  A C++-like typedef structure which includes executable methods
/// for the subsystem is defined here.
typedef struct ControlSubsystem {
	volatile uint8_t flagOne;	///< 1st boolean flag
        bool StreamEnable; ///< Mode to control streaming
        bool TrainRun;     ///< Mode for Start
        bool CLR;          ///< Command: stop and clear existing model states
        bool Stop;         ///< Command: stop model operation
        bool Start;        ///< Command: start model operation (controlled via modes)
        bool Delete;       ///< Command: Delete specific model
        bool DeleteAll;    ///< Command: Delete ALL models
} ControlSubsystem;

int8_t initializeControlPort(ControlSubsystem *pComm);  ///< Call this once to initialize structures, ports, etc.

#endif /* SENSOR_FUSION_CONTROL_H_ */
