/*
 * Copyright (c) 2015 - 2016, Freescale Semiconductor, Inc.
 * Copyright 2016-2017 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/*! File: fxos8700_conversions.h
* @brief The \b fxos8700_conversions.h file defines common conversion specifiers
*        for use with the FXOS8700 sensor
*/

#ifndef FXOS8700_CONVERSION_H_
#define FXOS8700_CONVERSION_H_

#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>
#include "convert.h"

//-----------------------------------------------------------------------
// Typedefs
//-----------------------------------------------------------------------

/*******************************************************************************
 * Conversion Definitions
 ******************************************************************************/
#define FXOS8700_2G_RAW16_TO_FLOAT_ACCEL  \
       CONV_BYTES(2),                     \
       CONV_ENDIANNESS(0x01),             \
       CONV_RSHIFT_SE(16,2),              \
       CONV_FLOAT_CAST(32),               \
       CONV_FLOAT_MULTIPLY(0.244/1000.0), \
       CONV_COMPLETE

#define FXOS8700_RAW16_TO_COUNTS_ACCEL    \
       CONV_BYTES(2),                     \
       CONV_ENDIANNESS(0x01),             \
       CONV_COMPLETE
         
#define FXOS8700_2G_RAW16_TO_FIXED_ACCEL  \
       CONV_BYTES(2),                     \
       CONV_ENDIANNESS(0x01),             \
       CONV_RSHIFT_SE(16,2),              \
       CONV_FLOAT_CAST(32),               \
       CONV_FLOAT_MULTIPLY(0.244/1000.0), \
       CONV_FIXED_POINT(32,16,1),         \
       CONV_COMPLETE
     
#define FXOS8700_2G_RAW16_TO_FLOAT_MAG    \
       CONV_BYTES(2),                     \
       CONV_ENDIANNESS(0x01),             \
       CONV_FLOAT_CAST(16),               \
       CONV_FLOAT_MULTIPLY(0.1),          \
       CONV_COMPLETE

#define FXOS8700_RAW16_TO_COUNTS_MAG      \
       CONV_BYTES(2),                     \
       CONV_ENDIANNESS(0x01),             \
       CONV_COMPLETE
         
#define FXOS8700_2G_RAW16_TO_FIXED_MAG    \
       CONV_BYTES(2),                     \
       CONV_ENDIANNESS(0x01),             \
       CONV_FLOAT_CAST(16),               \
       CONV_FLOAT_MULTIPLY(0.1),          \
       CONV_FIXED_POINT(32,16,1),         \
       CONV_COMPLETE
/*******************************************************************************
 * Conversion Constants
 ******************************************************************************/
// Conversion specifier to convert a raw sensor acceleration data buffer to 
// counts (essentially this just handles endianness)
extern const ConversionSpecifier_t FXOS8700_ACCEL_COUNT_CONVERSION[];

// Conversion specifier to convert a raw sensor acceleration data buffer to 
// floating point G's
extern const ConversionSpecifier_t FXOS8700_ACCEL_FLOAT_CONVERSION[];

// Conversion specifier to convert a raw sensor acceleration data buffer to 
// fixed point AC_Fixed(32,16,1)
extern const ConversionSpecifier_t FXOS8700_ACCEL_FIXED_CONVERSION[];

// Conversion specifier to convert a raw sensor magnetometer data buffer to 
// counts (essentially this just handles endianness)
extern const ConversionSpecifier_t FXOS8700_MAG_COUNT_CONVERSION[];

// Conversion specifier to convert a raw sensor magnetometer data buffer to 
// floating point G's
extern const ConversionSpecifier_t FXOS8700_MAG_FLOAT_CONVERSION[];

// Conversion specifier to convert a raw sensor magnetometer data buffer to 
// fixed point AC_Fixed(32,16,1)
extern const ConversionSpecifier_t FXOS8700_MAG_FIXED_CONVERSION[];

// Conversion specifier to convert a raw sensor magnetometer data buffer to 
// counts (essentially this just handles endianness)
extern const ConversionSpecifier_t FXOS8700_ACCELMAG_COUNT_CONVERSION[];

// Conversion specifier to convert a raw sensor magnetometer data buffer to 
// floating point G's
extern const ConversionSpecifier_t FXOS8700_ACCELMAG_FLOAT_CONVERSION[];

// Conversion specifier to convert a raw sensor magnetometer data buffer to 
// fixed point AC_Fixed(32,16,1)
extern const ConversionSpecifier_t FXOS8700_ACCELMAG_FIXED_CONVERSION[];
#endif //FXOS8700_CONVERSION_H_
