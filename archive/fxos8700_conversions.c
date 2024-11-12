/*
 * Copyright (c) 2015-2016, Freescale Semiconductor, Inc.
 * Copyright 2016-2017 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/*! File: fxos8700_conversions.c
* @brief The \b fxos8700_conversions.c file defines common conversion specifiers
*        for use with the FXOS8700 sensor
*/

#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>
#include "convert.h"
#include "fxos8700_conversions.h"

/*******************************************************************************
 * Constants
 ******************************************************************************/
const ConversionSpecifier_t FXOS8700_ACCEL_COUNT_CONVERSION[] =
    {
       FXOS8700_RAW16_TO_COUNTS_ACCEL,
       FXOS8700_RAW16_TO_COUNTS_ACCEL,
       FXOS8700_RAW16_TO_COUNTS_ACCEL,
       CONV_BUFFER_COMPLETE
    };

const ConversionSpecifier_t FXOS8700_ACCEL_FLOAT_CONVERSION[] =
    {
       FXOS8700_2G_RAW16_TO_FLOAT_ACCEL,
       FXOS8700_2G_RAW16_TO_FLOAT_ACCEL,
       FXOS8700_2G_RAW16_TO_FLOAT_ACCEL,
       CONV_BUFFER_COMPLETE
    };

const ConversionSpecifier_t FXOS8700_ACCEL_FIXED_CONVERSION[] =
    {
       FXOS8700_2G_RAW16_TO_FIXED_ACCEL,
       FXOS8700_2G_RAW16_TO_FIXED_ACCEL,
       FXOS8700_2G_RAW16_TO_FIXED_ACCEL,
       CONV_BUFFER_COMPLETE
    };

const ConversionSpecifier_t FXOS8700_MAG_COUNT_CONVERSION[] =
    {
       FXOS8700_RAW16_TO_COUNTS_MAG,
       FXOS8700_RAW16_TO_COUNTS_MAG,
       FXOS8700_RAW16_TO_COUNTS_MAG,
       CONV_BUFFER_COMPLETE
    };

const ConversionSpecifier_t FXOS8700_MAG_FLOAT_CONVERSION[] =
    {
       FXOS8700_2G_RAW16_TO_FLOAT_MAG,
       FXOS8700_2G_RAW16_TO_FLOAT_MAG,
       FXOS8700_2G_RAW16_TO_FLOAT_MAG,
       CONV_BUFFER_COMPLETE
    };

const ConversionSpecifier_t FXOS8700_MAG_FIXED_CONVERSION[] =
    {
       FXOS8700_2G_RAW16_TO_FIXED_MAG,
       FXOS8700_2G_RAW16_TO_FIXED_MAG,
       FXOS8700_2G_RAW16_TO_FIXED_MAG,
       CONV_BUFFER_COMPLETE
    };

const ConversionSpecifier_t FXOS8700_ACCELMAG_COUNT_CONVERSION[] =
    {
       FXOS8700_RAW16_TO_COUNTS_ACCEL,
       FXOS8700_RAW16_TO_COUNTS_ACCEL,
       FXOS8700_RAW16_TO_COUNTS_ACCEL,
       FXOS8700_RAW16_TO_COUNTS_MAG,
       FXOS8700_RAW16_TO_COUNTS_MAG,
       FXOS8700_RAW16_TO_COUNTS_MAG,
       CONV_BUFFER_COMPLETE
    };

const ConversionSpecifier_t FXOS8700_ACCELMAG_FLOAT_CONVERSION[] =
    {
       FXOS8700_2G_RAW16_TO_FLOAT_ACCEL,
       FXOS8700_2G_RAW16_TO_FLOAT_ACCEL,
       FXOS8700_2G_RAW16_TO_FLOAT_ACCEL,
       FXOS8700_2G_RAW16_TO_FLOAT_MAG,
       FXOS8700_2G_RAW16_TO_FLOAT_MAG,
       FXOS8700_2G_RAW16_TO_FLOAT_MAG,
       CONV_BUFFER_COMPLETE
    };

const ConversionSpecifier_t FXOS8700_ACCELMAG_FIXED_CONVERSION[] =
    {
       FXOS8700_2G_RAW16_TO_FIXED_ACCEL,
       FXOS8700_2G_RAW16_TO_FIXED_ACCEL,
       FXOS8700_2G_RAW16_TO_FIXED_ACCEL,
       FXOS8700_2G_RAW16_TO_FIXED_MAG,
       FXOS8700_2G_RAW16_TO_FIXED_MAG,
       FXOS8700_2G_RAW16_TO_FIXED_MAG,
       CONV_BUFFER_COMPLETE
    };