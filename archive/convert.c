/*
 * Copyright (c) 2015 - 2016, Freescale Semiconductor, Inc.
 * Copyright 2016-2017 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/*! File: convert.c
* @brief The \b convert.c file containes the definitions for the sensor data
*        conversion utility functions.
*/

#include <stdint.h>
#include "convert.h"
#include "Ac_Fixed_utils.h"

/*******************************************************************************
 * Code
 ******************************************************************************/

int32_t Convert_Sensor_Data(const ConversionSpecifier_t *convSpec, uint8_t *rawBuffer, void *result)
{
    Conversion_t  n;
    int           done = 0;
    uint8_t      *pRaw = rawBuffer;
    uint8_t      *pOut = (uint8_t *)result;

    for (int i=0; !done; i++)
    {
        switch (convSpec[i].op)
        {
            case CONVERSION_SKIP_BYTES:
                pRaw += convSpec[i].value.numBytes;
                break; 

            case CONVERSION_BYTES:
                n.size = convSpec[i].value.numBytes; 
                break; 

            case CONVERSION_ENDIANNESS:
                 {
                     uint32_t s = convSpec[i].value.endianness;
                     n.v.i = 0;
                     for(int j=0; j<n.size; j++)
                     {
                         uint32_t m = s & 0x000F;
                         if (m != 0xF)
                         {
                            n.v.i |= (pRaw[m] << (j<<3));
                         }
                         s >>= 4; 
                     }
                     pRaw += n.size;
                 }
                 break;

            case CONVERSION_MASK:
                 n.v.i &= convSpec[i].value.mask;
                 break;

            case CONVERSION_LSHIFT:
                 n.v.i <<= convSpec[i].value.shift;
                 break;
                 
            case CONVERSION_RSHIFT:
                 n.v.i >>= convSpec[i].value.shift;
                 break;
                 
            case CONVERSION_RSHIFT_SE:
                {
                   int w = 32 - (convSpec[i].value.shift >> 16);
                   n.v.i <<= w;   // shift sign bit of data into sign bit pos
                   n.v.si >>= w + (int32_t)(convSpec[i].value.shift&0xFF); // now shift right with sign extension
                   n.size = 4;
                }
                break;
                 
            case CONVERSION_INT_ADD:
                n.v.si += convSpec[i].value.i_val;
                break;

            case CONVERSION_FLOAT_CAST:
                {
                    int w = 32 - convSpec[i].value.shift;
                    n.v.i <<= w;   // shift sign bit of data into sign bit pos
                    n.v.si >>= w; // now shift back with sign extension  
                    n.v.f = (float)(n.v.si);

                    n.size = 4;
                }
                break;

            case CONVERSION_FLOAT_ADD:
                n.v.f += convSpec[i].value.f_val;
                break;

            case CONVERSION_FLOAT_MULTIPLY:
                n.v.f *= convSpec[i].value.f_val;
                break;

            case CONVERSION_INT_MULTIPLY:
                n.v.si *= convSpec[i].value.i_val;
                break;

            case CONVERSION_FIXED_POINT:
                {
                        int32_t st;
                        n.v.i = float32_To_AcFixed(n.v.f, convSpec[i].value.format, &st);
                }
                break;

            case CONVERSION_PAD_BYTES:
                pOut += convSpec[i].value.numBytes;
                break;

            case CONVERSION_END:
                {
                    if (n.size == 0) return SUCCESS;

                    uint8_t *b = (uint8_t *)&n.v.i;

                    for (int j=0; j++<n.size; )
                        *pOut++ = *b++;

                }
                n.size = 0;
                break;

            default:
                return BAD_PARAM;
        }
    }
    return 0;
}
