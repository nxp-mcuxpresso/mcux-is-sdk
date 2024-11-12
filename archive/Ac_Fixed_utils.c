/*
 * Copyright (c) 2015 - 2016, Freescale Semiconductor, Inc.
 * Copyright 2016-2017 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/*! File: Ac_Fixed_utils.c
* @brief The \b Ac_Fixed_utils.c file containes the definition for the
*        float to fixed point conversion utility function.
*/

#include "Ac_Fixed_utils.h"

typedef union {
    float  fN;
    uint32_t iN;
} conversionBuffer_t;

#define IEEE754_MANTISSA_MASK (0x7fffff)
#define IMPLICIT_ONE (0x800000)
#define IEEE754_EXPONENT(n)  ((int32_t)((n)>>23) & 0xff)
#define IEEE754_MANTISSA(n) ((int32_t)(n) & IEEE754_MANTISSA_MASK)
#define IEEE754_SIGN(n) (((uint32_t)(n))>>31)

/*******************************************************************************
 * Code
 ******************************************************************************/

int32_t float32_To_AcFixed(float fN, uint16_t format, int32_t *status)
//uint32_t float32_To_AcFixed(float fN, uint8_t W, uint8_t I, uint8_t S, int32_t *status)
{
    conversionBuffer_t cB;
    uint32_t n;
    int32_t  m,e,shift;
    uint8_t W,I,S;
    
    // format is 16 bit integer value as 0xS000WWWWWWIIIIII
    I = format & 0x003F;
    W = (format & 0x0FC0) >> 6;
    S = ((int16_t)format)<0;

    cB.fN = fN;
    m = IEEE754_MANTISSA(cB.iN);  /* mantissa */
    e = IEEE754_EXPONENT(cB.iN);  /* exponent */
    n = IEEE754_SIGN(cB.iN);      /* negative (sign bit is set) */
    
    /* add implicit 1 if exponent is not zero */
    if (e) m |= IMPLICIT_ONE;
    
    /* remove 127 bias for true exponent */
    e -= 127;
    
    /* compute amount to shift to get to the specified fixed format */
    shift = (I - e - 1) + (24 - W);
            
    /*  If shift is not enough to move the "1" into the
    **  valid bit range for the specified format, the
    **  original number is too large to fit.
    **  Report the error.
    */
    if (shift < (24 - W + S)) (*status)++;
        
    m >>= shift;
    
    /* change sign if float was negative and fixed format specifies a sign bit */
    if (n && S) m = -m;
    
    return (uint32_t)m;
}


#ifdef COMPILE_TEST_FUNCTIONS

typedef struct {
    float num;
    uint8_t W;
    uint8_t I;
    uint8_t S;
} AcFixed_t;

// This is by no means a complete test suite- simply a few numbers to check the
// operation
static AcFixed_t fnums[] = {
    { .num=  15.356789,  .W=16, .I=2, .S=0},
    { .num=   1.356789,  .W=16, .I=2, .S=0},
    { .num=   0.00356789,.W=16, .I=2, .S=0},
    { .num= 500.0,       .W=16, .I=8, .S=1},
    { .num=-500.0,       .W=16, .I=8, .S=1},
    { .num=   1.0,       .W=16, .I=8, .S=1},
    { .num=  -1.0,       .W=16, .I=8, .S=1},
    { .num=   2.5,       .W=16, .I=8, .S=1},
    { .num=  -2.5,       .W=16, .I=8, .S=1},
    { .num=  37.5,       .W=16, .I=8, .S=1},
    { .num= 112.9875,    .W=16, .I=8, .S=1},
    { .num= 360.0,       .W=16, .I=8, .S=1},
    { .num= 249.1231232, .W=16, .I=8, .S=1},
    { .num= -45.965,     .W=16, .I=8, .S=1},
    { .num= -27.8,       .W=16, .I=8, .S=1},
    { .num= -25.0,       .W=16, .I=8, .S=1},
    { .num=-279.2,       .W=16, .I=8, .S=1},
    { .num=-360.0,       .W=16, .I=8, .S=1}
};
           
void test_fixed_point()
{
    int32_t  st;
    uint16_t fixedPt;
    int    i;
    
    for (i=0; i < sizeof(fnums)/sizeof(AcFixed_t); i++ )
    {
        fixedPt = (uint16_t) float32_To_AcFixed(fnums[i].num, fnums[i].W, fnums[i].I, fnums[i].S, &st);
    };
}
#endif
