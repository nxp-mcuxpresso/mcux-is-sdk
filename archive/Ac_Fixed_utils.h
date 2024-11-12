/*
 * Copyright (c) 2015 - 2016, Freescale Semiconductor, Inc.
 * Copyright 2016-2017 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/*! File: Ac_Fixed_utils.h
* @brief The \b Ac_Fixed_utils.h file containes the prototype definition for the
*        float to fixed point conversion utility function.
*/

#ifndef AC_FIXED_UTILS_H_
#define AC_FIXED_UTILS_H_

#include <stdint.h>
/*! 
 * 
 * @brief This function converts an IEEE-754 32-bit floating point number into a 
 * fixed point integer format
 * 
 * @details The AC_Fixed(W,I,S) format describes a fixed point representation
 *          for numbers with a fractional component.
 *          The W is the full width of the resultant integer in bits
 *          W=16 means the integer is 16 bits wide and can be stored in a short
 *          or uint16.
 *          I represents the number of bits on the integer side of the fixed
 *          decimal point and includes the sign bit, if the number is a signed value.
 *          I=8 would indicate 8 bits are used for the integer portion of the number.
 *          S represents the signedness of the number. if S=1 then 1 bit in (I) 
 *          is used as the sign bit.  I=8,S=1 means the integer value is stored as a
 *          signed two's complement value with the same range as a signed 8-bit char.
 *          Each remaining bit (there are W-I of them) represent the fractional component
 *          of the value. the first bit to the right of the fixed decimal point represents the 
 *          value 0.5, the next to the right 0.25 etc.  In other words, the least significant bit
 *          would have the value 1 / ( 2^(W-I)) and each bit to the left would be twice that.
 *          As an example the value 3.625 could be represented in AC_Fixed(16,3,1) as
 *          3.625 * 2^13 = 29696d = 0111010000000000b where the fixed decimal point comes between
 *          bits 12 and 13:  011.1010000000000 and it is easy to see that there is an integer 3
 *          to the left of the decimal and a 0.5 + 0.125 on the right for a total of 3.625.
 *          
 *
 * @param [in]      fN The floating point number to be converted
 * 
 * @param [in]      W  The total bit width of the result
 * 
 * @param [in]      I  The bit width of the integer portion of the result
 * 
 * @param [in]      S  indicates whether the representation encodes a sign bit or not
 * 
 * @param [in,out]  status A pointer to a status word that can be set to indicate
 *                    the success or failure of a conversion.
 *                  Instead of the traditional 0 or 1 return value, float32_To_AcFixed
 *                  increments the status word when an error occurs. By passing in
 *                  a pointer to an integer with zero value, status will contain the 
 *                  traditional zero or one at the completion of the call.
 *                  However, if desired, several calls to float32_To_AcFixed can be invoked 
 *                  in a row, passing in the same pointer each time such that 
 *                  the integer will be incremented each time an error occurs.
 *                  This allows the program to do a series of conversions and then check
 *                  the validity of the conversions as a group rather than individually.
 *                                    
 * @return          float32_To_AcFixed() returns an uint32 value containing the 
 *                  converted floating point number in the specified fixed point 
 *                  format.
 *                  
 * @retval          0 returned in the status word indicates a successful conversion. 
 * 
 * @retval          1 returned in the status word indicates an unsucessful conversion.
 *                    
 * @Constraints     W, I and S are not checked for consistency or range prior to use
 *                  so the caller must ensure they are appropriate values.
 *
 * @Reentrant       Yes
 */
//uint32_t float32_To_AcFixed(float fN, uint8_t W, uint8_t I, uint8_t S, int32_t *status);
int32_t float32_To_AcFixed(float fN, uint16_t format, int32_t *status);

#endif /* AC_FIXED_UTILS_H_ */
