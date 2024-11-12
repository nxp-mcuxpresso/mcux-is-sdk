/*
 * Copyright (c) 2015 - 2016, Freescale Semiconductor, Inc.
 * Copyright 2016-2017 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/*! File: convert.h
* @brief The \b convert.h file containes the definitions for the sensor data
*        conversion utility functions.
*/

#ifndef CONVERT_H_
#define CONVERT_H_

// Convert should only be used to handle sample values not things like
// booleans, enumerations, etc.
// I.e., the value must represent a value in engineering units that can be
// used in counts, fixed point, or floating point representations.
//
// To convert a value one must know:
// 1. The endianness of the bytes to be used
// 2. The justification of any bits in bytes that are not fully used.
// 3. The value of the LSB in engineering units per bit.
// 4. whether the unused bits in a byte can be used or need to be cleared.
//
// Assumptions made:
// 1. Any raw value will be 32 bits or less in size.
//
//  It may be useful to represent endianness as 32bit hex number
//  where each byte's number can be used to locate it in the raw byte
//  buffer.
//  bytes can be numbered 0..3  (or 1..4 to avoid zero??)
//  with lower byte numbers for lower significance.
//  Thus the value 0x12345678 that was stored in a byte buffer as:
//   0000: 0x78
//   0001: 0x56
//   0002: 0x34
//   0003: 0x12
// could be have its endianness stored as: 0x3210
// And stored like:
//   0000: 0x56
//   0001: 0x78
//   0002: 0x12
//   0003: 0x34
// could be have its endianness stored as: 0x2301
//
// A 16 bit value could use 'F' for unused bytes:
//
//  The value 0x1234 that was stored in a byte buffer as:
//   0000: 0x34
//   0001: 0x12
// could be have its endianness stored as: 0xFF10

// An optional bit mask could be used after byte ordering to mask out unwanted bits
//
// An optional signed shift value could be used to justify or scale the result
//
// An optional floating point multiplier can be used to scale the result.
//
// An optional number of bytes to skip in the output buffer can be used for byte
// alignment purposes.
//
// An optional fixed point representation can be used to convert a float to
// the specified fixed point format. The fixed point number can be
// specified in AC_FIXED Format (e.g. 32w16i1s) which can be encoded in a
// 16 bit integer value as s000wwwwwwiiiiii
#define AC_FORMAT(W,I,S) ((uint16_t)((((W)&0x003F)<<6)|((I)&0x003F)|(((S)&0x0001)<<15)))
#define ENCODE_SIGN_EXT_INFO(W,S) ((uint32_t)(W<<16)|(S))

//
// The complete conversion specifier should be a list of these parameters
// that the converter can loop through to effect the conversion.
//
//The conversion specifier for a buffer of values should be a list of
//individual conversion specifiers.
//  A single specifier therefore should be made up of a sequence of conversion
//  primitive / value pairs.
//
//  There are:
//    1. conversion primitives
//    2. conversion specifiers
//    3. buffer conversion specifiers
//
//  The primitive types are:
typedef enum EConversionPrimitives {
   CONVERSION_END             = 0,
   CONVERSION_BYTES           = 1,
   CONVERSION_ENDIANNESS      = 2,
   CONVERSION_MASK            = 3,
   CONVERSION_LSHIFT          = 4,
   CONVERSION_RSHIFT          = 5,
   CONVERSION_RSHIFT_SE       = 6,
   CONVERSION_INT_ADD         = 7,
   CONVERSION_INT_MULTIPLY    = 8,
   CONVERSION_FLOAT_CAST      = 9,
   CONVERSION_FLOAT_ADD       = 10,
   CONVERSION_FLOAT_MULTIPLY  = 11,
   CONVERSION_FIXED_POINT     = 12,
   CONVERSION_SKIP_BYTES      = 13,
   CONVERSION_PAD_BYTES       = 14
} ConversionOperation_t;
//
// Conversion macros are defined to make usage easier:

#define CONV_SKIP_BYTES(b)     { .op=CONVERSION_SKIP_BYTES,     .value.numBytes=(b)                      }
#define CONV_BYTES(b)          { .op=CONVERSION_BYTES,          .value.numBytes=(b)                      }
#define CONV_ENDIANNESS(e)     { .op=CONVERSION_ENDIANNESS,     .value.endianness=(e)                    }
#define CONV_MASK(m)           { .op=CONVERSION_MASK,           .value.mask=(m)                          }
#define CONV_LSHIFT(s)         { .op=CONVERSION_LSHIFT,         .value.shift=(s)                         }
#define CONV_RSHIFT(s)         { .op=CONVERSION_RSHIFT,         .value.shift=(s)                         }
#define CONV_RSHIFT_SE(w,s)    { .op=CONVERSION_RSHIFT_SE,      .value.shift=(ENCODE_SIGN_EXT_INFO(w,s)) }
#define CONV_INT_ADD(o)        { .op=CONVERSION_INT_ADD,        .value.i_val=(o)                         }
#define CONV_INT_MULTIPLY(m)   { .op=CONVERSION_INT_MULTIPLY,   .value.i_val=(m)                         }
#define CONV_FLOAT_CAST(w)     { .op=CONVERSION_FLOAT_CAST,     .value.shift=(w)                         }
#define CONV_FLOAT_ADD(o)      { .op=CONVERSION_FLOAT_ADD,      .value.f_val=(o)                         }
#define CONV_FLOAT_MULTIPLY(m) { .op=CONVERSION_FLOAT_MULTIPLY, .value.f_val=(m)                         }
#define CONV_FIXED_POINT(w,i,s){ .op=CONVERSION_FIXED_POINT,    .value.format=(AC_FORMAT(w,i,s))         }
#define CONV_PAD_BYTES(b)      { .op=CONVERSION_PAD_BYTES,      .value.numBytes=(b)                      }
#define CONV_COMPLETE          { .op=CONVERSION_END,            .value.format=0                          }
#define CONV_BUFFER_COMPLETE   { .op=CONVERSION_END,            .value.format=0                          }

//
//  A conversion specifier is of type:
typedef struct {
    ConversionOperation_t  op;
    union {
        uint32_t  endianness;
        uint32_t  mask;
        int32_t   shift;
        uint32_t  numBytes;
        uint32_t  format;
        int32_t   i_val;
        float     f_val;
    } value;
} ConversionSpecifier_t;

//
//  Suppose a sensor register setup like as:
//      |  Reg 0   |   Reg 1  |  Reg 2   |  Reg 3   |  Reg 4   |   Reg 5  |  Reg 6   |
//      | 76543210 | 76543210 | 76543210 | 76543210 | 76543210 | 76543210 | 76543210 |
//      |  STATUS  |OUT_X_MSB |OUT_X_LSB |OUT_Y_MSB |OUT_Y_LSB |OUT_Z_MSB |OUT_Z_LSB |
//      | rrrrrrrr | bbbbbbbb | bbbbbb00 | bbbbbbbb | bbbbbb00 | bbbbbbbb | bbbbbb00 |
//
//  The data sheet says 14 bit left justified samples
//  In "2G" mode LSB is 0.244 mg/LSB  (note that this applies to bit 2 not bit 0)
//
//
//  A complete buffer specifier for obtaining floating point XYZ data from the above registers
//  might look like this:
//
//  Suppose the receiving type is:
//	typedef struct {
//	   float accel_x;
//	   float accel_y;
//	   float accel_z;
//	} fAccelData_t;
//
// A Conversion specifier can be defined to produce fAccelData_t from a buffer containing
// values for Reg 0 through Reg 6
//
// First create a conversion spec for a single accel float value:
//
//	#define FXOS8700_2G_RAW16_TO_FLOAT_ACCEL
//		   CONV_BYTES(3),
//		   CONV_ENDIANNESS(0x012),
//		   CONV_RSHIFT_SE(16,2),
//		   CONV_FLOAT_CAST(32),
//		   CONV_FLOAT_GAIN(0.244/1000.0),
//		   CONV_COMPLETE
//
//    This is decoded as:
//		   CONV_BYTES(2)                    /* Take 2 bytes from the input buffer */
//		   CONV_ENDIANNESS(0x10)            /*  Create a multi-byte integer by placing the bytes in the specified order
//                                                      buffer bytes are 0 indexed and the specifier uses the buffer indices
//                                                      in left to right order from MSB to LSB. */
//		   CONV_RSHIFT_SE(16,2))            /*  Right shift 2 places with sign extension based on a 16 bit quantity*/
//		   CONV_FLOAT_CAST(32)              /*  Cast the 32-integer to a float value */
//		   CONV_FLOAT_GAIN(0.244/1000.0)    /*  Multiply by 0.000244 to convert to Gs  (0.244 mg/Count)*/
//		   CONV_COMPLETE                    /*  write the result to the output buffer */
//
//    a Complete buffer conversion specifier can now be created:
//
//  const ConversionSpecifier_t FXOS8700_ACCEL_FLOAT_CONVERSION[] =
//		{
//		   CONV_SKIP_BYTES(1),
//		   FXOS8700_2G_RAW16_TO_FLOAT_ACCEL,
//		   FXOS8700_2G_RAW16_TO_FLOAT_ACCEL,
//		   FXOS8700_2G_RAW16_TO_FLOAT_ACCEL,
//		   CONV_BUFFER_COMPLETE
//		};
//
//   The buffer specifier can:
//      skip over the contents of Reg 0   ( CONV_SKIP_BYTES(1)  )
//      convert 3 16 bit accel values to float using FXOS8700_2G_RAW16_TO_FLOAT_ACCEL
//      then end the conversion.  ( CONV_BUFFER_COMPLETE )
//

typedef enum EConvertTypes {
    CONV_TYPE_COUNTS = 0,
    CONV_TYPE_FIXED  = 1,
    CONV_TYPE_FLOAT  = 2
} ConvertTypes_t;

typedef struct {
    uint8_t      size;
    union {
       uint32_t  i;
       int32_t   si;
       float     f;
    } v;
} Conversion_t;

#define SUCCESS   ( 1)
#define BAD_PARAM (-1)

int32_t Convert_Sensor_Data(const ConversionSpecifier_t *convSpec, uint8_t *rawBuffer, void *result);

#endif /* CONVERT_H_ */
