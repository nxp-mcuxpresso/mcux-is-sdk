#ifndef KMathUtilities_H
#define KMathUtilities_H
/*
============================================================================
Name        : KMathUtilities.h
Author      : KEYnetik, Inc.
Version     :
Copyright   : Copyright 2010 by KEYnetik, Inc.
Description : declarations for class MathUtilities
============================================================================
*/

// need this to enable M_PI
#define _USE_MATH_DEFINES
#include <cmath>
#include <cstdlib>

#include "KTypes.h"
#include "KFixed.h"
#include "KCircularBufferConstIterator.h"

namespace Keynetik
{
class XYZ;
class SharedBuffer;

/**
* Collection of math utility functions.
*/
class MathUtilities
{
   public:
    /**
    * Returns the length of a buffer needed to store the readings generated
    * in the time window specified (based on AccelerometerReading::GetFrequency)
    * \param p_timeWindow time window in seconds
    * \return number of readings
    */
    static CircularBuffer::Index BufferSize(Fixed p_timeWindow);
    /**
    * Returns the length of a buffer needed to store the readings generated
    * in the time window specified (based on AccelerometerReading::GetFrequency)
    * \param p_timeWindow time window in 100ths of a second
    * \return number of readings
    */
    static CircularBuffer::Index BufferSize(unsigned char p_timeWindow100thSec);

    /**
    * Returns the time window in seconds corresponding to the given length of a buffer
    * (based on AccelerometerReading::GetFrequency)
    * \param p_bufferSize number of readings in buffer
    * \return time window in seconds
    */
    static Fixed TimeWindow(CircularBuffer::Index p_bufferSize);

    /**
    * Average of a specified number of latest readings on an axis in a shared buffer of accelerometer readings.
    * \param p_buffer shared buffer
    * \param p_axis axis
    * \param p_length the number of latest values to average over
    * \return average value
    */
    static Fixed Average(const SharedBuffer &p_buffer, Axis p_axis, CircularBuffer::Index p_length);
    /**
    * Distance between minimum and maximum readings on an axis in a shared buffer of accelerometer readings.
    * \param p_buffer shared buffer
    * \param p_axis axis
    * \param p_length the number of latest values to average over
    * \return Max - Min (positive) if minimum occurred before maximum, Min - Max (negative) otherwise
    */
    static Fixed MaxSpread(const SharedBuffer &p_buffer, Axis p_axis, CircularBuffer::Index p_length);

    /**
    * Calculate acceleration value based on 3 axis readings (length of a 3D vector represented by axis values)
    * \param p_x X reading
    * \param p_y Y reading
    * \param p_z Z reading
    * \return acceleration, sqrt(x*x+y*y+z*z)
    */
    static Fixed CalcA(Fixed p_x, Fixed p_y, Fixed p_z);
    static double CalcA(double p_x, double p_y, double p_z);

    /**
    * Calculate acceleration value based on 3 axis readings (length of a 3D vector represented by axis values)
    * \param p_xyx X, Y, Z readings
    * \return acceleration, sqrt(x*x+y*y+z*z)
    */
    static Fixed CalcA(const XYZ &p_xyz);

    /**
          * Converts an angle value from radians to degrees
          * \param p_rad angle in radians
          * \return angle in degrees, from -179 to 180
          */
    static Fixed RadToDegrees(Fixed p_rad);

    /**
          * Calculate angle of rotation between 2D vectors, original (a0, b0) and new (a1, b1), in degrees (-179..180)
          * Positive result corresponds to counterclockwise rotation
          * \param p_a0 the X coordinate of the original vector
          * \param p_b0 the Y coordinate of the original vector
          * \param p_a1 the X coordinate of the new vector
          * \param p_b1 the Y coordinate of the new vector
          * \return angle of rotation from the original to the new vector
          */
    static Fixed RotationAngle(Fixed p_a0, Fixed p_b0, Fixed p_a1, Fixed p_b1);

    /**
    * Calculate dot product of 2 vectors of 3 values
    * \param p_x1 X component of vector 1
    * \param p_y1 Y component of vector 1
    * \param p_z1 Z component of vector 1
    * \param p_x2 X component of vector 2
    * \param p_y2 Y component of vector 2
    * \param p_z2 Z component of vector 2
    * \return dot product: x1*x2+y1*y2+z1*z2
    */
    static Fixed DotProduct(Fixed p_x1, Fixed p_y1, Fixed p_z1, Fixed p_x2, Fixed p_y2, Fixed p_z2);
    /**
    * Calculate dot product of 2 vectors of 3 values
    * \param p_v1 vector 1
    * \param p_v2 vector 1
    * \return dot product: x1*x2+y1*y2+z1*z2
    */
    static Fixed DotProduct(const XYZ &p_v1, const XYZ &p_v2);

    /**
    * Calculate cosine of angle between 2 vectors of 3 values
    * \param p_x1 X component of vector 1
    * \param p_y1 Y component of vector 1
    * \param p_z1 Z component of vector 1
    * \param p_x2 X component of vector 2
    * \param p_y2 Y component of vector 2
    * \param p_z2 Z component of vector 2
    * \return cosine of angle between vectors: dotProduct(v1, v2)/(|v1|*|v2|)
    */
    static Fixed CosineOfAngle(Fixed p_x1, Fixed p_y1, Fixed p_z1, Fixed p_x2, Fixed p_y2, Fixed p_z2);
    static Fixed CosineOfAngle(const XYZ &p_v1, const XYZ &p_v2);

    /**
    * Calculate a cross product of 2 vectors of 3 values
    * \param p_v1 vector 1
    * \param p_v2 vector 2
    * \param p_result cross product vector: {y1*z2-y2*z1, x2*z1-x1*z2, x1*y2-x2*y1}. Normalization is not performed.
    */
    static void CrossProduct(const XYZ &p_v1, const XYZ &p_v2, XYZ &p_result);

    /**
    * Calculates minimum power of 2 that is greater than the parameter.
    * \param p_num the parameter
    * \return min power of 2 greater than parameter. If most significant bit of parameter is 1, returns UINT_MAX
    */
    static unsigned int MinPowerOfTwoGreaterThan(unsigned char p_num);

    static Fixed Arctan2(Fixed p_y, Fixed p_x); // in degrees from -180 to 180

    /**
    * Calculates square root of an integer, truncated to the nearest integer.
    * \param p_num the parameter
    * \return highest integer whose square is less than or equal to the parameter
    */
    static unsigned int Sqrt(unsigned int p_num);

#ifndef KEYNETIK_EMBEDDED
    /**
    * Correlation function between segments of 2 circular buffers of Fixed values, with an optional phase shift.
    * See http://en.wikipedia.org/wiki/Pearson_product-moment_correlation_coefficient
    * NOTE: If there is more data in buffer than needed (p_range + abs(p_shift)), computations will include the latest
    * data in the buffers
    * \param p_buf1 buffer 1
    * \param p_buf2 buffer 2
    * \param p_range size of the segment, in values
    * \param p_shift phase shift: p_buf1[i] is correlated to p_buf2[i+p_shift] (can be negative)
    * \return correlation between 2 series, or Fixed::MinValue if not enough data
    */
    template <unsigned short size>
    static Fixed Correl(TCircularBuffer<size, Fixed> p_buf1,
                        TCircularBuffer<size, Fixed> p_buf2,
                        CircularBuffer::Index p_range,
                        int p_shift = 0)
    {
        typedef TCircularBufferConstIterator<size, Fixed> ConstIterator;

        if (p_buf1.GetOccupied() == p_buf2.GetOccupied())
        {
            short sizeDiff = p_buf1.GetOccupied() - (p_range + abs(p_shift));
            if (sizeDiff >= 0)
            {
                ConstIterator it1(p_buf1, p_shift > 0 ? sizeDiff : (abs(p_shift) + sizeDiff));
                ConstIterator it2(p_buf2, p_shift > 0 ? (p_shift + sizeDiff) : sizeDiff);
                double avg1(p_buf1.Average(it1.GetIndex(), p_range).ToDouble());
                double avg2(p_buf2.Average(it2.GetIndex(), p_range).ToDouble());
                double devSum = 0.0;
                double devSq1 = 0.0;
                double devSq2 = 0.0;
                for (unsigned int i = 0; i < p_range; ++i)
                {
                    if (!it1.IsGood() || !it2.IsGood())
                    {
                        return Fixed::MinValue;
                    }
                    double dev1(it1.GetValue().ToDouble() - avg1);
                    double dev2(it2.GetValue().ToDouble() - avg2);
                    devSum += dev1 * dev2;
                    devSq1 += dev1 * dev1;
                    devSq2 += dev2 * dev2;
                    it1.Next();
                    it2.Next();
                }
                return Fixed(devSum / sqrt(devSq1 * devSq2));
            }
        }
        return Fixed::MinValue;
    }
#endif

   private:
    MathUtilities();
    MathUtilities(const MathUtilities &);
    MathUtilities &operator=(const MathUtilities &);
    ~MathUtilities();
};
}

#endif
