/*
============================================================================
Name        : KMathUtilities.cpp
Author      : KEYnetik, Inc.
Version     :
Copyright   : Copyright 2010 by KEYnetik, Inc.
Description : definition for class MathUtilities 
============================================================================
*/

#include "KMathUtilities.h"

#include <math.h>
#include <limits.h>

#include "KSharedBuffer.h"

using namespace Keynetik;

CircularBuffer::Index 
MathUtilities::BufferSize(Fixed p_timeWindow)
{
	return CircularBuffer::Index((p_timeWindow * Fixed(AccelerometerReading::GetFrequency())).ToInt()+1);
}

CircularBuffer::Index 
MathUtilities::BufferSize(unsigned char p_timeWindow100thSec)
{
	return CircularBuffer::Index((unsigned int)p_timeWindow100thSec * (unsigned int)AccelerometerReading::GetFrequency() / 100 +1);
}

Fixed 
MathUtilities::TimeWindow(CircularBuffer::Index p_bufferSize)
{
    if( p_bufferSize < 1 )
    {
        return 0;
    }
    return Fixed(p_bufferSize)/Fixed(AccelerometerReading::GetFrequency());
}

Fixed 
MathUtilities::Average(const SharedBuffer& p_buffer, Axis p_axis, CircularBuffer::Index p_length) 
{
	if (p_length != 0 && p_buffer.IsFull(p_length)) 
	{
		Fixed sum(0);
		unsigned int count = 0;
		for( SharedBuffer::Iterator i=p_buffer.Begin(p_length); i.IsGood(); i.Next() ) 
		{ 
			sum+=i.GetValue().AxisValue(p_axis);
			count++;
		}
		if( count != 0 )
		{
			return sum / Fixed(count);
		}
	}
	return Fixed(0);
}

Fixed 
MathUtilities::MaxSpread(const SharedBuffer& p_buffer, Axis p_axis, CircularBuffer::Index p_length)
{
	if (p_length == 0 || !p_buffer.IsFull(p_length))
	{
		return Fixed(0);
	}

	SharedBuffer::Index minIdx=0;
	SharedBuffer::Index maxIdx=0;

	SharedBuffer::Iterator i = p_buffer.Begin(p_length);
	Fixed minVal=i.GetValue().AxisValue(p_axis);
	Fixed maxVal=minVal;
	CircularBuffer::Index count = 0;
	while(i.IsGood())
	{
		Fixed curVal = i.GetValue().AxisValue(p_axis);
		if (curVal < minVal)
		{
			minVal = curVal;
			minIdx = count;
		}
		else if (curVal > maxVal)
		{
			maxVal = curVal;
			maxIdx = count;
		}
		count++;
        i.Next();
	}

	// determine which of the minimum and maximum occurred first		
	if( minIdx > maxIdx )
	{
		return minVal - maxVal;
	}
	else
	{
		return maxVal - minVal;
	}
}

Fixed 
MathUtilities::CalcA(Fixed p_x, Fixed p_y, Fixed p_z)
{
	return ((p_x*p_x)+(p_y*p_y)+(p_z*p_z)).Sqrt();
}

double
MathUtilities::CalcA(double p_x, double p_y, double p_z)
{
	return sqrt((p_x*p_x)+(p_y*p_y)+(p_z*p_z));
}

Fixed 
MathUtilities::CalcA(const XYZ& p_xyz)
{
	return CalcA(p_xyz.X(), p_xyz.Y(), p_xyz.Z());
}

Fixed 
MathUtilities::RadToDegrees(Fixed p_rad)
{
    const Fixed pi(3.14159265358979323846);
    const Fixed piDegrees(180);
    return Fixed(p_rad*piDegrees/pi);
}

Fixed
MathUtilities::RotationAngle(Fixed p_a0, Fixed p_b0, Fixed p_a1, Fixed p_b1)
{   
    // protect from either vector being (0,0)
    Fixed deg0=MathUtilities::Arctan2(p_b0, p_a0);
    Fixed deg1=MathUtilities::Arctan2(p_b1, p_a1);
    if (deg0 == Fixed::MaxValue || deg1 == Fixed::MaxValue)
    {
        return Fixed::MaxValue;
    }

    Fixed ret = deg1 - deg0;
    // translate angle into [-180, 180]
    if (ret > KEYNETIK_FIXED_CONST(180))
    {
        ret=ret-KEYNETIK_FIXED_CONST(360);
    }
    else if (ret < KEYNETIK_FIXED_CONST(-179))
    {
        ret=ret+KEYNETIK_FIXED_CONST(360);
    }
    return ret;
}

Fixed
MathUtilities::DotProduct(Fixed p_x1, Fixed p_y1, Fixed p_z1, Fixed p_x2, Fixed p_y2, Fixed p_z2)
{
	return (p_x1*p_x2)+(p_y1*p_y2)+(p_z1*p_z2);
}

Fixed
MathUtilities::DotProduct(const XYZ& p_v1, const XYZ& p_v2)
{
	return DotProduct(p_v1.X(), p_v1.Y(), p_v1.Z(), p_v2.X(), p_v2.Y(), p_v2.Z());
}

Fixed 
MathUtilities::CosineOfAngle(Fixed p_x1, Fixed p_y1, Fixed p_z1, Fixed p_x2, Fixed p_y2, Fixed p_z2)
{
    return DotProduct(p_x1, p_y1, p_z1, p_x2, p_y2, p_z2) / CalcA(p_x1, p_y1, p_z1) / CalcA(p_x2, p_y2, p_z2);
}

Fixed 
MathUtilities::CosineOfAngle(const XYZ& p_v1, const XYZ& p_v2)
{
    return DotProduct(p_v1, p_v2) / CalcA(p_v1) / CalcA(p_v2);
}

void
MathUtilities::CrossProduct(const XYZ& p_v1, const XYZ& p_v2, XYZ& p_result)
{
    p_result.Set(p_v1.Y()*p_v2.Z() - p_v1.Z()*p_v2.Y(), 
                 p_v1.Z()*p_v2.X() - p_v1.X()*p_v2.Z(), 
                 p_v1.X()*p_v2.Y() - p_v1.Y()*p_v2.X());
}

unsigned int
MathUtilities::MinPowerOfTwoGreaterThan(unsigned char p_num)
{
    unsigned int ret = 1;
    while(ret <= p_num)
    {
        ret <<= 1;
        if(ret == 0) return UINT_MAX;   // prevent overflow
    }
    return ret;
}

Fixed 
MathUtilities::Arctan2(Fixed p_y, Fixed p_x)
{
    if (p_x == Fixed(0))
    {
        if (p_y == Fixed(0))
        {
            return Fixed::MaxValue;
        }
        return p_y > Fixed(0) ? Fixed(90) : Fixed(-90);
    }
    Fixed arctan= (p_y / p_x).Arctan();
    if (p_x > Fixed(0))
    {
        return arctan;
    }
    else if (p_y > Fixed(0))
    {   // x < 0 , y > 0, base arctan negative
        return Fixed(180)+arctan; 
    }
    else
    {   // x < 0 , y < 0, base arctan positive
        return Fixed(-180)+arctan;
    }
}

unsigned int 
MathUtilities::Sqrt(unsigned int p_num)
{
    unsigned int result = 0;
    unsigned int a = 1 << 15;
    do
    {
        unsigned int test = result + a;
        if (p_num >= test*test)
        {
            result += a;
        }
    } while (a >>= 1);
    return result;
}
