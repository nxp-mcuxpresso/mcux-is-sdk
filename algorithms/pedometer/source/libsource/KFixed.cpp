/*
============================================================================
 Name        : KFixed.cpp
 Author      : KEYnetik, Inc.
 Version     :
 Copyright   : Copyright 2010 by KEYnetik, Inc.
 Description : definition for Fixed class
============================================================================
*/

#include "KFixed.h"
#include "KTypes.h"

#include <limits.h>

#ifndef LONG_MAX 
#define LONG_MAX      2147483647L   /* maximum (signed) long value */
#endif

using namespace Keynetik;

static const unsigned int SignificantBits=31;
static const unsigned int TotalBits=SignificantBits+1;
static const unsigned int IntegerBits=SignificantBits-Fixed::FractionalBits;
static const long MostSignificantBit=(1 << (SignificantBits-1));
static const long One = 1<<Fixed::FractionalBits;

static const unsigned int IntegerMask	 = 0x7fffc000;
static const unsigned int FractionalMask = 0x00003fff;
static const unsigned int OverflowValue  = 0x0001ffff; // for the overflow check in Multiply

const Fixed Fixed::MaxValue((void*)LONG_MAX);
// NOTE: this is LONG_MIN+1, thus Fixed((void*)LONG_MIN) is outside of this type's range!
const Fixed Fixed::MinValue((void*)(-LONG_MAX)); 

Fixed::Fixed(double p_value)
: m_value((long)(p_value * (double)(1 << FractionalBits)))
{
}

double
Fixed::ToDouble() const
{
    return ((double)m_value) / (double)(1 << FractionalBits);
}

#define RemoveSigns(x, y, negativeResult) \
{\
    negativeResult=false;\
    if (x < 0)\
    {\
        x=-x;\
        negativeResult= !negativeResult;\
    }\
    if (y < 0)\
    {\
        y=-y;\
        negativeResult= !negativeResult;\
    }\
}

static
long 
Multiply(long x, long y)
{
    bool negative=false;
    RemoveSigns(x, y, negative);

	if (x <= One && y <= One) // no danger of overflow
	{
	    unsigned long ret= ((unsigned long)x * (unsigned long)y) >> Fixed::FractionalBits;
	    return negative ? -(long)ret : (long)ret;
	}
	
	unsigned long xI=(x & IntegerMask) >> Fixed::FractionalBits;
	unsigned long xF=x & FractionalMask;
	unsigned long yI=(y & IntegerMask) >> Fixed::FractionalBits;
	unsigned long yF=y & FractionalMask;

    unsigned long temp = xI * yI;
	if (temp <= OverflowValue)
	{
    	unsigned long result = temp << Fixed::FractionalBits;
        temp = xI * yF;
    	result += temp;
    	if (result <= LONG_MAX)
	    {
            temp = xF * yI;
	        result += temp;
        	if (result <= LONG_MAX)
    	    {
                temp = ((xF * yF) >> Fixed::FractionalBits);
	            result += temp;
            	if (result <= LONG_MAX) 
            	{
                    return negative ? -(long)result : (long)result;
            	}
    	    }
	    }
	}
	// overflow
    return negative ? -LONG_MAX : LONG_MAX;
}

Fixed 
Fixed::operator *(Fixed p_other) const
{
    Fixed ret;
    ret.m_value=Multiply(m_value, p_other.m_value);
	return ret;
}

Fixed& 
Fixed::operator *=(Fixed p_other) 
{ 
    m_value=Multiply(m_value, p_other.m_value);
	return *this;
}

Fixed& 
Fixed::Square() 
{
    m_value=Multiply(m_value, m_value);
	return *this;
}

static
long
Divide(long x, long y) // y <> 0
{
    bool negative=false;
    RemoveSigns(x, y, negative);

    // increase x as much as possible for maximum accuracy and remember the number of bits shifted by
    unsigned char xShiftedBy=0;
    while (x && !(x & MostSignificantBit))
    {
        x <<= 1;
        ++xShiftedBy;
    }

    long ret= x / y;

    // Normally, a quotiend would have to be shifted left by FractionalBits. 
    // Account for the shifting of x we did before division. This may result in shifting the result left or right.
    signed char diff=(signed char)(Fixed::FractionalBits - xShiftedBy);
    if (diff > 0)
    {   // careful to detect an overflow
        while (diff && !(ret & MostSignificantBit))
        {
            ret <<= 1;
            --diff;
        }
        if (diff) // the shifted result does not fit 
        {
            return negative ? -LONG_MAX : LONG_MAX;
        }
    }
    else if (diff < 0)
    {
        ret >>= -diff;
    }

    return negative ? -ret : ret;
}

Fixed 
Fixed::operator /(Fixed p_other) const
{
    if (p_other.m_value == 0)
    {
        return m_value >= 0 ? MaxValue : MinValue;
    }
	Fixed ret;
	ret.m_value = Divide(m_value, p_other.m_value);
    return ret;
}

Fixed 
Fixed::Sqrt() const
{
	if (m_value <= 0)
    {
		return 0;
    }

    static const long maxError =  1 << (FractionalBits-11); // ~0.0005    
    long hi;
    long lo;

    if( m_value <= One )
    {
        hi = One;
        lo = m_value;
    }
    else
    {
        hi = m_value;
        lo = One;
    }

    long mid = (hi + lo) >> 1;
    long mid2 = Multiply(mid,mid);
	long delta = ::Abs(m_value - mid2);
  
    while( (hi-lo) > maxError && delta > maxError ) 
    {
        if( mid2 == LONG_MAX || mid2 == -LONG_MAX )
        {
            hi = mid;
        }
        else if( mid2 < m_value ) 
        {
            lo = mid;
        }
        else
        {
            hi = mid;
        }
        mid = (hi + lo) >> 1;
        mid2 = Multiply(mid,mid);
        delta = ::Abs(m_value - mid2);
    }

    if( delta > maxError )
    {
        long deltaHi = ::Abs(m_value - Multiply(hi,hi));
        long deltaLo = ::Abs(m_value - Multiply(lo,lo));
        if( deltaHi <= deltaLo && deltaHi < delta )
        {
            mid = hi;
        }
        else if( deltaLo < deltaHi && deltaLo < delta )
        {
            mid = lo;
        }        
    }

	return Fixed((void*)mid);
}

int 
Fixed::Round() const
{
    Fixed pos(Abs());
    pos.m_value &= FractionalMask;
    int ret=ToInt();
    const Fixed half(KEYNETIK_FIXED_CONST(0.5));
    if (pos.m_value >= half.m_value)
    {
        return m_value < 0 ? ret : (ret+1);
    }
    else
    {
        return m_value < 0 ? (ret+1) : ret;
    }
}

// LookUp Table based methods

// binary search in a table of values
static Fixed Lookup(const Fixed* p_table, unsigned int p_maxIndex, long p_v)
{
    unsigned int lo=0;
    unsigned int hi=p_maxIndex;
    unsigned int i=p_maxIndex / 2;
    while (i != lo)
    {
        if (p_table[i].GetInternal() > p_v)
        {
            lo=i;
        }
        else if (p_table[i].GetInternal() < p_v)
        {
            hi=i;
        }
        else
        {
            break;
        }
        i=lo+(hi-lo)/2;
    }
    return Fixed(i);
}

Fixed 
Fixed::Arccos() const
{   

static Fixed ArccosTable[]={ // 0 to 90 degrees
    KEYNETIK_FIXED_CONST(1.0000),
    KEYNETIK_FIXED_CONST(0.9998),
    KEYNETIK_FIXED_CONST(0.9994),
    KEYNETIK_FIXED_CONST(0.9986),
    KEYNETIK_FIXED_CONST(0.9976),
    KEYNETIK_FIXED_CONST(0.9962),
    KEYNETIK_FIXED_CONST(0.9945),
    KEYNETIK_FIXED_CONST(0.9925),
    KEYNETIK_FIXED_CONST(0.9903),
    KEYNETIK_FIXED_CONST(0.9877),
    KEYNETIK_FIXED_CONST(0.9848),
    KEYNETIK_FIXED_CONST(0.9816),
    KEYNETIK_FIXED_CONST(0.9781),
    KEYNETIK_FIXED_CONST(0.9744),
    KEYNETIK_FIXED_CONST(0.9703),
    KEYNETIK_FIXED_CONST(0.9659),
    KEYNETIK_FIXED_CONST(0.9613),
    KEYNETIK_FIXED_CONST(0.9563),
    KEYNETIK_FIXED_CONST(0.9511),
    KEYNETIK_FIXED_CONST(0.9455),
    KEYNETIK_FIXED_CONST(0.9397),
    KEYNETIK_FIXED_CONST(0.9336),
    KEYNETIK_FIXED_CONST(0.9272),
    KEYNETIK_FIXED_CONST(0.9205),
    KEYNETIK_FIXED_CONST(0.9135),
    KEYNETIK_FIXED_CONST(0.9063),
    KEYNETIK_FIXED_CONST(0.8988),
    KEYNETIK_FIXED_CONST(0.8910),
    KEYNETIK_FIXED_CONST(0.8829),
    KEYNETIK_FIXED_CONST(0.8746),
    KEYNETIK_FIXED_CONST(0.8660),
    KEYNETIK_FIXED_CONST(0.8572),
    KEYNETIK_FIXED_CONST(0.8480),
    KEYNETIK_FIXED_CONST(0.8387),
    KEYNETIK_FIXED_CONST(0.8290),
    KEYNETIK_FIXED_CONST(0.8192),
    KEYNETIK_FIXED_CONST(0.8090),
    KEYNETIK_FIXED_CONST(0.7986),
    KEYNETIK_FIXED_CONST(0.7880),
    KEYNETIK_FIXED_CONST(0.7771),
    KEYNETIK_FIXED_CONST(0.7660),
    KEYNETIK_FIXED_CONST(0.7547),
    KEYNETIK_FIXED_CONST(0.7431),
    KEYNETIK_FIXED_CONST(0.7314),
    KEYNETIK_FIXED_CONST(0.7193),
    KEYNETIK_FIXED_CONST(0.7071),
    KEYNETIK_FIXED_CONST(0.6947),
    KEYNETIK_FIXED_CONST(0.6820),
    KEYNETIK_FIXED_CONST(0.6691),
    KEYNETIK_FIXED_CONST(0.6561),
    KEYNETIK_FIXED_CONST(0.6428),
    KEYNETIK_FIXED_CONST(0.6293),
    KEYNETIK_FIXED_CONST(0.6157),
    KEYNETIK_FIXED_CONST(0.6018),
    KEYNETIK_FIXED_CONST(0.5878),
    KEYNETIK_FIXED_CONST(0.5736),
    KEYNETIK_FIXED_CONST(0.5592),
    KEYNETIK_FIXED_CONST(0.5446),
    KEYNETIK_FIXED_CONST(0.5299),
    KEYNETIK_FIXED_CONST(0.5150),
    KEYNETIK_FIXED_CONST(0.5000),
    KEYNETIK_FIXED_CONST(0.4848),
    KEYNETIK_FIXED_CONST(0.4695),
    KEYNETIK_FIXED_CONST(0.4540),
    KEYNETIK_FIXED_CONST(0.4384),
    KEYNETIK_FIXED_CONST(0.4226),
    KEYNETIK_FIXED_CONST(0.4067),
    KEYNETIK_FIXED_CONST(0.3907),
    KEYNETIK_FIXED_CONST(0.3746),
    KEYNETIK_FIXED_CONST(0.3584),
    KEYNETIK_FIXED_CONST(0.3420),
    KEYNETIK_FIXED_CONST(0.3256),
    KEYNETIK_FIXED_CONST(0.3090),
    KEYNETIK_FIXED_CONST(0.2924),
    KEYNETIK_FIXED_CONST(0.2756),
    KEYNETIK_FIXED_CONST(0.2588),
    KEYNETIK_FIXED_CONST(0.2419),
    KEYNETIK_FIXED_CONST(0.2250),
    KEYNETIK_FIXED_CONST(0.2079),
    KEYNETIK_FIXED_CONST(0.1908),
    KEYNETIK_FIXED_CONST(0.1736),
    KEYNETIK_FIXED_CONST(0.1564),
    KEYNETIK_FIXED_CONST(0.1392),
    KEYNETIK_FIXED_CONST(0.1219),
    KEYNETIK_FIXED_CONST(0.1045),
    KEYNETIK_FIXED_CONST(0.0872),
    KEYNETIK_FIXED_CONST(0.0698),
    KEYNETIK_FIXED_CONST(0.0523),
    KEYNETIK_FIXED_CONST(0.0349),
    KEYNETIK_FIXED_CONST(0.0175),
    KEYNETIK_FIXED_CONST(0.0000),
};

    if (m_value < 0)
    {   // negative
        Fixed ret(*this);
        ret.m_value = -ret.m_value;
        if (ret.m_value > One)
        {
            return Fixed::MinValue;
        }
        ret=Lookup(ArccosTable, 90, ret.m_value);
        if (ret == MaxValue)
        {
            ret=MinValue;
        }
        else
        {
            ret= Fixed(180) - ret;
        }
        return ret;
    }

    // positive 
    if (m_value > One)
    {
        return Fixed::MaxValue;
    }
    if (m_value == 0)
    {
        return Fixed(90);
    }
    return Lookup(ArccosTable, 90, m_value);
}

Fixed 
Fixed::Arctan() const
{

static Fixed ArctanTable[]={ // 45 to 0 degrees
    KEYNETIK_FIXED_CONST(1.0000),
    KEYNETIK_FIXED_CONST(0.9656),
    KEYNETIK_FIXED_CONST(0.9325),
    KEYNETIK_FIXED_CONST(0.9004),
    KEYNETIK_FIXED_CONST(0.8692),
    KEYNETIK_FIXED_CONST(0.8390),
    KEYNETIK_FIXED_CONST(0.8097),
    KEYNETIK_FIXED_CONST(0.7812),
    KEYNETIK_FIXED_CONST(0.7535),
    KEYNETIK_FIXED_CONST(0.7265),
    KEYNETIK_FIXED_CONST(0.7002),
    KEYNETIK_FIXED_CONST(0.6745),
    KEYNETIK_FIXED_CONST(0.6494),
    KEYNETIK_FIXED_CONST(0.6248),
    KEYNETIK_FIXED_CONST(0.6008),
    KEYNETIK_FIXED_CONST(0.5773),
    KEYNETIK_FIXED_CONST(0.5543),
    KEYNETIK_FIXED_CONST(0.5317),
    KEYNETIK_FIXED_CONST(0.5095),
    KEYNETIK_FIXED_CONST(0.4877),
    KEYNETIK_FIXED_CONST(0.4663),
    KEYNETIK_FIXED_CONST(0.4452),
    KEYNETIK_FIXED_CONST(0.4244),
    KEYNETIK_FIXED_CONST(0.4040),
    KEYNETIK_FIXED_CONST(0.3838),
    KEYNETIK_FIXED_CONST(0.3639),
    KEYNETIK_FIXED_CONST(0.3443),
    KEYNETIK_FIXED_CONST(0.3249),
    KEYNETIK_FIXED_CONST(0.3057),
    KEYNETIK_FIXED_CONST(0.2867),
    KEYNETIK_FIXED_CONST(0.2679),
    KEYNETIK_FIXED_CONST(0.2493),
    KEYNETIK_FIXED_CONST(0.2308),
    KEYNETIK_FIXED_CONST(0.2125),
    KEYNETIK_FIXED_CONST(0.1943),
    KEYNETIK_FIXED_CONST(0.1763),
    KEYNETIK_FIXED_CONST(0.1583),
    KEYNETIK_FIXED_CONST(0.1405),
    KEYNETIK_FIXED_CONST(0.1227),
    KEYNETIK_FIXED_CONST(0.1051),
    KEYNETIK_FIXED_CONST(0.0874),
    KEYNETIK_FIXED_CONST(0.0699),
    KEYNETIK_FIXED_CONST(0.0524),
    KEYNETIK_FIXED_CONST(0.0349),
    KEYNETIK_FIXED_CONST(0.0174),
    KEYNETIK_FIXED_CONST(0.0000),
};

    if (m_value < 0)
    {   // negative
        if (m_value == MinValue.m_value)
        {
            return Fixed(-90);
        }
        if (m_value < -One)
        {
            return Fixed(-45)-Lookup(ArctanTable, 45, Divide(One, -m_value));
        }
        return Fixed(-45)+Lookup(ArctanTable, 45, -m_value);
    }

    if (m_value == 0)
    {
        return Fixed(0);
    }

    // positive
    if (m_value == MaxValue.m_value)
    {
        return Fixed(90);
    }
    if (m_value > One)
    {
        return Fixed(45)+Lookup(ArctanTable, 45, Divide(One, m_value));
    }
    return Fixed(45)-Lookup(ArctanTable, 45, m_value);
}
