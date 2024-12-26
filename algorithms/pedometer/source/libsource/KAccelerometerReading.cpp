/*
============================================================================
 Name        : KAccelerometerReading.cpp
 Author      : KEYnetik, Inc.
 Version     :
 Copyright   : Copyright 2008 by KEYnetik, Inc.
 Description : AccelerometerReading definition
============================================================================
*/

#include "KAccelerometerReading.h"
#include "KContext.h"
#include "KMathUtilities.h"

using namespace Keynetik;

Fixed AccelerometerReading::m_1G(1);
unsigned short AccelerometerReading::m_frequency;
unsigned int AccelerometerReading::m_nextID = 0;

bool 
AccelerometerReading::Set1G(Fixed p_val)
{
	if( p_val != Fixed(0) )
	{
		m_1G = p_val;
#ifndef KEYNETIK_EMBEDDED
		Context::AccelerometerReset();
#endif
		return true;
	}
	return false;
}

bool
AccelerometerReading::SetFrequency(unsigned short p_val)
{
	if( p_val >= MinFrequency && p_val <= MaxFrequency) 
	{
		m_frequency = p_val;
#ifndef KEYNETIK_EMBEDDED
		Context::AccelerometerReset();
#endif
		return true;
	}
	return false;
}

AccelerometerReading::AccelerometerReading() 
{
    a = Fixed(0);
#ifndef KEYNETIK_EMBEDDED
    id = m_nextID;
	m_nextID++; // TBD: catch overflow
#endif
}

AccelerometerReading::AccelerometerReading(const AccelerometerReading& source)
{
    m_xyz = source.GetXYZ();
    a = Fixed::MaxValue;
    id = source.GetReadingID();
}

AccelerometerReading::AccelerometerReading(Fixed p_x, Fixed p_y, Fixed p_z, bool normalized/*=false*/)
{
    Set(p_x, p_y, p_z, normalized);
#ifndef KEYNETIK_EMBEDDED
	id = m_nextID;
	m_nextID++; // TBD: catch overflow
#endif
}

AccelerometerReading& 
AccelerometerReading::operator=(const AccelerometerReading& source)
{
    m_xyz=source.GetXYZ();
    a=source.a;
    id=source.id;
    return *this;
}


void
AccelerometerReading::Clear()
{
    Set(0, 0, 0, true);
}

Fixed
AccelerometerReading::AxisValue(Axis p_axis) const
{
    switch(p_axis)
    {
    case AxisX: return m_xyz.X();
    case AxisY: return m_xyz.Y();
    case AxisZ: return m_xyz.Z();
    case AxisA: return A();
    default: break;
    };
    return Fixed(0);
}

void 
AccelerometerReading::Set(Fixed p_x, Fixed p_y, Fixed p_z, bool normalized/*=false*/)
{
	if( !normalized && m_1G != Fixed(1) )
	{
        m_xyz.Set(p_x/m_1G, p_y/m_1G, p_z/m_1G);
	}
	else
	{
        m_xyz.Set(p_x, p_y, p_z);
	}
    a = Fixed::MaxValue;
}

Fixed
AccelerometerReading::A() const
{
    if( a == Fixed::MaxValue )
    {
        a = MathUtilities::CalcA(m_xyz.X(), m_xyz.Y(), m_xyz.Z());
    }
    return a;
}

