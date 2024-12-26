/*
============================================================================
 Name        : KSharedBuffer.cpp
 Author      : KEYnetik, Inc.
 Version     :
 Copyright   : Copyright 2010 by KEYnetik, Inc.
 Description : definition of class SharedBuffer
============================================================================
*/

#include "KSharedBuffer.h"

using namespace Keynetik;

const SharedBuffer::Iterator SharedBuffer::END((const SharedBuffer&)SharedBuffer(0,0));

SharedBuffer::SharedBuffer(unsigned int p_length, AccelerometerReading* const p_readings) 
:   m_readings(p_readings),
	m_curIndex(0), 
    m_length(0), 
    m_maxBufferSize(p_length),
    m_lastReadingID(0)
{
	Clear();
}

SharedBuffer::~SharedBuffer(void)
{
}

void 
SharedBuffer::Clear()
{
	for (Index i = 0; i < m_maxBufferSize; ++i)
	{
		m_readings[i].Clear();
	}
	m_curIndex = 0;
	m_length = 0;
	m_lastReadingID = 0;
}

SharedBuffer::Iterator 
SharedBuffer::Begin(Index p_length/*=0*/) const
{
    if( p_length == 0 )
    {
        p_length = m_maxBufferSize;
    }

	if( m_length >= p_length && p_length > 0 )
	{
		Index start = IsFull() ? 
			(m_curIndex + (m_maxBufferSize - p_length)) % m_maxBufferSize :
			(m_curIndex - p_length);
		return Iterator(*this, start);
	}
	return END;
}

void
SharedBuffer::Next(Iterator& p_iter) const
{
	if( p_iter.current != Iterator::BAD_VAL )
	{
		// when porting to power of 2 size buffer, this will just be (current+1)%MaxBufferSize
		if( p_iter.current != m_length-1 )
		{
			p_iter.current = p_iter.current+1;
		}
		else
		{
			p_iter.current = (m_length == m_maxBufferSize) ? 0 : p_iter.current+1;
		}
		if( p_iter.current == m_curIndex )
		{
			p_iter.current = Iterator::BAD_VAL;
		}
	}	
}

bool 
SharedBuffer::PushBack(const AccelerometerReading& p_reading)
{
	if( p_reading.GetReadingID() == 0 )
	{
		Clear();
	}
	else if( m_lastReadingID == p_reading.GetReadingID() )
	{
		return false;
	}
	m_readings[m_curIndex] = p_reading;
	m_lastReadingID = p_reading.GetReadingID();
	m_curIndex = (m_curIndex + 1) % m_maxBufferSize;
	if( m_length < m_maxBufferSize )
	{
		m_length++;
	}
    return m_lastReadingID == 0;
}
