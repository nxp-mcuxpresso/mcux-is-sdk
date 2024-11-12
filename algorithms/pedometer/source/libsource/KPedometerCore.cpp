/*
============================================================================
 Name        : KPedometerCore.cpp
 Author      : KEYnetik, Inc.		    		   	 
 Version     :
 Copyright   : Copyright 2010 by KEYnetik, Inc.
 Description : definition of class PedometerCore
============================================================================
*/

#include "KPedometerCore.h"                   

#include "KLoggingUtility.h"

using namespace Keynetik;

#define THRESHOLD_DEFAULT 130
const short PedometerCore::StepThresholdDefault=THRESHOLD_DEFAULT;

CircularBuffer::Index 
PedometerCore::BufferSize(unsigned char p_timeWindow)
{
    CircularBuffer::Index ret=MathUtilities::BufferSize(p_timeWindow);
    return ret < MaxBufferSize ? ret : MaxBufferSize;
}

enum
{
    X,
    Y,
    Z,
    A,
    AvgA,
    SpreadA,
    Intensity,
    Signal,
    AboveThreshMax
};

PedometerCore::PedometerCore(ColumnLogger* p_log) :
    m_StepThreshold(THRESHOLD_DEFAULT),
    m_avgA(0),
    m_sprA(0)
#ifndef KEYNETIK_EMBEDDED
    ,
    m_log(p_log)
#endif
{
	Reset();

    LOG_ADD_OWNER("Pedometer");
    LOG_ADD_COLUMN(X);
    LOG_ADD_COLUMN(Y);
    LOG_ADD_COLUMN(Z);
    LOG_ADD_COLUMN(A);
    LOG_ADD_COLUMN(AvgA);
    LOG_ADD_COLUMN(SpreadA);
    LOG_ADD_COLUMN(Signal);
    LOG_ADD_COLUMN(AboveThreshMax);
}

PedometerCore::~PedometerCore()
{
}

void
PedometerCore::SaveAvgA(short p_avgA)
{
    m_avgA.Append(p_avgA);
    if (m_avgA.IsFull())
    {
        short avgA=m_avgA.Average();
        LOG(AvgA, avgA);
        m_sprA.Append(avgA);
    }
}

PedometerCore::Event 
PedometerCore::HandleEvent()
{
    m_lastEvent=None;

    ++m_readings;

    if (m_sprA.IsFull())
    {
        short aSpread=m_sprA.MaxSpread();
        LOG(SpreadA, aSpread);
        AnalyzeStep(aSpread);
    }

    LOG_ENDLINE();
    return m_lastEvent;
}

PedometerCore::Event 
PedometerCore::HandleIncomingEvent(short x, short y, short z)
{
    int newA=(int)x*x+(int)y*y+(int)z*z;
	LOG(X, Fixed(x)/Fixed(ScaleToG));
	LOG(Y, Fixed(y)/Fixed(ScaleToG));
	LOG(Z, Fixed(z)/Fixed(ScaleToG));
	LOG(A, newA);
    SaveAvgA((short)MathUtilities::Sqrt((unsigned int)newA));
    return HandleEvent();
}


void
PedometerCore::AnalyzeStep(short p_aSpread)
{
    if (m_readingsAboveZero == 0)
    {
        /*
        we are only interested in positive spreads for this comparison because a negative
        spread indicates a return to low G which could be a jump or something else not
        resembling a step
        */
        if (p_aSpread > m_StepThreshold)
        {
            m_readingsAboveZero=1;
            m_lastMaxSpread = p_aSpread - m_StepThreshold;
        }
    }
    else
    {
        if (p_aSpread <= 0)
        {
            HandleStep();
        }
        else
        {
            if (m_lastMaxSpread < p_aSpread - m_StepThreshold)
            {
                m_lastMaxSpread = p_aSpread - m_StepThreshold;
            }
            ++m_readingsAboveZero;
        }
    }
}
void
PedometerCore::HandleStep()
{
    unsigned char NoiseFilterWindow=7; // 0.07 sec
    if (m_readingsAboveZero > BufferSize(NoiseFilterWindow))
    {
        unsigned short stepRate= (unsigned short)((AccelerometerReading::GetFrequency() * ScaleRateToStepPerSec) / (m_readings - m_lastStep));
        LOG(Intensity, Fixed(stepRate)/Fixed(ScaleRateToStepPerSec));

        m_lastEvent = Step;
        m_stepRate = stepRate;

        LOG(Signal, "Step");
        LOG(AboveThreshMax, m_lastMaxSpread);

        ++m_stepCount;
        m_lastStep = m_readings;
		if( m_stepCount == 1 )
		{
            m_firstStep = m_lastStep - m_avgA.GetSize() - m_readingsAboveZero;
		}
    }
    m_readingsAboveZero = 0;
}

void 
PedometerCore::Reset()
{
    unsigned char TimeWindow=19; // 0.19 sec
    CircularBuffer::Index bSize=BufferSize(TimeWindow);
    m_avgA.Resize(bSize);
    m_sprA.Resize(bSize);
    m_readingsAboveZero=0;
    m_stepCount=0;
    m_readings=0;
    m_lastStep = 0;
    m_firstStep = 0;
    m_lastMaxSpread = 0;

    m_lastEvent=None;
    m_stepRate = 0;
}													 	  


