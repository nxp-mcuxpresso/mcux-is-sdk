/*
============================================================================
 Name        : KPedometerActivity.cpp
 Author      : KEYnetik, Inc.
 Version     :
 Copyright   : Copyright 2010-2012 by KEYnetik, Inc.
 Description : definition of class PedometerActivity
============================================================================
*/

#include "KPedometerActivity.h"
#include "KAccelerometerReading.h"
#include "KCircularBufferConstIterator.h"

using namespace Keynetik;

#define WalkingThresholdDefault  1000
#define JoggingThresholdDefault  6500
#define RunningThresholdDefault 10500

// if more than SecondsBeforeRestDefault passes between steps, we assume the user has stopped
#define SecondsBeforeRestDefault1000th 2500
#define InstaSpeedPeriodDefault 5u

#define ActivityStepsWindow 5

PedometerActivity::PedometerActivity() :
    m_WalkingThreshold(WalkingThresholdDefault),
    m_JoggingThreshold(JoggingThresholdDefault),
    m_RunningThreshold(RunningThresholdDefault),
    m_SecondsBeforeRest(SecondsBeforeRestDefault1000th),
    m_InstaSpeedPeriod(InstaSpeedPeriodDefault)
{
	Reset();
}

PedometerActivity::~PedometerActivity()
{
}

PedometerActivity::ActivityLevel
PedometerActivity::CalcActivityLevel() const
{
    if (m_speed > m_RunningThreshold)
    {
        return Running;
    }
    if (m_speed > m_JoggingThreshold)
    {
        return Jogging;
    }
    if (m_speed > m_WalkingThreshold)
    {
        return Walking;
    }
    return Rest;
}

PedometerActivity::ActivityLevel
PedometerActivity::HandleReading(unsigned int p_readingNumber)
{
    ActivityLevel ret;
    if (m_steps.IsEmpty() || p_readingNumber - m_lastStepReading > m_restThr)
    {
        ret = Rest;
    }
    else if ( (p_readingNumber - m_lastStepReading) % AccelerometerReading::GetFrequency() != 0) // only update once a second when no steps are detected
    {
        ret = NoValue;
    }
    else
    {
        m_speed=InstaSpeed(p_readingNumber);
        ret = CalcActivityLevel();
    }

    if (ret == Rest)
    {
        m_speed = 0;
    }

    return ret;
}

PedometerActivity::ActivityLevel
PedometerActivity::HandleStep(unsigned int p_readingNumber, StepDistanceCm p_distance)
{
    unsigned int delta=p_readingNumber - m_lastStepReading;
    m_steps.Append(delta > 255 ? 255u : (StepBuffer::ElementType)delta);
    m_lastStepReading=p_readingNumber;
    m_stepDistances.Append(p_distance);
    m_speed=InstaSpeed(p_readingNumber);
    return CalcActivityLevel();
}

unsigned short
PedometerActivity::InstaSpeed(unsigned int p_readingNumber)
{
    unsigned int speedWindow=(unsigned int)AccelerometerReading::GetFrequency() * m_InstaSpeedPeriod;

    if (speedWindow == 0 || speedWindow > p_readingNumber)
    {
        return 0;
    }

    // find all steps in the period P, sum distances
    unsigned int readingsPassed=p_readingNumber - m_lastStepReading;
    m_distanceCm=0;
    TCircularBufferConstIterator<StepBuffer::MaxBufferSize, StepBuffer::ElementType> lastStep(m_steps, -1);
    if (!lastStep.IsGood())
    {   // no steps
        return 0;
    }

    while (lastStep.IsGood())
    {
        CircularBuffer::Index step=lastStep.GetIndex();

        // contribute the step's distance
        if ( readingsPassed + m_steps.GetData(step) > speedWindow )
        {   // only partial distance from the step
            // contribute estimated distance between the oldest step in-period and the start of the period
            m_distanceCm += m_stepDistances.GetData(step) * (speedWindow-readingsPassed) / m_steps.GetData(step);
            break;
        }
        readingsPassed += m_steps.GetData(step);
        m_distanceCm   += m_stepDistances.GetData(step);
        lastStep.Prev();
    }
    // speed = meters per hour; convert from cm/sec into m/hour
    return (unsigned short)(m_distanceCm * AccelerometerReading::GetFrequency() * 3600 / 100 / speedWindow);
}

void
PedometerActivity::Reset()
{
    m_steps.Clear();
    m_stepDistances.Clear();
    m_lastStepReading=0;

    unsigned int freq(AccelerometerReading::GetFrequency());
    m_restThr   =(unsigned short)(freq * m_SecondsBeforeRest / 1000);
}
