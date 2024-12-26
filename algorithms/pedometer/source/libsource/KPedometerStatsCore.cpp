/*
============================================================================
Name        : KPedometerStatsCore.cpp
Author      : KEYnetik, Inc.
Version     :
Copyright   : Copyright 2010 by KEYnetik, Inc.
Description : definition for class PedometerStatsCore
============================================================================
*/

#include "KPedometerStatsCore.h"

#include "KLoggingUtility.h"
#include "KPedometerStride.h"

using namespace Keynetik;

#define DefaultMinStride    20
#define DefaultMaxStride    200
#define SpeedWindowDefault  5

enum
{
    StepEvent,
    TimeStamp,
    StepRate,
    BaseStride,
    StepConfidence,
    Filtering,
    Stride,
    StideConfidence,
    TotalDistance,
    TotalCalories,
    TotalStepCount,
    TentativeAccepted,
    GpsDistance,
    NewDistance,
    NewStride
};

PedometerStatsCore::PedometerStatsCore(ColumnLogger* p_log) 
:   m_male(true),
    m_height(0), 
    m_weight(0), 
    m_stride(0), 
    m_StepDelay(0),
    m_DelayTime(0), 
    m_speedWindow(SpeedWindowDefault),
    m_pedometer(p_log), 
    m_previousSteps(0)
#ifndef KEYNETIK_COMPACT
    ,
    m_MinStride(DefaultMinStride),
    m_MaxStride(DefaultMaxStride),
    m_prevDistance(0),
    m_prevBaseStride(0)
#endif
#ifndef KEYNETIK_EMBEDDED
    ,
    m_log(p_log)
#endif
{
    Reset(); 

    LOG_ADD_OWNER("PedStats");
    LOG_ADD_COLUMN(StepEvent);
    LOG_ADD_COLUMN(TimeStamp);
    LOG_ADD_COLUMN(StepRate);
    LOG_ADD_COLUMN(BaseStride);
    LOG_ADD_COLUMN(StepConfidence);
    LOG_ADD_COLUMN(Filtering);
    LOG_ADD_COLUMN(Stride);
    LOG_ADD_COLUMN(StideConfidence);
    LOG_ADD_COLUMN(TotalDistance);
    LOG_ADD_COLUMN(TotalCalories);
    LOG_ADD_COLUMN(TotalStepCount);
    LOG_ADD_COLUMN(TentativeAccepted);
    LOG_ADD_COLUMN(GpsDistance);
    LOG_ADD_COLUMN(NewDistance);
    LOG_ADD_COLUMN(NewStride);
}    

PedometerStatsCore::~PedometerStatsCore()
{
}

PedometerStatsCore::Event
PedometerStatsCore::HandleIncomingEvent(const AccelerometerReading& p_reading)
{
    return HandleIncomingEvent((short)(p_reading.X() * Fixed(PedometerCore::ScaleToG)).ToInt(),
                               (short)(p_reading.Y() * Fixed(PedometerCore::ScaleToG)).ToInt(),
                               (short)(p_reading.Z() * Fixed(PedometerCore::ScaleToG)).ToInt());
}

PedometerStatsCore::Event
PedometerStatsCore::HandleIncomingEvent(short x, short y, short z)
{
    if (m_pedometer.HandleIncomingEvent(x, y, z) == PedometerCore::Step)
    {
        Event ret=HandleStep();
        LOG_ENDLINE();
        return ret;
    }
    LOG_ENDLINE();
    return None;
}

#define LOG_STEP(p_lastStride)\
{\
    LOG(TentativeAccepted, "accepted");\
    LOG(Stride, int(p_lastStride));\
    LOG(TotalDistance, double(GetDistance())/100.0);\
    LOG(TotalCalories, Fixed(GetCalories()));\
    LOG(TotalStepCount, GetStepCount());\
}

PedometerStatsCore::Event
PedometerStatsCore::HandleStep()
{
    unsigned short coreRate=m_pedometer.GetStepRate();
    LOG(StepEvent, "Step");
    LOG(TimeStamp, m_pedometer.GetReadingCount());
    // for a first step after a stand-still interval, report a minimum rate of 1.0 step/sec
    LOG(StepRate, Fixed(coreRate < PedometerCore::ScaleRateToStepPerSec ? PedometerCore::ScaleRateToStepPerSec : coreRate) / 
                                                                                    Fixed(PedometerCore::ScaleRateToStepPerSec));
    Event ret=None;
    if (m_StepDelay == 0 ||
        (m_CountMode == Immediate && m_StepDelay <= (unsigned char)(m_DelayTime * coreRate / PedometerCore::ScaleRateToStepPerSec)))
    {
        m_baseStride=m_strideCalculator.GetBaseStride();
        PedometerStride::StrideCm step=RegisterStep(coreRate);
        LOG_STEP(step);
        ret=Step;
    }
    else
    {
        m_CountMode = Tentative;        
        m_previousSteps.Append(coreRate);
        if( m_previousSteps.IsFull() )
        {
            // calculate time elapsed between the earliest and the latest steps
            int time(0); // in 1000ths of a second
            const int ScaleToSec=1000;

            // skip the oldest recorded step in the buffer because its value 
            // is not relevant to our calculation
            KEYNETIK_CONST_ITER(PrevStepsBuffer, m_previousSteps, it, 1);
            while (it.IsGood())
            {
                unsigned short v = it.GetValue();
                if (v == 0) // too much time between the steps
                {
                    LOG(Filtering, "tentative");
                    return TentativeStep;
                }
                time += PedometerCore::ScaleRateToStepPerSec * ScaleToSec / v;
                it.Next();
            }

            if (time <= m_DelayTime*ScaleToSec)
            {
                PedometerStride::StrideCm lastStride;
                KEYNETIK_CONST_ITER_OLDEST(PrevStepsBuffer, m_previousSteps, it);
                // report all steps in buffer and update distance calculation
                while (it.IsGood())
                {
                    lastStride=RegisterStep(it.GetValue());
                    it.Next();
                }
                LOG(Filtering, m_previousSteps.GetOccupied());
                m_previousSteps.Clear();
                m_CountMode = Immediate;

                m_baseStride=m_strideCalculator.GetBaseStride();
                LOG_STEP(lastStride);
                ret=Step;
            }
            else
            {
                ret=TentativeStep;
                LOG(Filtering, "tentative");
            }
        }
        else
        {
            ret=TentativeStep;
            LOG(Filtering, "tentative");
        }
    }
    return ret;
}

PedometerStride::StrideCm
PedometerStatsCore::RegisterStep(unsigned short p_rate)
{
    ++m_localStepCount;
    PedometerStride::StrideCm ret;
    unsigned short calories;
    m_strideCalculator.ProcessStep(p_rate, m_height, ret, calories);
   
    m_distanceCm += ret; 
    m_calories += calories;
    return ret;
}

void 
PedometerStatsCore::Reset()
{
    m_pedometer.Reset();
    m_baseStride=0;

    m_localStepCount = 0;
    m_previousSteps.Resize(CircularBuffer::Index(m_StepDelay)); 
    m_CountMode = Tentative;

    m_distanceCm=0;
    m_calories=0;

    m_strideCalculator.Reset();

#ifndef KEYNETIK_COMPACT
    m_prevDistance=0;
    m_prevBaseStride=0;
    m_strideCalculator.SetMinStride(m_MinStride);
    m_strideCalculator.SetMaxStride(m_MaxStride);
#endif
    if (m_stride != 0)
    {
        m_strideCalculator.SetFixedStride(m_stride);
    }
    else
    {
        m_strideCalculator.SetUserStride(m_male, m_height);
    }   
}

unsigned short 
PedometerStatsCore::SpeedMtpH(DistanceCm p_distance, unsigned int p_readings)
{
    unsigned int seconds1000ths = p_readings * 1000 / AccelerometerReading::GetFrequency();
    if (seconds1000ths == 0)
    {
        return 0;
    }
    const unsigned int SecondsInHour = 60*60;
    const unsigned int CmInM = 100;
    if (p_distance > 10000)
    {   // divide first to avoid overflow
        p_distance /= CmInM; 
        p_distance *= 1000;
        p_distance /= seconds1000ths;
        return (unsigned short)(p_distance * SecondsInHour);
    }
    else    
    {   // multiply first
        p_distance *= 1000;
        p_distance /= CmInM; 
        p_distance *= SecondsInHour;
        return (unsigned short)(p_distance / seconds1000ths);
    }
#undef SecondsMetersFactor
}

Fixed
PedometerStatsCore::GetSpeed() const
{   
    return Fixed(GetSpeedMtpH()) / KEYNETIK_FIXED_CONST(1000);
}
unsigned short
PedometerStatsCore::GetSpeedMtpH() const
{   
    return SpeedMtpH(GetDistance(), m_pedometer.GetReadingCount());
}

PedometerStatsCore::Calories
PedometerStatsCore::GetCalories() const 
{   
    return m_calories * m_weight;
}

Fixed
PedometerStatsCore::GetElapsedTimeSec() const
{
    if (AccelerometerReading::GetFrequency() == 0)
    {
        return 0;
    }
    return Fixed(m_pedometer.GetReadingCount()) / Fixed(AccelerometerReading::GetFrequency());
}

#ifndef KEYNETIK_COMPACT
PedometerStatsCore::Event
PedometerStatsCore::HandleIncomingEvent(const GpsReading& p_evt)
{
    m_lastEvent = None;

    switch (p_evt.eventID)
    {
    case GpsReading::Start:
        GetStrideCalculator().StartLearning();
        break;
    case GpsReading::Stop:
        GetStrideCalculator().StopLearning();
        break;
    case GpsReading::Update:
        if (GetStrideCalculator().IsLearning())
        {
            LOG(GpsDistance, p_evt.distanceM);

            // save off pre-calibration values
            DistanceCm oldDistance = GetDistance();
            StrideCm oldBaseStride = GetStrideCalculator().GetBaseStride();

            // now, recalibrate the base stride
            if (GetStrideCalculator().UpdateDistance(p_evt.distanceM)) // update accepted
            {
                // newDistance / oldDistance == newBaseStride / oldBaseStride, thus:
                UpdateBaseStrideLength();

                // override some values
                m_lastEvent = Calibration;
                m_prevBaseStride = oldBaseStride;
                m_prevDistance = oldDistance;                

                LOG(NewDistance, double(m_distanceCm)/100.0);
                LOG(NewStride, m_baseStride);
            }
            else
            {
                LOG(NewDistance, GetStrideCalculator().IsLearning() ? "rejected" : "not learning");
            }
        }
        break;
    }
    LOG_ENDLINE();
    return m_lastEvent;
}

void 
PedometerStatsCore::SetMinStride(StrideCm p_minStride)
{
    m_MinStride=p_minStride;
}

void 
PedometerStatsCore::SetMaxStride(StrideCm p_maxStride)
{
    m_MaxStride=p_maxStride;
}

void 
PedometerStatsCore::UpdateBaseStrideLength()
{
    StrideCm newBaseStrideLength=m_strideCalculator.GetBaseStride();
    m_distanceCm = (m_distanceCm * newBaseStrideLength) / m_baseStride;
    m_baseStride = newBaseStrideLength;
}
#endif
