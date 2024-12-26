/*
============================================================================
Name        : KPedometerStride.cpp
Author      : KEYnetik, Inc.
Version     :
Copyright   : Copyright 2009 by KEYnetik, Inc.
Description : definition for class PedometerStride
============================================================================
*/

#include "KPedometerStride.h"

using namespace Keynetik;

#define DefaultMinStride 20
#define DefaultMaxStride 200

PedometerStride::PedometerStride() :
    m_baseStride(0),
    m_minStride(DefaultMinStride),
    m_maxStride(DefaultMaxStride)
{
    Reset();
}    

PedometerStride::~PedometerStride()
{
}

void
PedometerStride::Reset()
{
    m_distanceUsed=0;
#ifndef KEYNETIK_COMPACT
    m_learning=false;
    ClearSteps(m_steps);
    ClearSteps(m_stepsSinceUpdate);
#endif
    m_prevDistanceReported=0;
}

void 
PedometerStride::SetFixedStride(StrideCm p_length)
{
    m_baseStride=p_length;
}

void 
PedometerStride::SetUserStride(bool p_male, StrideCm p_heightCm)
{
    #define DefaultMaleStride    415
    #define DefaultFemaleStride  413
    #define StrideMultipler      11  // correction stride length value based on the algorithm's performance analysis

    unsigned int sexCoef = p_male ? DefaultMaleStride : DefaultFemaleStride;
    unsigned int strideCm=((unsigned int)p_heightCm * sexCoef * StrideMultipler)/10000;
    if (strideCm > MaxStride)
    {
        m_baseStride = MaxStride;
    }
    else
    {
        m_baseStride = (StrideCm)strideCm; 
    }
}

// Step rate thresholds, in seconds per step * 200
#define VerySlowRate        224
#define SlowRate            190
#define FastRate            130
#define VeryFastRate        125

// Distance is a function of stride length; the stride length formula features a rate factor calculated based 
// on the rate thresholds above. 
// RateFactor = XXXStrideFactor / RateFactorScale
#define RateFactorScale      100 
#define VerySlowStrideFactor  88 
#define SlowStrideFactor      (95-VerySlowStrideFactor) 
#define NormalStrideFactor    (100-VerySlowStrideFactor) 
#define FastStrideFactor      (130-VerySlowStrideFactor) 
#define VeryFastStrideFactor  (230-VerySlowStrideFactor) 

#define METscalingFactor 2900

const PedometerStride::IntensityLevel PedometerStride::levels[PedometerStride::MaxRate]=
{
    {PedometerStride::VerySlow,  VerySlowRate,      VerySlowStrideFactor,  20 * METscalingFactor},   // 1.5 mph
    {PedometerStride::Slow,      SlowRate,          SlowStrideFactor,      25 * METscalingFactor}, // 2.5 mph
    {PedometerStride::Normal,    FastRate,          NormalStrideFactor,    38 * METscalingFactor}, // 3.5 mph 
    {PedometerStride::Fast,      VeryFastRate,      FastStrideFactor,      80 * METscalingFactor},   // 5 mph
    {PedometerStride::VeryFast,  0,                 VeryFastStrideFactor, 125 * METscalingFactor},// 7.5 mph
};

const PedometerStride::IntensityLevel& 
PedometerStride::GetIntensityLevel(unsigned short p_rate, HeightCm p_height)
{
    const IntensityLevel* level=levels;
    if( p_height == 0 )
    {
        level += 2;
    }
    else
    {
		const int AverageHeightCm = 180;
		const int K = 90;
        while (level->threshold != 0 &&  
               p_rate > (int)(AverageHeightCm-p_height) * ScaleIntensityToStepSec / K + 
                            (AverageHeightCm * ScaleIntensityToStepSec * 2) / level->threshold)
        {
           ++level;
        }
    }
    return *level;
}

void
PedometerStride::ProcessStep(unsigned short p_intensity, HeightCm p_height, StrideCm& p_length, unsigned short& p_calories) 
{
    const IntensityLevel& level=GetIntensityLevel(p_intensity, p_height);

#ifndef KEYNETIK_COMPACT

    if (m_learning)
    {
        ++(m_stepsSinceUpdate[level.rate]);
    }
#endif

    unsigned int rateFactor=level.rateFactor;
    if (level.rate != PedometerStride::VerySlow)
    {
        rateFactor += VerySlowStrideFactor;
    }
    p_length= (StrideCm)((unsigned int)m_baseStride * rateFactor / RateFactorScale);
    // for the puropses of calorie counting, assume the intensity cannot be lower than 1.0 (step/sec),
    // otherwise the first step after a break would result in a high calorie increment
    const unsigned short minRate(100); // in 100ths of step/sec
    if (p_intensity < minRate)
    {
        p_intensity=minRate;
    }
    const unsigned int ScalingFactor=10;
    p_calories= (unsigned short)(level.metabolicFactor / p_intensity / ScalingFactor);
}

#ifndef KEYNETIK_COMPACT
void 
PedometerStride::ClearSteps(StepStats p_stats)
{
    for (int i=(int)VerySlow; i < (int)MaxRate; ++i)
    {
        p_stats[i]=0;
    }
}

PedometerStride::StrideCm
PedometerStride::CalculateBaseStride(const StepStats p_stats, Fixed p_distance) 
{
    Fixed total; // total distance divided by current baseStride
    for (int i=(int)VerySlow; i < (int)MaxRate; ++i)
    {
        unsigned int rateFactor=levels[i].rateFactor;
        if (i != VerySlow)
        {
            rateFactor += VerySlowStrideFactor;
        }
        total+= Fixed(rateFactor) * Fixed(p_stats[i]) / Fixed(RateFactorScale);
    }
    // newStride / oldStride = p_distance / totalDistance 
    //      where totalDistance == total * oldStride, which cancels out oldStride 
    //  on both sides of the equation, thus:
    int ret=(p_distance / total * KEYNETIK_FIXED_CONST(100)).ToInt();
    return ret > 255 ? MaxStride : StrideCm(ret);
}

bool 
PedometerStride::CheckDistanceUpdate(Fixed p_periodDistance) const
{
    StrideCm periodStride=CalculateBaseStride(m_stepsSinceUpdate, p_periodDistance);
    return periodStride >= m_minStride && periodStride <= m_maxStride;
}

bool 
PedometerStride::StartLearning()
{
    if (m_learning)
    {
        return false;
    }
    ClearSteps(m_steps);
    ClearSteps(m_stepsSinceUpdate);
    m_learning=true;
    return true;
}

bool 
PedometerStride::UpdateDistance(Fixed p_distanceReported)
{
    if (!m_learning)
    {
        return false;
    }
    // filter out bad updates
    Fixed periodDistance= p_distanceReported - m_prevDistanceReported;
    if (CheckDistanceUpdate(periodDistance))
    {   // learning period accepted; 
        m_distanceUsed += periodDistance;
        m_prevDistanceReported = p_distanceReported;
        // apply steps collected over the last period
        for (int i=(int)VerySlow; i < (int)MaxRate; ++i)
        {
            m_steps[i] += m_stepsSinceUpdate[i];
        }
        // update base stride for the entire distance
        m_baseStride=CalculateBaseStride(m_steps, m_distanceUsed); 
        ClearSteps(m_stepsSinceUpdate);
        return true;
    }
    ClearSteps(m_stepsSinceUpdate);
    return false;
}

bool 
PedometerStride::StopLearning()
{
    if (!m_learning)
    {
        return false;
    }
    m_learning=false;
    return true;
}

ConfidenceLevel
PedometerStride::Confidence(Fixed p_lowThresh, Fixed p_highThresh, Fixed p_value)
{
    #define OneTenth    KEYNETIK_FIXED_CONST(0.1)
    #define TwoTenths   KEYNETIK_FIXED_CONST(0.2)
    #define ThreeTenths KEYNETIK_FIXED_CONST(0.3)
    #define FourTenths  KEYNETIK_FIXED_CONST(0.4)

    Fixed toLow=(p_lowThresh - p_value).Abs();
    Fixed toHigh=(p_highThresh-p_value).Abs();

    Fixed percent = (toLow < toHigh ? toLow : toHigh) / (p_highThresh-p_lowThresh).Abs();

    if( percent < OneTenth )
    {
        return ConfidenceVeryLow;
    }
    if( percent < TwoTenths )
    {
        return ConfidenceLow;
    }
    if( percent < ThreeTenths )
    {
        return ConfidenceMedium;
    }
    if( percent < FourTenths )
    {
        return ConfidenceHigh;
    }
    else
    {
        return ConfidenceVeryHigh;
    }   
}

#endif
