#ifndef KPedometerStride_H
#define KPedometerStride_H
/*
============================================================================
Name        : KPedometerStats.h
Author      : KEYnetik, Inc.
Version     :
Copyright   : Copyright 2008 by KEYnetik, Inc.
Description : declarations for class PedometerStride
============================================================================
*/

#include "KTypes.h"
#include "KFixed.h"

namespace Keynetik
{
/**
*   Stride length calculation for pedometry.
*   Implements stride length calculations using a Base Stride Length (BSL)
*   BSL can be standard (a function of user's gender and height), or user-specified.
*   BSL can be dynamically recalculated based on real distance updates (e.g. from GPS),
*   The class estimates each step's length based on the above factors and current intensity (rate of steps).
*
*   NOTE: All distance values this class operates on are in meters.
*/
class PedometerStride
{
   public:
    typedef unsigned char StrideCm;
    static const StrideCm MaxStride = 255u;
    typedef unsigned char HeightCm;
    static const HeightCm MaxHeight = 255u;

    /**
    * Levels of step rate
    */
    typedef enum
    {
        VerySlow,
        Slow,
        Normal,
        Fast,
        VeryFast,
        MaxRate
    } StepRate;

    typedef struct
    {
        PedometerStride::StepRate rate;
        unsigned char threshold;      // step rat threshold, in seconds per step *200
        unsigned char rateFactor;     // 100ths of a step per second
        unsigned int metabolicFactor; // based on http://prevention.sph.sc.edu/tools/docs/documents_compendium.pdf
    } IntensityLevel;
    static inline Fixed ThresholdToFixed(unsigned char p_thr)
    {
        return Fixed(p_thr) / KEYNETIK_FIXED_CONST(200);
    }

    static const IntensityLevel *GetIntensityLevels()
    {
        return levels;
    }

   public:
    /**
    * Constructor. Sets base stride to 0.
    */
    PedometerStride();
    /**
    * Destructor.
    */
    ~PedometerStride();

    /**
    * Set fixed BSL
    */
    void SetFixedStride(StrideCm p_length);
    /**
    * Set fixed BSL based on the preson's gender and height
    */
    void SetUserStride(bool p_male, HeightCm p_height);

    // base stride limits (used to filter out distance updates)
    /**
    *   Set minimum aceptable value for BSL (default 0.2m)
    */
    void SetMinStride(StrideCm p_min)
    {
        m_minStride = p_min;
    }
    /**
    *   Set maximum aceptable value for BSL (default 2.0m)
    */
    void SetMaxStride(StrideCm p_max)
    {
        m_maxStride = p_max;
    }

    /**
    *   Restart calibration mode.
    */
    void Reset();

#ifndef KEYNETIK_COMPACT
    /**
    *   Initialize calibration mode
    */
    bool StartLearning();
    /**
    *   Query the calibration mode
    *   \return true if calibration is in progress
    */
    bool IsLearning() const
    {
        return m_learning;
    }
    /**
    *   Update distance travelled since the start of calibration, recalculate BSL
    *   \param p_distance distance travelled since the start of calibration
    *   \return true if successful, false if not calibrating or BSL is outside of acceptable range
    */
    bool UpdateDistance(Fixed p_distance);
    /**
    *   Stop calibration mode
    *   \return false if not in calibration mode
    */
    bool StopLearning();
#endif

    /**
    *   Register a step.
    *   \param p_intensity step rate (in 1000th of a step per sec)
    *   \param [out] p_length estimated stride length for this step, in cm
    *   \param [out] p_calories estimated calories burnt for this step, in 10000ths of cal per kg of weight
    */
    void ProcessStep(unsigned short p_intensity, HeightCm p_height, StrideCm &p_length, unsigned short &p_calories);
    static const short ScaleIntensityToStepSec = 1000;
    /**
    *  Get the current BSL
    *  \return current BSL
    */
    StrideCm GetBaseStride() const
    {
        return m_baseStride;
    }

   private:
    static const IntensityLevel levels[PedometerStride::MaxRate];
    static const IntensityLevel &GetIntensityLevel(unsigned short p_rate, HeightCm p_height);

    Fixed m_prevDistanceReported;
    Fixed m_distanceUsed;

#ifndef KEYNETIK_COMPACT
    typedef unsigned int StepStats[MaxRate];
    StepStats m_stepsSinceUpdate; // since last UpdateDistance
    StepStats m_steps;            // since Start
    bool m_learning;

    void ClearSteps(StepStats);
    bool CheckDistanceUpdate(Fixed p_periodDistance) const;
    static StrideCm CalculateBaseStride(const StepStats p_stats, Fixed p_distance);
    /**
    * Rate confidence level of a value falling into an interval.
    * The closer to the middle point of the interval, the higher the confidence.
    * \param p_lowThresh lower boundary of the interval
    * \param p_highThresh higher boundary of the interval
    * \param p_value value to be rated
    * \return confidence level, from Very Low to VeryHigh
    */
    static ConfidenceLevel Confidence(Fixed p_lowThresh, Fixed p_highThresh, Fixed p_value);
#endif

    StrideCm m_baseStride;
    StrideCm m_minStride;
    StrideCm m_maxStride;
};
}

#endif
