#ifndef KPedometerStatsCore_H
#define KPedometerStatsCore_H
/*
============================================================================
Name        : KPedometerStats.h
Author      : KEYnetik, Inc.
Version     :
Copyright   : Copyright 2010 by KEYnetik, Inc.
Description : declarations for class PedometerStatsCore
============================================================================
*/

#include "KPedometerCore.h"
#include "KPedometerStride.h"
#include "KGpsReading.h"
#include "KFixed.h"

namespace Keynetik
{
/**
* Extends the basic Pedometer algorithm with statistical capabilities.
* Adds the ability to start and stop statistics tracking.
* Also allows to fine tune the step count by ignoring possible noise
* that registered as a step.
* Maintains user data needed for pedometer stride and calorie burning calculations.
*/
class PedometerStatsCore
{
   public:
    /**
    * Representation of a PedometerStatsCore event
    */
    /**
    * Possible PedometerStatsCore enevts
    */
    typedef enum
    {
        None,
        Step,
        Calibration,
        TentativeStep
    } Event;

    typedef PedometerStride::StrideCm StrideCm;
    typedef PedometerStride::HeightCm HeightCm;
    typedef unsigned int DistanceCm;
    typedef unsigned char WeightKg;
    typedef unsigned char TimeSec;

    typedef unsigned int Calories; // in 10000ths of a calorie
    static const unsigned int CaloriesScale = 10000;

    static inline Fixed DistanceCmToM(DistanceCm p_dist)
    {
        if (p_dist > 10000)
            return Fixed(p_dist / 100);
        else
            return Fixed(p_dist) / KEYNETIK_FIXED_CONST(100);
    }

    /**
    * Step threshold setting of the underlying PedometerCore algorithm
    * \return Step threshold setting of the underlying PedometerCore algorithm
    */
    Fixed GetStepThreshold() const
    {
        return Fixed(m_pedometer.GetThreshold()) / Fixed(PedometerCore::ScaleToG);
    }
    void SetStepThreshold(Fixed p_val)
    {
        m_pedometer.SetThreshold((short)(p_val * Fixed(PedometerCore::ScaleToG)).ToInt());
    }

    // Settings
    void SetGender(bool p_male)
    {
        m_male = p_male;
    }
    void SetHeight(HeightCm p_height)
    {
        m_height = p_height;
    }
    void SetWeight(WeightKg p_weight) // kg
    {
        m_weight = p_weight;
    }
    void SetStride(StrideCm p_stride)
    {
        m_stride = p_stride;
    }
    void SetStepDelay(unsigned char p_stepDelay) // steps
    {
        m_StepDelay = p_stepDelay > (unsigned char)MaxBufferSize ? (unsigned char)MaxBufferSize : p_stepDelay;
    }
    void SetDelayTime(TimeSec p_delayTime) // seconds
    {
        m_DelayTime = p_delayTime;
    }
    void SetSpeedWindow(TimeSec p_window)
    {
        m_speedWindow = p_window;
    }

    bool GetGender() const
    {
        return m_male;
    }
    HeightCm GetHeight() const
    {
        return m_height;
    }
    WeightKg GetWeight() const
    {
        return m_weight;
    }
    StrideCm GetStride() const
    {
        return m_stride;
    }
    unsigned char GetStepDelay() const
    {
        return m_StepDelay;
    } // steps
    TimeSec GetDelayTime() const
    {
        return m_DelayTime;
    } // seconds
    TimeSec GetSpeedWindow()
    {
        return m_speedWindow;
    }

   protected:
    void SetMinStride(StrideCm p_minStride); // meters
    void SetMaxStride(StrideCm p_maxStride); // meters
    StrideCm GetMinStride() const
    {
        return m_MinStride;
    } // meters
    StrideCm GetMaxStride() const
    {
        return m_MaxStride;
    } // meters

   private:
    /**
    * Maintains all the settings specific to the PedometerStatsCore algorithm
    */
    WeightKg m_weight;
    HeightCm m_height;
    StrideCm m_stride;
    StrideCm m_MinStride;
    StrideCm m_MaxStride;
    bool m_male;
    unsigned char m_StepDelay;
    TimeSec m_DelayTime;
    TimeSec m_speedWindow;

   public:
    /**
    * Constructor. Resets the algorithm Settings Data to default values and
    * calls Reset() to initialize the algorithm.
    */
    PedometerStatsCore(ColumnLogger * = 0);
    /**
    * Destructor.
    */
    ~PedometerStatsCore();

    /**
    * Processes a single accelerometer reading .
    * \param p_reading incoming accelerometer reading
    * \return Step if a step has been detected, None otherwise
    */
    Event HandleIncomingEvent(const AccelerometerReading &p_reading);
    Event HandleIncomingEvent(short x, short y, short z); // in 1000ths of 1G

    /**
    * Processes a single Gps event.
    * \param p_evt incoming Gps event
    * \return Calibration if the incoming event is Update, None otherwise
    */
    Event HandleIncomingEvent(const GpsReading &p_evt);

    /**
    * Updates base stride length
    */
    void UpdateBaseStrideLength();

    /**
    * Deletes all buffered algorithm data and restores the algorithm to its default initial state.
    */
    void Reset();

    /**
    * Event created by last call to HandleIncomingEvent
    * \return Event created by last call to HandleIncomingEvent
    */
    Event GetLastEvent() const
    {
        return m_lastEvent;
    }

    /**
    * Get the total step count from algorithm creation or last call to Reset().
    * This total step count does not include steps that might have been filtered out
    * by the user specified StepDelay and DelayTime parameters.
    * For unfiltered step count, use GetPedometer::GetStepCount()
    * \return step count since the last Reset, with step filtering applied if in effect
    */
    unsigned int GetStepCount() const
    {
        return m_localStepCount;
    }

    /**
    * Get the total distance from algorithm creation or last call to Reset().
    * \return Distance in centimeters
    */
    DistanceCm GetDistance() const
    {
        return m_distanceCm;
    }
    Fixed GetDistanceM() const
    {
        return DistanceCmToM(m_distanceCm);
    }
    /**
    * Get the average speed from algorithm creation or last call to Reset().
    * \return Speed in kilometers per hour
    */
    Fixed GetSpeed() const;
    unsigned short GetSpeedMtpH() const; // meters per hour
                                         /**
                                         * Get the total calories burned from algorithm creation or last call to Reset(). In 1000ths of a calorie
                                         * \return  Number of calories (weightKg * distanceKm) * 1000
                                         */
    Calories GetCalories() const;

    /**
    * Get the total amount of time in seconds from algorithm creation or last call to Reset().
    * This time value is based solely on the number of reading that came in and the value set
    * as the polling frequence of the accelerometer. There are no internal timers and the system
    * clock is NOT accessed.
    * \return elapsed time, in seconds
    */
    Fixed GetElapsedTimeSec() const;

    /**
    * Calculate speed based on distance and time.
    * \param p_distanceCm distance in centimeters
    * \param p_timeSec time in seconds
    * \return speed, in meters per hour (distance / time)
    */
    static unsigned short SpeedMtpH(DistanceCm p_distance, unsigned int p_timeSec);

    /**
    * Returns const reference to internal pedometer object
    * \return internal PedometerCore algorithm
    */
    const PedometerCore &GetPedometer() const
    {
        return m_pedometer;
    }
    /**
    * Returns reference to internal pedometer object
    * \return internal PedometerCore algorithm
    */
    PedometerCore &GetPedometer()
    {
        return m_pedometer;
    }

    /**
    * Returns const reference to internal PedometerStride object
    * \return internal PedometerStride object
    */
    const PedometerStride &GetStrideCalculator() const
    {
        return m_strideCalculator;
    }
    /**
    * Returns reference to internal PedometerStride object
    * \return internal PedometerStride object
    */
    PedometerStride &GetStrideCalculator()
    {
        return m_strideCalculator;
    }

#ifndef KEYNETIK_COMPACT
    /**
    * Returns previous base stride length
    * \return previously calculates base stride length
    */
    StrideCm GetPrevBaseStride() const
    {
        return m_prevBaseStride;
    }
    /**
    * Returns previous distance
    * \return previously calculated distance
    */
    DistanceCm GetPrevDistance() const
    {
        return m_prevDistance;
    }
    Fixed GetPrevDistanceM() const
    {
        return DistanceCmToM(m_prevDistance);
    }
#endif

    /**
    * Returns base stride length
    * \return base stride length
    */
    StrideCm GetBaseStride() const
    {
        return m_baseStride;
    }

   private:
    Event HandleStep(); // None - tentative, do not report

    PedometerStride::StrideCm RegisterStep(unsigned short p_rate); // returns stride length

    PedometerCore m_pedometer;

    /**
    * Total distance
    */
    DistanceCm m_distanceCm;

    /**
    * Base Stride Length used in calculating the total distance
    */
    StrideCm m_baseStride;

    Calories m_calories;

#ifndef KEYNETIK_COMPACT
    StrideCm m_prevBaseStride;
    DistanceCm m_prevDistance;
#endif
    enum CountMode
    {
        Immediate,
        Tentative
    };

    CountMode m_CountMode;
    unsigned int m_localStepCount;

#ifdef KEYNETIK_PEDOMETER_MAX_STEP_DELAY
    static const unsigned char MaxBufferSize = KEYNETIK_PEDOMETER_MAX_STEP_DELAY;
#else
    static const unsigned char MaxBufferSize = DefaultMaxBufferSize;
#endif
    typedef TCircularBuffer<MaxBufferSize, unsigned short> PrevStepsBuffer;
    PrevStepsBuffer m_previousSteps;

    Event m_lastEvent;

    PedometerStride m_strideCalculator;

   private:
    PedometerStatsCore(const PedometerStatsCore &);
    PedometerStatsCore operator=(const PedometerStatsCore &);

#ifndef KEYNETIK_EMBEDDED
    ColumnLogger *m_log;
#endif
};
}

#endif
