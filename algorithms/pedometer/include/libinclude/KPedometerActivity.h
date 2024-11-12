#ifndef KPedometerActivityCore_H
#define KPedometerActivityCore_H
/*
============================================================================
 Name        : KPedometerActivity.h
 Author      : KEYnetik, Inc.
 Version     :
 Copyright   : Copyright 2010 by KEYnetik, Inc.
 Description : declaration of class PedometerActivity
============================================================================
*/
#include "KCircularBuffer.h"

namespace Keynetik
{
/**
* Implements PedometerActivity utility which determines activity level and instantaneous speed
*/
class PedometerActivity
{
   public:
    typedef enum
    {
        Rest = 0,
        Walking,
        Jogging,
        Running,
        NoValue
    } ActivityLevel;

    typedef unsigned char StepDistanceCm;

   public:
    /**
    * Constructor
    */
    PedometerActivity();
    /**
    * Destructor
    */
    ~PedometerActivity();

    /**
    * Process an accelerometer reading for which a step was not detected. Updates instantaneous speed.
    * \param p_readingNumber reading number
    * \returns current activity level
    */
    ActivityLevel HandleReading(unsigned int p_readingNumber);
    /**
    * Process an accelerometer reading for which a step was detected. Updates instantaneous speed.
    * \param p_readingNumber reading number
    * \param p_distance (in meters) estimated distance covered by the step
    * \returns current activity level
    */
    ActivityLevel HandleStep(unsigned int p_readingNumber, StepDistanceCm p_distance);

    /**
    * Returns the current value of instantaneous speed.
    * \return speed in meters/hr measured over the last m_InstaSpeedPeriod seconds. 0 if not enough data.
    */
    unsigned short GetSpeed() const
    {
        return m_speed;
    }

    /**
    * Returns the latest distance covered over the speed measurement period
    * \return distance in centimeters measured over the last m_InstaSpeedPeriod seconds. 0 if not enough data.
    */
    unsigned short GetDistanceCm() const
    {
        return m_distanceCm;
    }

    /**
    * Deletes all buffered data and restores the utility to its default initial state.
    */
    void Reset();

    // Settings
    // in meters/hour
    unsigned short m_WalkingThreshold;
    unsigned short m_JoggingThreshold;
    unsigned short m_RunningThreshold;
    unsigned short m_SecondsBeforeRest;

    unsigned char m_InstaSpeedPeriod; // in seconds

   private:
    ActivityLevel CalcActivityLevel() const;
    unsigned short InstaSpeed(unsigned int p_readingNumber);

   private:
    static const unsigned char StepBufferSizeSec = 5; // instaspeed measured on up to 5 seconds
    static const unsigned char MaxStepsPerSecond = 3; // expecting up to 3 steps a second
    // step coordinates, in increments
    typedef TCircularBuffer<StepBufferSizeSec * MaxStepsPerSecond, unsigned char> StepBuffer;
    // step distances
    typedef TCircularBuffer<StepBufferSizeSec * MaxStepsPerSecond, StepDistanceCm> StepDistanceBuffer;

   private:
    PedometerActivity(const PedometerActivity &);
    PedometerActivity operator=(const PedometerActivity &);

    StepBuffer m_steps;
    StepDistanceBuffer m_stepDistances;
    unsigned int m_lastStepReading;
    unsigned short m_speed;
    unsigned short m_distanceCm;

    unsigned short m_restThr;
};
}

#endif
