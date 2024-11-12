#ifndef KPedometerCore_H
#define KPedometerCore_H
/*
============================================================================
 Name        : KPedometerCore.h
 Author      : KEYnetik, Inc.
 Version     :
 Copyright   : Copyright 2010 by KEYnetik, Inc.
 Description : declaration of class PedometerCore
============================================================================
*/

#include "KCircularBuffer.h"
#include "KAccelerometerReading.h"

namespace Keynetik
{
class ColumnLogger;
/**
* Implements core logic for the Pedometer algorithm.
*/
class PedometerCore
{
   public:
    /**
          * Results
          */
    typedef enum
    {
        None,
        Step
    } Event;

    /**
    * Setting Name: StepThreshold \n
    * Default Value: 130 \n
    * Valid Range: 10 to 500 \n
    * Description: In 1000ths of 1G. The minimum spread of average A that must be sustained
    * for 0.07s in order for a step to be reported.
    */
    short GetThreshold() const
    {
        return m_StepThreshold;
    }
    void SetThreshold(short p_value)
    {
        m_StepThreshold = p_value;
    }

    static const short ScaleToG = 1000;

   public:
    /**
    * Constructor. Resets the algorithm to default values and calls Reset() to initialize the algorithm.
    * \param p_log  pointer to optional logging object
    */
    PedometerCore(ColumnLogger *p_log = 0);
    /**
    * Destructor.
    */
    ~PedometerCore();

    /**
    * Processes a single accelerometer reading. A non-None event will be returned at the end of each dyadic block.
    * \return An event that might have occured due to the processed reading. None if no new event occured.
    */
    Event HandleIncomingEvent(short x, short y, short z); // in 1000ths of 1G

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

    // based on last step
    /**
    * Get the total step count from algorithm creation or last call to Reset()
    * \return total step count from algorithm creation or last call to Reset()
    */
    unsigned int GetStepCount() const
    {
        return m_stepCount;
    }
    /**
    * Returns step rate between last two steps, in 1000ths of a step per second
    * \return step rate between last two steps
    */
    unsigned short GetStepRate() const
    {
        return m_stepRate;
    }
    static const short ScaleRateToStepPerSec = 1000;

    /**
    * Returns amplitude of previous step, in 1000ths of G. Only valid until the next call to HandleIncomingEvent()
    * \return amplitude of previous step
    */
    short GetAmplitude() const
    {
        short ret = m_avgA.MaxSpread();
        return ret < 0 ? -ret : ret;
    }

    // based on last reading
    /**
    * Returns the number of AccelerometerReading events processed since construction or last Reset()
    * \return the number of AccelerometerReading events processed since construction or last Reset()
    */
    unsigned int GetReadingCount() const
    {
        return m_readings;
    }

   private:
    static const short StepThresholdDefault;

#ifdef KEYNETIK_PEDOMETER_MAX_BUFFER
    static const unsigned char MaxBufferSize = KEYNETIK_PEDOMETER_MAX_BUFFER;
#else
    static const unsigned char MaxBufferSize = DefaultMaxBufferSize;
#endif
    typedef TCircularBuffer<MaxBufferSize, short> CircularBuffer;

    CircularBuffer::Index BufferSize(unsigned char p_timeWindow);

    void SaveAvgA(short p_aSquared);
    void AnalyzeStep(short p_aSpread);
    Event HandleEvent();
    void HandleStep();

    unsigned short m_stepRate;

    // Settings
    short m_StepThreshold;

    CircularBuffer m_avgA;
    CircularBuffer m_sprA;

    unsigned int m_readingsAboveZero;
    unsigned int m_stepCount;
    unsigned int m_readings;
    unsigned int m_firstStep;
    unsigned int m_lastStep;

    short m_lastMaxSpread;

    Event m_lastEvent;

   private:
    PedometerCore(const PedometerCore &);
    PedometerCore operator=(const PedometerCore &);

#ifndef KEYNETIK_EMBEDDED
    ColumnLogger *m_log;
#endif
};
}

#endif
