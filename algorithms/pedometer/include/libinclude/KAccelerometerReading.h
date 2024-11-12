#ifndef KAccelerometerReading_H
#define KAccelerometerReading_H
/*
============================================================================
Name        : KAccelerometerReading.h
Author      : KEYnetik, Inc.
Version     :
Copyright   : Copyright 2008 by KEYnetik, Inc.
Description : AccelerometerReading declaration
============================================================================
*/

#include "KTypes.h"
#include "KXYZ.h"

namespace Keynetik
{
/**
*  A single accelerometer reading.
*  The reading contains values of X, Y, and Z axes
*
* NOTE:
* As a matter of convention throughout the SDK, users are responsible for making sure
* that raw accelerometer readings are converted as follows (assuming a Portrait orientation):
* - the X axis goes from the left of the screen to the right
* - the Y axis goes from the top of the screen to the bottom
* - the Z axis goes from behind the screen towards the user
*/
class AccelerometerReading
{
   public:
/**
* Maximum supported sensor polling frequency, in readings per second (Hz)
*/
#ifdef KEYNETIK_MAX_FREQUENCY
    static const unsigned short MaxFrequency = KEYNETIK_MAX_FREQUENCY;
#else
    static const unsigned short MaxFrequency = 100;
#endif
    /**
    * Minimum supported sensor polling frequency, in readings per second (Hz)
    */
    static const unsigned short MinFrequency = 10;

   public:
    /**
    * Reinitialize allocation of accelerometer reading IDs
    */
    inline static void ResetNextID()
    {
        m_nextID = 0;
    }

    /**
          * Set the global setting representing the 1G value (e.g. reading on an axis aligned with gravity)
          * for the current accelerometer.
          * \param p_val 1G value
          */
    static bool Set1G(Fixed p_val);
    /**
          * Set the global setting representing the current accelerometer polling frequency, in Hz
          * \param p_readingsPerSecond polling frequency, in Hz (readings per second)
          * \return false if frequency is out of bounds (MinFrequency..MaxFrequency)
          */
    static bool SetFrequency(unsigned short p_readingsPerSecond);

    /**
          * Query the 1G value for the current accelerometer
          * \return 1G value
          */
    static Fixed Get1G()
    {
        return m_1G;
    }
    /**
          * Query the current accelerometer polling frequency
          * \return accelerometer polling frequency, in Hz
          */
    static unsigned short GetFrequency()
    {
        return m_frequency;
    }

   public:
    /**
    *  Compare two events.
    *  Two AccelerometerReading objects match if they have matching axis values.
    *  \param p_event an event object to compare to
    *  \return true if values of the axes match, false otherwise. Event IDs are not compared.
    **/
    bool Compare(const AccelerometerReading &p_event) const
    {
        return GetXYZ().X() == p_event.GetXYZ().X() && GetXYZ().Y() == p_event.GetXYZ().Y() &&
               GetXYZ().Z() == p_event.GetXYZ().Z();
    }

   public:
    /**
    * Default constructor creates an accelerometer reading and initializes all axes values to 0.0
    */
    AccelerometerReading();
    /**
    * Creates an accelerometer reading and initializes all axes values to the user specified values
    * \param p_x X axis value
    * \param p_y Y axis value
    * \param p_z Z axis value
    * \param normalized if false, the axis values will be divided by Get1G()
    */
    AccelerometerReading(Fixed p_x, Fixed p_y, Fixed p_z, bool normalized = false);
    /**
    *  Copy constructor
    * \param source source object
    */
    AccelerometerReading(const AccelerometerReading &source);

    AccelerometerReading &operator=(const AccelerometerReading &source);

    /**
    *  Sets all axes values to 0.0
    */
    void Clear();

    /**
    * Get the user specified accelerometer axis value
    * \param p_axis specifies the axis for which to retrieve the value
    * \return The value of the user specified axis
    */
    Fixed AxisValue(Axis p_axis) const;

    /**
          * Get unique identifier of this reading
          * \return The value of Z
          */
    unsigned int GetReadingID() const
    {
        return id;
    }

    /**
    * Set reading to user specified values
    * \param p_x X axis value of the new accelerometer reading
    * \param p_y Y axis value of the new accelerometer reading
    * \param p_z Z axis value of the new accelerometer reading
    * \param normalized if false, the axis values will be divided by Get1G()
    */
    void Set(Fixed p_x, Fixed p_y, Fixed p_z, bool normalized = false);

    const XYZ &GetXYZ() const
    {
        return m_xyz;
    }

    Fixed X() const
    {
        return m_xyz.X();
    }
    Fixed Y() const
    {
        return m_xyz.Y();
    }
    Fixed Z() const
    {
        return m_xyz.Z();
    }

    /**
    * Compute A for this reading. A will only be computed upon the first
    * request and subsequently, only upon the first request after
    * a change to x, y or z values has been made.
    * \return The value of A
    */
    Fixed A() const;

   protected:
    class XYZ_Access : public XYZ
    {
       public:
        XYZ_Access()
        {
        }
        XYZ_Access &operator=(const XYZ &p_that)
        {
            XYZ::operator=(p_that);
            return *this;
        }
        Fixed &GetX()
        {
            return m_x;
        }
        Fixed &GetY()
        {
            return m_y;
        }
        Fixed &GetZ()
        {
            return m_z;
        }
    };
    XYZ_Access m_xyz;
    /**
    * Acceleration value
    */
    mutable Fixed a;

    /**
    * Reading ID
    */
    unsigned int id;

   protected:
    /**
          * Current 1G value
          */
    static Fixed m_1G;
    /**
          * Current polling frequency
          */
    static unsigned short m_frequency;
    /**
          * Next ID to be assigned to a new and unique AccelerometerReading
          */
    static unsigned int m_nextID;
};
}

#endif
