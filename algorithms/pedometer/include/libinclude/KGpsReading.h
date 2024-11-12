#ifndef KGpsReading_H
#define KGpsReading_H

/*
============================================================================
Name        : KGpsReading.h
Author      : KEYnetik, Inc.
Version     :
Copyright   : Copyright 2010 by KEYnetik, Inc.
Description : GpsReading declaration
============================================================================
*/

#include "KFixed.h"

namespace Keynetik
{
/**
*  Events associated with Gps-based distance reporting
*/
class GpsReading
{
   public:
    /**
    * Possible event IDs
    */
    typedef enum
    {
        Start,
        Stop,
        Update
    } EventID;

    /**
    * Phone line state
    */
    EventID eventID;
    /**
    * Distance (meters) after the last Start, only used if eventID = Update.
    */
    Fixed distanceM;

    /**
    * Constructor. Sets ID to Start, Distance to 0
    */
    GpsReading() : eventID(Start), distanceM(Fixed(0))
    {
    }
    /**
    * Constructor. Sets Distance to 0
    * \param p_id event Id
    */
    GpsReading(EventID p_id) : eventID(p_id), distanceM(Fixed(0))
    {
    }
    /**
    * Constructor. Sets Id to Update
    * \param p_dist distance in meters
    */
    GpsReading(Fixed p_dist) : eventID(Update), distanceM(p_dist)
    {
    }
};
}

#endif
