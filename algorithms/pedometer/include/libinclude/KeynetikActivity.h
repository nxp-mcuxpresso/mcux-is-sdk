#ifndef KEYNETIKACTIVITY_H
#define KEYNETIKACTIVITY_H
/*
============================================================================
 Name        : KeynetikActivity.h
 Author      : KEYnetik, Inc.
 Version     :
 Copyright   : Copyright 2010 by KEYnetik, Inc.
 Description : PedometerActivity interface for Eve Pedometer project
============================================================================
*/

#ifdef KEYNETIK_EMBEDDED
extern "C" {
#endif

/*
* Global step count populated by the latest call to KeynetikHandleIncomingEvent.
*/
extern unsigned int keynetikStepCount;
/*
* Current activity level, updated with every call to KeynetikHandleIncomingEvent
*/
typedef enum
{
    KeynetikActivityUnknown = 0,
    KeynetikActivityRest = 1,
    KeynetikActivityWalking = 2,
    KeynetikActivityJogging = 3,
    KeynetikActivityRunning = 4
} KeynetikActivityLevel;
extern KeynetikActivityLevel keynetikActivityLevel;

/**
* Initialize the Keynetik library.
* \param oneG value of 1G
* \param frequencyHz accelerometer polling frequency, in readings per second
*/
void KeynetikInitialize(unsigned int oneG, unsigned int frequencyHz);

/**
* Process an accelerometer reading and report detected events if any.
* \param x X axis reading
* \param y Y axis reading
* \param z Z axis reading
* \return 0 = no event, 1 = a Step event detected
*/
int KeynetikHandleIncomingEvent(int x,
                                int y,
                                int z); // 0 = no event, 1= a step detected (keynetikStepCount incremented)

/**
* Finalize the Keynetik library.
*/
void KeynetikTerminate();

#ifdef KEYNETIK_EMBEDDED
}
#endif

#endif
