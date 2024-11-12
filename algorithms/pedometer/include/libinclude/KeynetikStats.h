#ifndef KEYNETIK_H
#define KEYNETIK_H
/*
============================================================================
 Name        : KeynetikStats.h
 Author      : KEYnetik, Inc.
 Version     :
 Copyright   : Copyright 2010 by KEYnetik, Inc.
 Description : Pedometry interface for Eve Pedometer project, step counter and statistics package
============================================================================
*/

#ifdef KEYNETIK_EMBEDDED
extern "C" {
#endif

/*
* Pedometry staistcics (since last KeynetikInitialize), populated by the latest call to KeynetikHandleIncomingEvent:
*/
/*
* Step count
*/
extern unsigned int keynetikStepCount;
/*
* Distance covered (in meters)
*/
extern unsigned int keynetikDistance;
/*
* Average speed (in meters per hour).
*/
extern unsigned int keynetikSpeed;
/*
* Calories buirned (since last KeynetikInitialize), populated by the latest call to KeynetikHandleIncomingEvent.
*/
extern unsigned int keynetikCalories;

/**
* Initialize the Keynetik library.
* \param oneG Value of 1G
* \param frequencyHz Accelerometer polling frequency, in readings per second
* \param isMale User's gender: 0 for female, non-0 for male
* \param heightCm User's height in centimeters. May be 0 if a custom stride length is to be used
* \param weightCm User's weight in kilograms
* \param strideCm Custom stride length in centimeters, or 0 for a standard stride length based on gender and height
* \param stepDelay Number of steps to expect before counting any steps (0 to turn off step filtering)
* \param delayWindow Amount of time within which these steps must be detected (in seconds; 0 to turn off step filtering)
*/
void KeynetikInitialize(unsigned int oneG,
                        unsigned int frequencyHz,
                        int isMale,
                        unsigned int heightCm,
                        unsigned int weightKg,
                        unsigned int strideCm,
                        unsigned int stepDelay,
                        unsigned int delayWindow);

/**
* Process an accelerometer reading and report detected events if any.
* \param x X axis reading
* \param y Y axis reading
* \param z Z axis reading
* \return 0 = no event, 1 = an event detected (check algorithm-specific external variables for details)
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
