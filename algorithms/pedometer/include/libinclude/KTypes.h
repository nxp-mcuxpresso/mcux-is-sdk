#ifndef KTypes_h
#define KTypes_h
/*
============================================================================
Name        : KTypes.h
Author      : KEYnetik, Inc.
Version     :
Copyright   : Copyright 2009 by KEYnetik, Inc.
Description : declarations for types used through out
============================================================================
*/

namespace Keynetik
{
/**
* Accelerometer reading for a single axis
*/
typedef double AccVal;
const AccVal MaxAccVal = AccVal(1.7976931348623158e+308); // DBL_MAX
const AccVal MinAccVal = AccVal(2.2250738585072014e-308); // DBL_MIN

/**
* Axis identification
*/
typedef enum
{
    AxisUnknown = 0,
    AxisX,
    AxisY,
    AxisZ,
    AxisA
} Axis;

/**
* Confidence level. For a numeric value and an interval, describes how close the value is to the mean point of the
* interval.
*/
typedef enum
{
    ConfidenceUnknown = 0,
    ConfidenceVeryLow,
    ConfidenceLow,
    ConfidenceMedium,
    ConfidenceHigh,
    ConfidenceVeryHigh
} ConfidenceLevel;

template <typename T>
inline T Abs(T p_v)
{
    return p_v < 0 ? -p_v : p_v;
}
}

#ifdef __IAR_SYSTEMS_ICC__
// IAR STL templates are all defined in the global namespace, and namespace std does not exist.
// We need to define std on IAR to be able say "using namespace std" and refer to STL names without qualification,
// on all platforms
namespace std
{
}
#endif

#endif
