/*
============================================================================
 Name        : KContext.cpp
 Author      : KEYnetik, Inc.
 Version     :
 Copyright   : Copyright 2010 by KEYnetik, Inc.
 Description : Context definition
============================================================================
*/

#include "KContext.h"

using namespace Keynetik;

/**
* Shared accelerometer reading buffer for use in Algorithm Cores
*/
static DefaultSharedBuffer g_sharedBuffer;
/**
* Current rest/motion state
*/
static RestState g_restState;
/**
* Current proximity state
*/
static ProximityState g_proximityState;

SharedBuffer& 
Context::GetSharedBuffer()
{
    return g_sharedBuffer;
}

RestState& 
Context::GetRestState()
{
    return g_restState;
}

ProximityState& 
Context::GetProximityState()
{
    return g_proximityState;
}

void 
Context::AccelerometerReset()
{
    AccelerometerReading::ResetNextID(); 
    g_sharedBuffer.Clear();
	g_restState.Reset();
}

void 
Context::GlobalReset()
{
	AccelerometerReset();
	g_proximityState.Reset();
}
