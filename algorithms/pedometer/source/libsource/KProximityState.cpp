/*
============================================================================
 Name        : KProximityState.cpp
 Author      : KEYnetik, Inc.
 Version     :
 Copyright   : Copyright 2010 by KEYnetik, Inc.
 Description : definition of class ProximityState
============================================================================
*/

#include "KProximityState.h"

using namespace Keynetik;

void 
ProximityState::Reset() 
{ 
    SetState(ProximityState::Unknown); 
}
