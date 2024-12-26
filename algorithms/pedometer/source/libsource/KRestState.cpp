/*
============================================================================
 Name        : KRestState.cpp
 Author      : KEYnetik, Inc.
 Version     :
 Copyright   : Copyright 2010 by KEYnetik, Inc.
 Description : definition of class RestState
============================================================================
*/

#include "KRestState.h"

using namespace Keynetik;

RestState::RestState()
: state(Unknown), 
  averageX_AtRest(0),
  averageY_AtRest(0),
  averageZ_AtRest(0)
{
}

void 
RestState::GetAveragesAtRest(Fixed& p_avgX, Fixed& p_avgY, Fixed& p_avgZ) const 
{ 
	p_avgX=averageX_AtRest; 
	p_avgY=averageY_AtRest; 
	p_avgZ=averageZ_AtRest;  
}

void 
RestState::GetAveragesAtRest(AccVal& p_avgX, AccVal& p_avgY, AccVal& p_avgZ) const 
{ 
	p_avgX=averageX_AtRest.ToDouble(); 
	p_avgY=averageY_AtRest.ToDouble(); 
	p_avgZ=averageZ_AtRest.ToDouble();  
}

void 
RestState::SetState(State p_state, Fixed p_avgX, Fixed p_avgY, Fixed p_avgZ)
{
    state=p_state;

    if( state != AtRest )
    {
        averageX_AtRest = averageY_AtRest = averageZ_AtRest = 0;
    }
    else
    {
        averageX_AtRest = p_avgX;
		averageY_AtRest = p_avgY;
		averageZ_AtRest = p_avgZ;
    }
}

void 
RestState::Reset() 
{ 
    SetState(RestState::Unknown, Fixed(0), Fixed(0), Fixed(0));
}
