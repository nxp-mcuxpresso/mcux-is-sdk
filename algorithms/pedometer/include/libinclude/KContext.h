#ifndef KContext_h
#define KContext_h

/*
============================================================================
Name        : KContext.h
Author      : KEYnetik, Inc.
Version     :
Copyright   : Copyright 2010 by KEYnetik, Inc.
Description : declarations for Context
============================================================================
*/

#include "KSharedBuffer.h"
#include "KRestState.h"
#include "KProximityState.h"

namespace Keynetik
{
/**
* Global data for AlgorithmCores library
*/
class Context
{
   public:
    static SharedBuffer &GetSharedBuffer();
    static RestState &GetRestState();
    static ProximityState &GetProximityState();

    /**
    * Reset all states, buffers and accelerometer IDs
    */
    static void GlobalReset();
    /**
    * Reset rest state, shared buffer and accelerometer IDs
    */
    static void AccelerometerReset();

   private:
    Context(void);
    Context(const Context &);
    Context &operator=(const Context &);
    ~Context(void);
};
}

#endif
