#ifndef KProximityState_h
#define KProximityState_h
/*
============================================================================
Name        : KProximityState.h
Author      : KEYnetik, Inc.
Version     :
Copyright   : Copyright 2010 by KEYnetik, Inc.
Description : declarations for ProximityState
============================================================================
*/

namespace Keynetik
{
/**
* A description of the current proximity state, as reported by a proximity algorithm.
*/
class ProximityState
{
   public:
    /**
    * Possible proximity states
    */
    typedef enum
    {
        Unknown,
        Detected,
        NotDetected
    } State;

    /**
    * Default constructor. Sets state to Unknown
    */
    ProximityState() : m_state(Unknown)
    {
    }

    /**
    * Indicates whether Proxomity state have ever been updated
    * \return true if state is not Unknown (i.e. has been updated)
    */
    bool IsAvailable() const
    {
        return m_state != Unknown;
    }

    /**
    * Sets the state
    * \param p_state new state
    */
    void SetState(State p_state)
    {
        m_state = p_state;
    }
    /**
    * Gets the state
    * \return current state
    */
    State GetState() const
    {
        return m_state;
    }

    /**
    * Resets state to Unknown
    */
    void Reset();

   private:
    /**
    * Current proximity state
    */
    State m_state;
};
}

#endif
