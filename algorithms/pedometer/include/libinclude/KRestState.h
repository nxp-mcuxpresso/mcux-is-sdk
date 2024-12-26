#ifndef KRestState_h
#define KRestState_h
/*
============================================================================
Name        : KRestState.h
Author      : KEYnetik, Inc.
Version     :
Copyright   : Copyright 2010 by KEYnetik, Inc.
Description : declarations for Rest State
============================================================================
*/

#include "KTypes.h"
#include "KFixed.h"

namespace Keynetik
{
/**
   * A description of the current rest state, as reported by a rest detection algorithm.
   * If the device is at rest, provides average values on all axes.
   */
class RestState
{
   public:
    /**
    * Possible Rest states
    */
    typedef enum
    {
        AtRest,
        Moving,
        Unknown
    } State;

    /**
    * Default constructor. Sets state to Unknown
    */
    RestState();
    /**
    * Sets state and average readings
    * \param p_state the new state
    * \param p_avgX average X reading for the Rest state; ignored if p_state is not AtRest
    * \param p_avgY average Y reading for the Rest state; ignored if p_state is not AtRest
    * \param p_avgZ average Z reading for the Rest state; ignored if p_state is not AtRest
    */
    void SetState(State p_state, AccVal p_avgX = 0.0, AccVal p_avgY = 0.0, AccVal p_avgZ = 0.0)
    {
        SetState(p_state, Fixed(p_avgX), Fixed(p_avgY), Fixed(p_avgZ));
    }
    /**
    * Sets state and average readings
    * \param p_state the new state
    * \param p_avgX average X reading for the Rest state; ignored if p_state is not AtRest
    * \param p_avgY average Y reading for the Rest state; ignored if p_state is not AtRest
    * \param p_avgZ average Z reading for the Rest state; ignored if p_state is not AtRest
    */
    void SetState(State p_state, Fixed p_avgX, Fixed p_avgY, Fixed p_avgZ);

    /**
    * Gets the state
    * \return current state
    */
    State GetState() const
    {
        return state;
    }
    /**
    * Gets the average readings for the latest rest state.
    * \param[out] p_avgX average value on X
    * \param[out] p_avgY average value on Y
    * \param[out] p_avgZ average value on Z
    * \return if at rest, average readings. All 0s otherwise
    */
    void GetAveragesAtRest(AccVal &p_avgX, AccVal &p_avgY, AccVal &p_avgZ) const;
    /**
    * Gets the average readings for the latest rest state.
    * \param[out] p_avgX average value on X
    * \param[out] p_avgY average value on Y
    * \param[out] p_avgZ average value on Z
    * \return if at rest, average readings. All 0s otherwise
    */
    void GetAveragesAtRest(Fixed &p_avgX, Fixed &p_avgY, Fixed &p_avgZ) const;

    /**
    * Resets state to Unknown
    */
    void Reset();

   private:
    RestState(const RestState &);
    RestState &operator=(const RestState &);

   private:
    /**
    * Current state
    */
    State state;
    /**
    * Accelerometer averages at rest
    */
    Fixed averageX_AtRest;
    Fixed averageY_AtRest;
    Fixed averageZ_AtRest;
};
}

#endif
