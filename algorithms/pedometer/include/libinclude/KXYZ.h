#ifndef KXYZ_H
#define KXYZ_H
/*
============================================================================
Name        : KXYZ.h
Author      : KEYnetik, Inc.
Version     :
Copyright   : Copyright 2010 by KEYnetik, Inc.
Description : XYZ declaration
============================================================================
*/

#include "KMathUtilities.h"

namespace Keynetik
{
class XYZ
{
   public:
    XYZ()
    {
    }
    XYZ(Fixed p_x, Fixed p_y, Fixed p_z)
    {
        Set(p_x, p_y, p_z);
    }
    XYZ(const XYZ &p_that)
    {
        Set(p_that.X(), p_that.Y(), p_that.Z());
    }
    ~XYZ()
    {
    }

    XYZ &operator=(const XYZ &p_that)
    {
        Set(p_that.X(), p_that.Y(), p_that.Z());
        return *this;
    }
    bool operator==(const XYZ &p_that) const
    {
        return m_x == p_that.X() && m_y == p_that.Y() && m_z == p_that.Z();
    }
    bool operator!=(const XYZ &p_that) const
    {
        return !(*this == p_that);
    }
    XYZ operator+(const XYZ &p_that) const
    {
        return XYZ(X() + p_that.X(), Y() + p_that.Y(), Z() + p_that.Z());
    }
    XYZ operator-(const XYZ &p_that) const
    {
        return XYZ(X() - p_that.X(), Y() - p_that.Y(), Z() - p_that.Z());
    }

    Fixed X() const
    {
        return m_x;
    }
    Fixed Y() const
    {
        return m_y;
    }
    Fixed Z() const
    {
        return m_z;
    }

    void Set(Fixed p_x, Fixed p_y, Fixed p_z)
    {
        m_x = p_x;
        m_y = p_y;
        m_z = p_z;
    }
    void Clear()
    {
        Set(0, 0, 0);
    }

    Fixed Modulo() const
    {
        return MathUtilities::CalcA(*this);
    }

    void Unitize()
    {
        Fixed mod = Modulo();
        if (mod != Fixed(0))
        {
            m_x /= mod;
            m_y /= mod;
            m_z /= mod;
        }
    }

   protected:
    Fixed m_x;
    Fixed m_y;
    Fixed m_z;
};
}

#endif
