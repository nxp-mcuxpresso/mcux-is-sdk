#ifndef FIXED_H
#define FIXED_H
/*
============================================================================
Name        : KFixed.h
Author      : KEYnetik, Inc.
Version     :
Copyright   : Copyright 2010 by KEYnetik, Inc.
Description : declarations for class Fixed
============================================================================
*/

namespace Keynetik
{
/**
* Fixed point arithmetics for use in the AlgorithmCore classes.
*/
class Fixed
{
   public:
    /**
    * Maximum supported value
    */
    static const Fixed MaxValue;
    /**
    * Minimum supported value (negative). Same as -MaxValue.
    */
    static const Fixed MinValue;
    /**
    * Number of bits used to represent the fractional part of the number
    */
    static const unsigned int FractionalBits = 14;

   public:
    /**
    * Default constructor. Sets value to 0
    */
    Fixed() : m_value(0)
    {
    }
    /**
    * Copy constructor
    * \param p_other value to copy from
    */
    Fixed(const Fixed &p_other) : m_value(p_other.m_value)
    {
    }
    /**
    * Construction from an int
    * \param p_value int value to construct from
    */
    explicit Fixed(int p_value) : m_value((long)p_value << FractionalBits)
    {
    }
    /**
    * Construction from an unsigned int
    * \param p_value unsigned int value to construct from
    */
    explicit Fixed(unsigned int p_value) : m_value((long)p_value << FractionalBits)
    {
    }

    /**
    * Construction from a double. Do not use to construct from literal constants,
    * use KEYNETIK_FIXED_CONST macro instead (to avoid linking in double precision system library)
* Overflow possible; result undefined!
    * \param p_value double value to construct from
    */
    explicit Fixed(double p_value);

/**
* Macro for defining Fixed constants initialized as a literal constant, with conversion performed at compile time
* Overflow possible; result undefined!
*/
#define KEYNETIK_FIXED_CONST(x) ((void *)long(x * 16384))
    /**
    * No-conversion constructor, for use in KEYNETIK_FIXED_CONST.
    * Should not be used directly.
    * \param p_value value to store in the object.
    */
    Fixed(void *p_value) : m_value((long)p_value)
    {
    }

    /**
    * Equality operator
    * \param p_other value to compare with
    * \return true if values are equal
    */
    bool operator==(Fixed p_other) const
    {
        return m_value == p_other.m_value;
    }
    /**
    * Inequality operator
    * \param p_other value to compare with
    * \return true if values are not equal
    */
    bool operator!=(Fixed p_other) const
    {
        return m_value != p_other.m_value;
    }
    /**
    * Less Than operator
    * \param p_other value to compare with
    * \return true if this is less than other
    */
    bool operator<(Fixed p_other) const
    {
        return m_value < p_other.m_value;
    }
    /**
    * LessThanOr Equal operator
    * \param p_other value to compare with
    * \return true if this is less than or equal to other
    */
    bool operator<=(Fixed p_other) const
    {
        return m_value <= p_other.m_value;
    }
    /**
    * GreaterThan operator
    * \param p_other value to compare with
    * \return true if this is greater than other
    */
    bool operator>(Fixed p_other) const
    {
        return m_value > p_other.m_value;
    }
    /**
    * GreaterThanOrEqual operator
    * \param p_other value to compare with
    * \return true if this is greater than or equal to other
    */
    bool operator>=(Fixed p_other) const
    {
        return m_value >= p_other.m_value;
    }

    /**
    * Addition operator
* Overflow possible; result undefined!
    * \param p_other value to add
    * \return the result of addition
    */
    Fixed operator+(Fixed p_other) const
    {
        Fixed ret;
        ret.m_value = m_value + p_other.m_value;
        return ret;
    }
    /**
    * Subtraction operator
* Overflow possible; result undefined!
    * \param p_other value to subtract
    * \return the result of subtraction
    */
    Fixed operator-(Fixed p_other) const
    {
        Fixed ret;
        ret.m_value = m_value - p_other.m_value;
        return ret;
    }
    /**
    * Unary negation operator
    * \return the result of negation
    */
    Fixed operator-() const
    {
        Fixed ret;
        ret.m_value = -m_value;
        return ret;
    }

    /**
    * Assignment operator
    * \param p_other value to assign from
    * \return reference to the object assigned to (this)
    */
    Fixed &operator=(Fixed p_other)
    {
        m_value = p_other.m_value;
        return *this;
    }
    /**
    * Addition assignment operator
* Overflow possible; result undefined!
    * \param p_other value to increment by
    * \return reference to the object assigned to (this)
    */
    Fixed &operator+=(Fixed p_other)
    {
        m_value += p_other.m_value;
        return *this;
    }
    /**
    * Subtraction assignment operator
    * Overflow possible; result undefined!
* \param p_other value to decrement by
    * \return reference to the object assigned to (this)
    */
    Fixed &operator-=(Fixed p_other)
    {
        m_value -= p_other.m_value;
        return *this;
    }

    /**
    * Multiplication operator
* Overflow possible; result MaxValue or MinValue!
    * \param p_other value to multiply by
    * \return result of multiplication
    */
    Fixed operator*(Fixed p_other) const;
    /**
    * Square function. Multiplies the value by itself.
* Overflow possible; result MaxValue or MinValue!
    * \return reference to the object squared (this)
    */
    Fixed &Square();
    /**
    * Division operator
* Overflow possible; result MaxValue or MinValue!
    * \param p_other value to divide by
    * \return result of division
    */
    Fixed operator/(Fixed p_other) const;
    /**
    * Multiplication assignment operator
* Overflow possible; result MaxValue or MinValue!
    * \param p_other value to multiply by
    * \return reference to the object assigned to (this)
    */
    Fixed &operator*=(Fixed p_other);
    /**
    * Division assignment operator
    * Overflow possible; result MaxValue or MinValue!
* \param p_other value to divide by
    * \return reference to the object assigned to (this)
    */
    Fixed &operator/=(Fixed p_other)
    {
        *this = *this / p_other;
        return *this;
    }

    /**
    * Conversion to int. Returns the higherst integer lower than or equal to the value, same as floor().
    * \return integer part of the value
    */
    int ToInt() const
    {
        return (int)(m_value >> FractionalBits);
    }
    /**
    * Round to the nearest integer.
    * \return nearest integer value
    */
    int Round() const;

    /**
    * Conversion to double
    * \return converted value
    */
    double ToDouble() const;
    operator double() const
    {
        return ToDouble();
    }

    /**
    * Access to the internal representation.
    * \return internal representation.
    */
    long GetInternal() const
    {
        return m_value;
    }

    /**
    * Square root function
    * \return square root of the object's value
    */
    Fixed Sqrt() const;
    /**
    * Absolute value of the object
    * \return Absolute value of the object
    */
    Fixed Abs() const
    {
        Fixed ret;
        ret.m_value = m_value < 0 ? -m_value : m_value;
        return ret;
    }
    /**
    * Sign of the object
    * \return Sign of the object (-1, 0, or 1)
    */
    int Sign() const
    {
        if (m_value > 0)
            return 1;
        else if (m_value < 0)
            return -1;
        return 0;
    }
    /**
    * Divide value by a power of 2 by shifting
* Underflow possible; result 0
    * \param p_pow power of 2 to divide by, default value 1
    * \return result of division
    */
    Fixed DivPow2(unsigned int p_pow = 1) const
    {
        Fixed ret;
        ret.m_value = m_value >> p_pow;
        return ret;
    }
    /**
    * Multiply value by a power of 2 by shifting
* Overflow possible; result undefined!
    * \param p_pow power of 2 to multiply by, default value 1
    * \return result of multiplication
    */
    Fixed MultPow2(unsigned int p_pow = 1) const
    {
        Fixed ret;
        ret.m_value = m_value << p_pow;
        return ret;
    }

    /**
    * Calculate arccosine of the value, in degrees. If value is outside of range [-1, 1], returns MinValue or MaxValue
    * depending on the sign
    * \return arccosine (in degrees from 0 to 180), or MinValue/MaxValue
    */
    Fixed Arccos() const;

    /**
    * Calculate arctangent of the value, in degrees.
    * \return arctan (in degrees from -90 to 90)
    */
    Fixed Arctan() const;

    /**
    * Negation. Changes the sign of the value
    */
    void Negate()
    {
        m_value = -m_value;
    }

   private:
    long m_value;
};

/**
* Maximum of 2 Fixed values
* \param p_a object a
* \param p_b object b
* \return the larger of a and b
*/
inline Fixed Max(Fixed p_a, Fixed p_b)
{
    return p_a < p_b ? p_b : p_a;
}

} // namespace Keynetik;

#endif // _FIXED_H
