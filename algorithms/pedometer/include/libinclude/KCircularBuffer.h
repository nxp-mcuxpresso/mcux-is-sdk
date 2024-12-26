#ifndef KCircularBuffer_h
#define KCircularBuffer_h
/*
============================================================================
Name        : KCircularBuffer.h
Author      : KEYnetik, Inc.
Version     :
Copyright   : Copyright 2008 by KEYnetik, Inc.
Description : declarations for TCircularBuffer
============================================================================
*/

#include "KTypes.h"

namespace Keynetik
{
class CircularBufferImpl
{
   public:
    typedef unsigned short Index;
    CircularBufferImpl() : m_size(0), m_curIdx(0), m_full(false)
    {
    }

    /**
    * Returns the size of the buffer
    * \return Maximum number of values allowed in this buffer
    */
    Index GetSize() const
    {
        return m_size;
    }
    /**
    * Get the index of the oldest element in the buffer.
    * \return The index of the oldest element in the buffer.
    */
    Index GetOldestIdx() const
    {
        return m_full ? m_curIdx : 0;
    }
    /**
    * Get the index to which the next value will be saved.
    * \return The index to which the next value will be saved.
    */
    Index GetCurIdx() const
    {
        return m_curIdx;
    }
    /**
    * Get the index of the newest element.
    * \return The index of the newest saved element. 0 if the buffer is empty.
    */
    Index GetNewestIdx() const
    {
        return m_curIdx == 0 ? (m_full ? GetSize() - 1 : 0) : m_curIdx - 1;
    }
    /**
    * Get the index following the specified.
    * \param p_idx the index for which to calculate the following
    * \return index following p_idx (0 if p_idx == GetSize()-1)
    *           If p_idx is greater than the size of the buffer, returns 0.
    */
    Index GetNextIdx(Index p_idx) const
    {
        return p_idx >= GetSize() - 1 ? 0 : p_idx + 1;
    }
    /**
    * Get the index preceding the specified.
    * \param p_idx the index for which to calculate the previous
    * \return index previous to p_idx (GetSize()-1 if p_idx == 0).
    *           If p_idx is greater than the size of the buffer, returns the last index in the buffer.
    */
    Index GetPrevIdx(Index p_idx) const
    {
        return (p_idx == 0 || p_idx > GetSize()) ? GetSize() - 1 : p_idx - 1;
    }
    /**
    * Checks if the number of values currently in the buffer is equal to the user specified size of the buffer.
    * \return true if buffer is full, false otherwise
    */
    bool IsFull() const
    {
        return m_full;
    }

    bool IsEmpty() const
    {
        return !m_full && m_curIdx == 0;
    }

    /**
    * Determines how far an index lies from the current end of the circular buffer.
    * \param p_index Index to check for.
    * \return Distance from the specified index to curIdx, -1 if p_index is out of bounds.
    */
    int DistanceFromEnd(Index p_index) const
    {
        if (p_index >= m_size)
        {
            return -1;
        }
        if (p_index <= m_curIdx)
        {
            return (int)(m_curIdx - p_index);
        }
        return (int)(m_size - p_index + m_curIdx);
    }

   protected:
    /**
    * Assigns p_a and p_b to p_first and p_second based on which of p_a/p_b was entered into the circular buffer first
    * \param p_a first index
    * \param p_b second index
    * \param p_cur current index
    * \param p_first output; assigned p_a if p_a occured before p_b, p_b otherwise
    * \param p_second output; assigned p_b if p_a occured before p_b, p_a otherwise
    */
    static void CalcOrder(Index p_a, Index p_b, Index p_cur, Index *p_first, Index *p_second)
    {
        if (p_a >= p_cur)
        {
            if (p_b > p_a || p_b < p_cur)
            {
                *p_first = p_a;
                *p_second = p_b;
            }
            else
            {
                *p_first = p_b;
                *p_second = p_a;
            }
        }
        else // p_a < p_cur
        {
            if (p_b < p_a || p_b >= p_cur)
            {
                *p_first = p_b;
                *p_second = p_a;
            }
            else
            {
                *p_first = p_a;
                *p_second = p_b;
            }
        }
    }

    /**
    * The number of values currently allowed in the buffer
    */
    Index m_size;
    /**
    * The index at which the next insertion will occur
    */
    Index m_curIdx;
    /**
    * true if the buffer contains m_size values
    */
    bool m_full;
};

/**
* A circular buffer of values, of size <= MaxBufferSize
* No memory is dynamically allocated
*/
template <unsigned short TMaxBufferSize, typename T>
class TCircularBuffer : public CircularBufferImpl
{
   public:
    /**
    * The type used to index values in the TCircularBuffer
    */
    typedef CircularBufferImpl::Index Index;
    const static unsigned short MaxBufferSize = TMaxBufferSize;
    typedef T ElementType;

   public:
    /**
    * Initializes all the buffer values to 0.
    * \param p_size Specifies the maximum number of values allowed in the buffer.
    *               If p_size is greater than MaxBufferSize, buffer size is set to MaxBufferSize
    */
    TCircularBuffer(Index p_size = MaxBufferSize);

    /**
          * Returns the number of occupied slots in the buffer (same as GetSize when buffer is full, otherwise less than
     * GetSize())
          * \return The number of occupied slots in the buffer
          */
    Index GetOccupied() const
    {
        return m_full ? m_size : m_curIdx;
    }

    /**
    * Sets all the buffer values to 0
    */
    void Clear()
    {
        Resize(GetSize());
    }

    /**
    * Deletes all the data currently in the buffer and sets the size to the user specified size
    * \param p_newSize Specifies the maximum number of values allowed in the buffer.
    *               If p_newSize is greater than MaxBufferSize, buffer size is set to MaxBufferSize
    */
    void Resize(Index p_newSize);

    /**
    * Gets the value stored in the buffer at the user specified index
    * \param p_index Index at which to retrieve the value.
    * \return Value found at user specified index. 0.0 is returned if p_index is out of bounds.
    */
    T GetData(Index p_index) const;

    /**
    * Get the value of the newest element.
    * \return The value of the newest saved element. 0 if the buffer is empty.
    */
    T GetNewest() const
    {
        return m_curIdx == 0 ? (m_full ? m_data[GetSize() - 1] : 0) : m_data[m_curIdx - 1];
    }
    /**
    * Get the value of the oldest element.
    * \return The value of the oldest saved element. 0 if the buffer is empty.
    */
    T GetOldest() const
    {
        return m_full ? m_data[m_curIdx] : (m_curIdx == 0 ? 0 : m_data[0]);
    }

    /**
    * Appends the user specified value to the end of the buffer
    * \param p_value New value to be added to the buffer
    */
    void Append(T p_value);

    /**
    * Helper method for calculating the maximum spread between all the values currently in the buffer.
    * The order of the values is not significant.
    * \return Difference between the minimum and maximum values in the buffer.
    */
    T MaxSpread() const;

    /**
    * Helper method for calculating the maximum spread between all the values within the user specified range.
    * The order of the values is not significant.
    * \param p_first Index of the first value in the range.
    * \param p_second Index of the last value in the range.
    * \return Difference between the minimum and maximum values in the user specified range.
    */
    T MaxSpread(Index &p_first, Index &p_second) const;

    /**
    * Helper method for calculating the average of all the values currently in the buffer.
    * \return Average of all buffer values.
    */
    T Average() const;

    /**
    * Helper method for calculating the average of the last p_length values in the buffer.
    * \param  p_length  Number of values to average
    * \return Average of last p_length values in the buffer.
    */
    T Average(Index p_length) const;

    /**
    * Helper method for calculating the average of arbitrary segment of values in the buffer.
    * \param  p_start Starting index of the segment to average
    * \param  p_length  Number of values to average. The segment cannot go beyond the newest element, thus the actual
    * number
    * of averaged values can be less than p_length.
    * \return Average of p_length values starting from idex p_start.
    */
    T Average(Index p_start, Index p_length) const;

    /**
    * Calculating the variance (mean of squared deviations from the average) of all values in the buffer.
    * \return Variance of last p_length values in the buffer.
    */
    T Variance() const;

    /**
    * Helper method for calculating the variance of the last p_length values in the buffer.
    * \param  p_average Average of last p_length values in the buffer
    * \param  p_length  Number of values to average
    * \return Variance of last p_length values in the buffer.
    */
    T Variance(T p_average, Index p_length) const;

    /**
    * Calculating "noise", which is average distance between two adjacent values.
    */
    T Noise() const;

    /**
    * Finds the index of the maximum element in the buffer.
    * If more than one element have the buggest value, returns the one inserted the latest.
    * Returns 0 if the buffer is empty.
    * \return Index of the element with the biggest value.
    */
    Index IdxOfMax() const
    { // implemented here due to compiler issues
        Index ret = m_size;
        Index idx = GetNewestIdx();
        T max;
        for (unsigned int i = 0; i < GetOccupied(); ++i)
        {
            T val = GetData(idx);
            if (ret == m_size || max < val)
            {
                max = val;
                ret = idx;
            }
            idx = GetPrevIdx(idx);
        }
        if (ret == m_size)
        {
            return 0;
        }
        return ret;
    }

    T Sum() const
    {
        T ret(0);
        for (unsigned int i = 0; i < GetOccupied(); ++i)
        {
            ret += m_data[i];
        }
        return ret;
    }

   private:
    /**
    * Data buffer
    */
    T m_data[MaxBufferSize];
};

/**
* Standard buffer size
*/
static const unsigned char DefaultMaxBufferSize = 20;
/**
* Standard buffer of accelerometer axis values used in motion algorithms
*/
typedef TCircularBuffer<DefaultMaxBufferSize, AccVal> CircularBuffer;

// Implementation section

template <unsigned short MaxBufferSize, typename T>
TCircularBuffer<MaxBufferSize, T>::TCircularBuffer(Index p_size)
{
    Resize(p_size);
}

template <unsigned short MaxBufferSize, typename T>
T TCircularBuffer<MaxBufferSize, T>::GetData(Index p_idx) const
{
    if (p_idx >= m_size)
    {
        return T(0);
    }
    return m_data[p_idx];
}

template <unsigned short MaxBufferSize, typename T>
void TCircularBuffer<MaxBufferSize, T>::Append(T p_value)
{
    m_data[m_curIdx] = p_value;
    ++m_curIdx;
    if (m_curIdx >= m_size)
    {
        m_full = true;
        m_curIdx = 0;
    }
}

template <unsigned short MaxBufferSize, typename T>
T TCircularBuffer<MaxBufferSize, T>::MaxSpread() const
{
    Index f, c;
    return MaxSpread(f, c);
}

template <unsigned short MaxBufferSize, typename T>
T TCircularBuffer<MaxBufferSize, T>::MaxSpread(Index &p_first, Index &p_second) const
{
    if (!IsFull())
    {
        return T(0);
    }

    Index minIdx = 0;
    T minVal = m_data[0];
    Index maxIdx = 0;
    T maxVal = m_data[0];
    Index i;
    for (i = 1; i < m_size; ++i)
    {
        T curVal = m_data[i];
        if (curVal < minVal)
        {
            minVal = curVal;
            minIdx = i;
        }
        else if (curVal > maxVal)
        {
            maxVal = curVal;
            maxIdx = i;
        }
    }

    // determine which of the minimum and maximum occurred first
    Index first;
    Index second;
    CalcOrder(minIdx, maxIdx, m_curIdx, &first, &second);
    p_first = first;
    p_second = second;
    return m_data[p_second] - m_data[p_first];
}

template <unsigned short MaxBufferSize, typename T>
T TCircularBuffer<MaxBufferSize, T>::Average() const
{
    if (m_curIdx == 0 && !IsFull())
    {
        return T(0);
    }

    T sum = 0;
    Index i;
    Index count = IsFull() ? m_size : m_curIdx;
    for (i = 0; i < count; ++i)
    {
        sum += m_data[i];
    }
    return sum / T(count);
}

template <unsigned short MaxBufferSize, typename T>
T TCircularBuffer<MaxBufferSize, T>::Average(Index p_length) const
{
    if (p_length > GetOccupied())
    {
        p_length = GetOccupied();
    }
    if (p_length == 0)
    {
        return 0;
    }
    // calculate average in relevant part of buffer
    Index idx = GetNewestIdx();
    T average(0);
    T tw(p_length);
    for (Index i = 0; i < p_length; ++i)
    {
        average += GetData(idx) / tw;
        idx = GetPrevIdx(idx);
    }

    return average;
}

template <unsigned short MaxBufferSize, typename T>
T TCircularBuffer<MaxBufferSize, T>::Average(Index p_start, Index p_length) const
{
    // calculate average in relevant part of buffer
    Index idx = p_start;
    T sum(0);
    Index count(0);
    for (Index i = 0; i < p_length; ++i)
    {
        sum += GetData(idx);
        ++count;
        if (idx == GetNewestIdx())
        {
            break;
        }
        idx = GetNextIdx(idx);
    }

    return count == 0 ? T(0) : (sum / T(count));
}

template <unsigned short MaxBufferSize, typename T>
T TCircularBuffer<MaxBufferSize, T>::Variance() const
{
    if (m_curIdx == 0 && !IsFull())
    {
        return T(0);
    }

    T average = Average();
    T sum = 0;
    Index count = IsFull() ? m_size : m_curIdx;
    for (Index i = 0; i < count; ++i)
    {
        T dev = m_data[i] - average;
        sum += dev * dev;
    }
    return sum / T(count);
}

template <unsigned short MaxBufferSize, typename T>
T TCircularBuffer<MaxBufferSize, T>::Variance(T p_average, Index p_length) const
{
    if (p_length > GetOccupied())
    {
        p_length = GetOccupied();
    }
    if (p_length == 0)
    {
        return 0;
    }
    // calculate variance in relevant part of buffer
    Index idx = GetNewestIdx();
    T sum(0);
    for (Index i = 0; i < p_length; ++i)
    {
        T data = GetData(idx);
        sum += (data - p_average) * (data - p_average);
        idx = GetPrevIdx(idx);
    }
    return sum / T(p_length);
}

template <unsigned short MaxBufferSize, typename T>
T TCircularBuffer<MaxBufferSize, T>::Noise() const
{
    Index idx = GetOldestIdx();
    Index end = GetNewestIdx();

    T sum(0);
    Index count = 0;
    while (idx != end)
    {
        Index next = GetNextIdx(idx);
        T diff = GetData(next) - GetData(idx);
        if (diff < T(0))
        {
            diff = -diff;
        }
        sum += diff;
        ++count;
        idx = next;
    }
    return count == 0 ? 0 : sum / T(count);
}

template <unsigned short MaxBufferSize, typename T>
void TCircularBuffer<MaxBufferSize, T>::Resize(Index p_newSize)
{
    m_size = p_newSize > MaxBufferSize ? MaxBufferSize : p_newSize;
    for (Index i = 0; i < m_size; ++i)
    {
        m_data[i] = 0;
    }
    m_full = false;
    m_curIdx = 0;
}
}

#endif
