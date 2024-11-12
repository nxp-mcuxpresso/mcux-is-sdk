#ifndef KCircularBufferConstIterator_h
#define KCircularBufferConstIterator_h
/*
============================================================================
Name        : KCircularBufferConstIterator.h
Author      : KEYnetik, Inc.
Version     :
Copyright   : Copyright 2011 by KEYnetik, Inc.
Description : declarations for TCircularBufferConstIterator
============================================================================
*/

#include "KTypes.h"
#include "KCircularBuffer.h"

namespace Keynetik
{
class CircularBufferConstIteratorImpl
{
   protected:
    typedef unsigned short Index;

    CircularBufferConstIteratorImpl(Index p_maxSize, const CircularBufferImpl &p_buffer, int p_offset)
        : m_buffer(p_buffer), current(m_buffer.GetOldestIdx()), maxSize(p_maxSize)
    {
        if (p_buffer.IsEmpty())
        { // invalidate
            current = maxSize;
        }
        else if (p_offset > 0)
        {
            while (p_offset > 0)
            {
                Next();
                --p_offset;
            }
        }
        else if (p_offset < 0)
        {
            current = m_buffer.GetNewestIdx();
            ++p_offset; // -1 corresponds to the newest element
            while (p_offset < 0)
            {
                Prev();
                ++p_offset;
            }
        }
    }
    CircularBufferConstIteratorImpl(const CircularBufferConstIteratorImpl &p_source)
        : m_buffer(p_source.m_buffer), current(p_source.current), maxSize(p_source.maxSize)
    {
    }
    /**
    * Moves the iterator to the next position in the buffer. This could move the iterator
    * into an invalid position, if the end of the buffer has been reached. It is important
    * to call IsGood after each call to Next to validate that the iterator has not reached
    * an invalid position.
    */
    bool Next()
    {
        if (current == m_buffer.GetNewestIdx())
        { // invalidate
            current = maxSize;
            return false;
        }
        else
        {
            current = m_buffer.GetNextIdx(current);
            return true;
        }
    }
    /**
    * Moves the iterator to the previous position in the buffer. This could move the iterator
    * into an invalid position, if the beginning of the buffer has been reached. It is important
    * to call IsGood after each call to Prev to validate that the iterator has not reached
    * an invalid position.
    */
    bool Prev()
    {
        if (current == m_buffer.GetOldestIdx())
        { // invalidate
            current = maxSize;
            return false;
        }
        else
        {
            current = m_buffer.GetPrevIdx(current);
            return true;
        }
    }

    const CircularBufferImpl &m_buffer;
    Index current;
    Index maxSize;

   private:
    CircularBufferConstIteratorImpl &operator=(const CircularBufferConstIteratorImpl &);
};
/**
* Iterator for traversing values in the buffer
*/
template <unsigned short MaxBufferSize, typename T>
class TCircularBufferConstIterator : private CircularBufferConstIteratorImpl
{
   public:
    typedef TCircularBuffer<MaxBufferSize, T> Buffer;
    typedef CircularBufferConstIteratorImpl::Index Index;

   public:
    TCircularBufferConstIterator(const Buffer &p_buf) : CircularBufferConstIteratorImpl(MaxBufferSize, p_buf, 0)
    {
    }
    TCircularBufferConstIterator(const Buffer &p_buf, int p_offset)
        : CircularBufferConstIteratorImpl(MaxBufferSize, p_buf, p_offset)
    {
    }
    /**
    * Copy constructor
    */
    TCircularBufferConstIterator(const TCircularBufferConstIterator &p_source)
        : CircularBufferConstIteratorImpl(p_source)
    {
    }

    const Buffer &GetBuffer() const
    {
        return (const Buffer &)m_buffer;
    }

    /**
    * Checks validity of current iterator position
    * \return true if iterator is pointing to a valid location in the buffer,
    * false otherwise
    */
    bool IsGood() const
    {
        return current != MaxBufferSize;
    }

    /**
    * Returns the value in the buffer to which the iterator is currently pointing.
    * It is assumed that this method will not be called on an invalid iterator, so
    * user must first call IsGood to validate the iterator.
    * \return data value at current iterator position
    */
    T GetValue() const
    {
        return GetBuffer().GetData(current);
    }

    Index GetIndex()
    {
        return current;
    }

    /**
    * Moves the iterator to the next position in the buffer. This could move the iterator
    * into an invalid position, if the end of the buffer has been reached. It is important
    * to call IsGood after each call to Next to validate that the iterator has not reached
    * an invalid position.
    */
    bool Next()
    {
        return CircularBufferConstIteratorImpl::Next();
    }
    /**
    * Moves the iterator to the previous position in the buffer. This could move the iterator
    * into an invalid position, if the beginning of the buffer has been reached. It is important
    * to call IsGood after each call to Prev to validate that the iterator has not reached
    * an invalid position.
    */
    bool Prev()
    {
        return CircularBufferConstIteratorImpl::Prev();
    }

   private:
    TCircularBufferConstIterator &operator=(const TCircularBufferConstIterator &);
    TCircularBufferConstIterator();
};

#define KEYNETIK_CONST_ITER_OLDEST(T, buf, it) TCircularBufferConstIterator<T::MaxBufferSize, T::ElementType> it(buf)
#define KEYNETIK_CONST_ITER_NEWEST(T, buf, it) \
    TCircularBufferConstIterator<T::MaxBufferSize, T::ElementType> it(buf, -1)
#define KEYNETIK_CONST_ITER(T, buf, it, idx) TCircularBufferConstIterator<T::MaxBufferSize, T::ElementType> it(buf, idx)
}

#endif // KCircularBufferConstIterator_h
