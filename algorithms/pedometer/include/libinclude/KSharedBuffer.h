#ifndef KSharedBuffer_h
#define KSharedBuffer_h
/*
============================================================================
Name        : KSharedBuffer.h
Author      : KEYnetik, Inc.
Version     :
Copyright   : Copyright 2010 by KEYnetik, Inc.
Description : declarations for SharedBuffer
============================================================================
*/

#include "KAccelerometerReading.h"

namespace Keynetik
{
/**
* Buffer utility class which allows the user to specify the length and
* storage location. Implemented as a circular buffer, where only the latest
* N values are stored at any given time (N is max buffer size).
* Provides a simple iterator class and supporting methods.
*/
class SharedBuffer
{
   public:
    /**
    * Datatype used as index into the buffer
    */
    typedef unsigned short Index;

    /**
    * Iterator for traversing values in the buffer
    */
    class Iterator
    {
       public:
        /**
        * Represents a non-existent index, used to denote end of traversal
        */
        static const unsigned int BAD_VAL = 999;

       public:
        /**
                 * Copy constructor
                 */
        Iterator(const Iterator &p_source) : current(p_source.current), buffer(p_source.buffer)
        {
        }

        /**
        * Checks validity of current iterator position
        * \return true if iterator is pointing to a valid location in the buffer,
        * false otherwise
        */
        bool IsGood() const
        {
            return current != BAD_VAL;
        }

        /**
        * Returns the value in the buffer to which the iterator is currently pointing.
        * It is assumed that this method will not be called on an invalid iterator, so
        * user must first call IsGood to validate the iterator.
        * \return AccelerometerReading at current iterator position
        */
        const AccelerometerReading &GetValue() const
        {
            return buffer.GetAt(*this);
        }

        /**
        * Moves the iterator to the next position in the buffer. This could move the iterator
        * into an invalid position, if the end of the buffer has been reached. It is important
        * to call IsGood after each call to Next to validate that the iterator has not reached
        * an invalid position.
        */
        void Next()
        {
            buffer.Next(*this);
        }

       private:
        Iterator &operator=(const Iterator &);
        Iterator();

       private:
        Iterator(const SharedBuffer &p_buf) : current(BAD_VAL), buffer(p_buf)
        {
        }
        Iterator(const SharedBuffer &p_buf, Index p_current) : current(p_current), buffer(p_buf)
        {
        }

       private:
        Index current;
        const SharedBuffer &buffer;

        friend class SharedBuffer;
    };
    /**
    * END iterator value to be compared agains when iterating to the end of the buffer
    */
    static const Iterator END;

   public:
    /**
    * Constructor requires the length of the buffer and a pointer to an array of AccelerometerReadings
    * to which values will be stored.
    * \param p_length number of readings in the buffer
    * \param p_readings non-0 pointer to an array of p_length AccelerometerReading's that is guaranteed to be persistent
    * for the lifespan of the created SharedBuffer instance
    */
    SharedBuffer(unsigned int p_length, AccelerometerReading *const p_readings);
    /**
    * Destructor. Does not destroy anything. AccelerometerReading's array is managed by client.
    */
    ~SharedBuffer();

    /**
    * Checks if the buffer is full given the number of reading the user is interested in.
    * \param p_length specifies minimum number of readings to check for, defaults to 0 which indicates to check for
    * entire buffer to be full
    * \return true if buffer contains at least p_length readings and p_length is greater than 0;
    * if p_length 0, returns true only if entire buffer is full; false if the number of readings in buffer is less than
    * p_length (even if buffer is actually full, but p_length is greater than maximum buffer size)
    */
    bool IsFull(Index p_length = 0) const
    {
        return ((p_length != 0) ? p_length : m_maxBufferSize) <= m_length;
    }
    /**
    * Returns the number of readings currently stored in the buffer
    * \return number of readings in buffer
    */
    Index GetLength() const
    {
        return m_length;
    }

    /**
    * Returns the maximum number of readings allowed in the buffer
    * \return number of readings allowed in buffer
    */
    Index GetSize() const
    {
        return m_maxBufferSize;
    }

    /**
    * Clears all values in buffer. No memory is deleted or allocated as a result of this call.
    */
    void Clear();

    /**
    * Returns an iterator pointing to the oldest reading in the buffer. The user can choose to iterate over
    * a subset of the latest buffer values in which case the parameter is used to specify how many readings
    * to traverse. If the user chooses to traverse K values in a buffer of size N, the iterator will begin
    * at position N-K
    * \param p_length number of latest readings to traverse, default 0 indicates entire buffer length
    * \return new iterator pointing to the oldest reading in the user specified subset of the buffer
    */
    Iterator Begin(Index p_length = 0) const;

    /**
    * Moves the iterator to the next position in the buffer. This could move the iterator
    * into an invalid position, if the end of the buffer has been reached. It is important
    * to call Iterator::IsGood after each call to Next to validate that the iterator has not reached
    * an invalid position.
    * \param p_iter reference to the iterator to increment
    */
    void Next(Iterator &p_iter) const;

    /**
          * Returns the value in the buffer to which the iterator is currently pointing.
          * It is assumed that this method will not be called on an invalid iterator, so
          * user must first call Iterator::IsGood to validate the iterator.
          * \param p_iter valid iterator pointing to position from which to retrieve a value
          * \return AccelerometerReading at current iterator position
          */
    const AccelerometerReading &GetAt(const Iterator &p_iter) const
    {
        return m_readings[p_iter.current];
    }

    /**
          * Inserts the AccelerometerReading into the end of the buffer. Buffer is circular, which means
          * if the buffer is full before this method is called, calling this method will result in
          * the oldest value in the buffer (front of buffer) being erased from the buffer and the new one
          * being added to the end.
          * \param p_reading new value to add to buffer
          * \return true if this is the first reading after an accelerometer ID reset. The client algorithms should
     * reset in this case.
          */
    bool PushBack(const AccelerometerReading &p_reading);

   private:
    SharedBuffer(const SharedBuffer &);
    SharedBuffer &operator=(const SharedBuffer &);

   private:
    AccelerometerReading *const m_readings;
    Index m_curIndex; // next index at which to insert
    Index m_length;
    Index m_maxBufferSize;
    unsigned int m_lastReadingID;
};

/**
* Subclass of SharedBuffer that maintains a buffer of size 20 readings.
*/
class DefaultSharedBuffer : public SharedBuffer
{
   public:
/**
* Standard buffer size
*/
#ifdef KEYNETIK_SHAREDBUFFER_SIZE
    static const unsigned int MaxBufferSize = KEYNETIK_SHAREDBUFFER_SIZE;
#else
    static const unsigned int MaxBufferSize = 100;
#endif
    /**
          * Constructor
          */
    DefaultSharedBuffer() : SharedBuffer(MaxBufferSize, m_readings)
    {
    }
    /**
    * Destructor
    */
    ~DefaultSharedBuffer()
    {
    }

   private:
    AccelerometerReading m_readings[MaxBufferSize];
};
}

#endif
