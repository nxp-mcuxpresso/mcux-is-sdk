#ifndef KColumnLogger_H
#define KColumnLogger_H
/*
============================================================================
Name        : KLogger.h
Author      : KEYnetik, Inc.
Version     :
Copyright   : Copyright 2011 by KEYnetik, Inc.
Description : ColumnLogger declaration
============================================================================
*/

#include <vector>
#include <string>
#include <map>
#include <sstream>

namespace Keynetik
{
// using namespace std;
class ColumnLogger
{
   public:
    typedef const void *OwnerId;

   public:
    ColumnLogger();
    virtual ~ColumnLogger();

    void AddOwner(OwnerId, const char *p_name);
    void AddColumn(OwnerId, unsigned int p_colId, const char *p_header, bool p_shared = false);

    /**
    * Sends a value to the log. A call immediately following AddColumn() outputs headers
    * \tparam p_param the value to log
    * \tparam p_endLine true if an end of line is to be sent after the value
    */
    template <typename T>
    void Log(OwnerId p_owner, unsigned int p_colId, T p_param)
    {
        if (m_headersChanged)
        {
            OutputHeaders();
        }
        ostringstream oss(ostringstream::out);
        oss << p_param;
        AddValue(p_owner, p_colId, oss.str());
    }
    /**
    * End the current line in the log. A call immediately following AddColumn() outputs headers
    */
    virtual void EndLine(OwnerId p_owner);

    void SetPrimary(OwnerId p_owner); // the last one to call this wins. If not called, the last one to call AddColumn
                                      // is the primary owner

    template <typename T>
    void Log(T p_param) // extra column to the right
    {
        if (m_headersChanged)
        {
            OutputHeaders();
        }
        ostringstream oss(ostringstream::out);
        oss << p_param;
        AddExtra(oss.str());
    }

   protected:
    /**
    * Sends a character string to the log.
    */
    virtual void InternalLog(const char *) = 0;
    virtual void OutputHeaders();
    // void AddValue(OwnerId p_owner, unsigned int p_colId, const std::string p_str);
    void AddValue(OwnerId p_owner, unsigned int p_colId, const string p_str);
    // void AddExtra(const std::string p_str);
    void AddExtra(const string p_str);

    int FindShared(const char *) const;

   private:
    struct SharedColumn
    {
        string title;
        string value;
    };
    struct RegularColumn
    {
        OwnerId ownerId;
        unsigned int id;
        string title;
        int shared; // -1 if not shared
        string value;
    };

    map<OwnerId, string> m_owners;
    vector<SharedColumn> m_shared;
    vector<RegularColumn> m_regular;
    vector<string> m_extras;

    bool m_headersChanged;
    bool m_primarySet;
    OwnerId m_primaryOwner;
};
}

#endif
