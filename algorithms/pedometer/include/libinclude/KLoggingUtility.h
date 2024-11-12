#ifndef KLoggingUtility_H
#define KLoggingUtility_H
/*
============================================================================
Name        : KLoggingUtility.h
Author      : KEYnetik, Inc.
Version     :
Copyright   : Copyright 2008 by KEYnetik, Inc.
Description : declaration of class LoggingUtility
============================================================================
*/

#include "KFixed.h"
#include "KXYZ.h"

#ifndef KEYNETIK_EMBEDDED
#include "KColumnLogger.h"
#endif

namespace Keynetik
{
const int XyzLogColumnCount = 3;

#ifndef KEYNETIK_EMBEDDED
extern void AddOwner(ColumnLogger *p_log, ColumnLogger::OwnerId p_owner, const char *p_name);
extern void AddColumn(ColumnLogger *p_log,
                      ColumnLogger::OwnerId p_owner,
                      unsigned int p_colId,
                      const char *p_header,
                      bool p_shared = false);
extern void AddXyzColumn(ColumnLogger *p_log,
                         ColumnLogger::OwnerId p_owner,
                         unsigned int p_colId,
                         const char *p_header,
                         bool p_shared = false);

extern void Log(ColumnLogger *p_log, ColumnLogger::OwnerId p_owner, unsigned int p_colId, Fixed p_param);
template <typename T>
inline void Log(ColumnLogger *p_log, ColumnLogger::OwnerId p_owner, unsigned int p_colId, T p_param)
{
    if (p_log)
    {
        p_log->Log(p_owner, p_colId, p_param);
    }
}
inline void Log(ColumnLogger *p_log, ColumnLogger::OwnerId p_owner, unsigned int p_colId, const XYZ &p_param)
{
    Log(p_log, p_owner, p_colId, p_param.X());
    Log(p_log, p_owner, p_colId + 1, p_param.Y());
    Log(p_log, p_owner, p_colId + 2, p_param.Z());
}
template <typename T>
inline void Log(ColumnLogger *p_log, T p_param)
{
    if (p_log)
    {
        p_log->Log(p_param);
    }
}
inline void Log(ColumnLogger *p_log, const XYZ &p_param)
{
    if (p_log)
    {
        p_log->Log(p_param.X().ToDouble());
        p_log->Log(p_param.Y().ToDouble());
        p_log->Log(p_param.Z().ToDouble());
    }
}

extern void EndLine(ColumnLogger *p_log, ColumnLogger::OwnerId p_owner);

#define LOG_ADD_COLUMN(col) Keynetik::AddColumn(m_log, Keynetik::ColumnLogger::OwnerId(this), col, #col)
#define LOG_ADD_XYZ_COLUMN(col) Keynetik::AddXyzColumn(m_log, Keynetik::ColumnLogger::OwnerId(this), col, #col)
#define LOG_ADD_SHARED_COLUMN(col) Keynetik::AddColumn(m_log, Keynetik::ColumnLogger::OwnerId(this), col, #col, true)
#define LOG_ADD_OWNER(name) Keynetik::AddOwner(m_log, Keynetik::ColumnLogger::OwnerId(this), name)
#define LOG(col, v) Keynetik::Log(m_log, Keynetik::ColumnLogger::OwnerId(this), col, v)
#define LOG_EXTRA(v) Keynetik::Log(m_log, v)
#define LOG_EXTRA_XYZ(v) Keynetik::Log(m_log, v)
#define LOG_ENDLINE() Keynetik::EndLine(m_log, Keynetik::ColumnLogger::OwnerId(this))

#else
#define LOG_ADD_COLUMN(col)
#define LOG_ADD_SHARED_COLUMN(col)
#define LOG_ADD_XYZ_COLUMN(col)
#define LOG_ADD_OWNER(name)
#define LOG(col, v)
#define LOG_EXTRA(v)
#define LOG_EXTRA_XYZ(v)
#define LOG_ENDLINE()

#endif
}

#endif
