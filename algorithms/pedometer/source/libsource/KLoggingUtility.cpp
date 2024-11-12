/*
============================================================================
Name        : KLoggingUtility.cpp
Author      : KEYnetik, Inc.
Version     :
Copyright   : Copyright 2008 by KEYnetik, Inc.
Description : definition of class LoggingUtility
============================================================================
*/

#include "KLoggingUtility.h"

using namespace Keynetik;

#ifndef KEYNETIK_EMBEDDED

void 
Keynetik::AddOwner(ColumnLogger* p_log, ColumnLogger::OwnerId p_owner, const char* p_name)
{
    if (p_log)
    {
        p_log->AddOwner(p_owner, p_name);
    }
}

void 
Keynetik::AddColumn(ColumnLogger* p_log, ColumnLogger::OwnerId p_owner, unsigned int p_colId, const char* p_header, bool p_shared)
{
    if (p_log)
    {
        p_log->AddColumn(p_owner, p_colId, p_header, p_shared);
    }
}

void 
Keynetik::AddXyzColumn(ColumnLogger* p_log, ColumnLogger::OwnerId p_owner, unsigned int p_colId, const char* p_header, bool p_shared)
{
    string h(p_header);
    AddColumn(p_log, p_owner, p_colId, (h+"X").c_str(), p_shared);
    AddColumn(p_log, p_owner, p_colId+1, (h+"Y").c_str(), p_shared);
    AddColumn(p_log, p_owner, p_colId+2, (h+"Z").c_str(), p_shared);
}

void 
Keynetik::Log(ColumnLogger* p_log, ColumnLogger::OwnerId p_owner, unsigned int p_colId, Fixed p_param) 
{ 
    Log(p_log, p_owner, p_colId, p_param.ToDouble());
}

void 
Keynetik::EndLine(ColumnLogger* p_log, ColumnLogger::OwnerId p_owner)
{
    if (p_log)
    {
        p_log->EndLine(p_owner);
    }
}

#endif
