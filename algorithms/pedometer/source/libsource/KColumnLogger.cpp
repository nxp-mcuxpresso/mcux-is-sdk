/*
============================================================================
 Name        : KColumnLogger.cpp
 Author      : KEYnetik, Inc.
 Version     :
 Copyright   : Copyright 2008 by KEYnetik, Inc.
 Description : ColumnLogger definition
============================================================================
*/

#include "KColumnLogger.h"

#include <vector>
#include <string>
#include <map>

//using namespace std;
using namespace Keynetik;

ColumnLogger::ColumnLogger() 
:   m_headersChanged(false),
    m_primarySet(false),
    m_primaryOwner(0)
{
}

ColumnLogger::~ColumnLogger() 
{
}

void 
ColumnLogger::SetPrimary(OwnerId p_owner) // the last one to call this wins. If not called, the last one to call AddColumn is the primary owner
{
    m_primaryOwner=p_owner;
    m_primarySet=true;
}

void 
ColumnLogger::AddColumn(OwnerId p_owner, unsigned int p_colId, const char* p_header, bool p_shared)
{
    RegularColumn rc;
    rc.ownerId  =p_owner;
    rc.id       =p_colId;
    rc.title    =p_header;
    int sharedIdx = FindShared(p_header);
    if (p_shared && sharedIdx == -1) // set up a new shared column
    {
        SharedColumn sc;
        sc.title=p_header;
        rc.shared=m_shared.size();
        // convert all regular columns with the same name into shared
        for (vector<RegularColumn>::iterator it=m_regular.begin(); it != m_regular.end(); ++it)
        {
            if (it->title == p_header)
            {
                it->shared=rc.shared;
            }
        }
        m_shared.push_back(sc);
    }
    else
    {
        rc.shared=sharedIdx;
    }
    m_regular.push_back(rc);

    m_headersChanged=true;
    if (!m_primarySet)
    {
        m_primaryOwner=p_owner;
    }
}

int 
ColumnLogger::FindShared(const char* p_header) const
{
    for (vector<SharedColumn>::const_iterator it=m_shared.begin(); it != m_shared.end(); ++it)
    {
        if (it->title == p_header)
        {
            return it - m_shared.begin();
        }
    }
    return -1;
}

void 
ColumnLogger::OutputHeaders() 
{
    if (m_headersChanged)
    {
        for (vector<SharedColumn>::const_iterator it=m_shared.begin(); it != m_shared.end(); ++it)
        {
            InternalLog(it->title.c_str());
        }
        for (vector<RegularColumn>::const_iterator it=m_regular.begin(); it != m_regular.end(); ++it)
        {
            if (it->shared == -1)
            {
                map<OwnerId, string>::const_iterator owner=m_owners.find(it->ownerId);
                if (owner != m_owners.end() && m_owners.size() > 1)
                {
                    InternalLog((it->title+"_"+owner->second).c_str());
                }
                else
                {
                    InternalLog(it->title.c_str());
                }
            }
        }
        InternalLog("\n");
        m_headersChanged=false;
    }
} 

void 
ColumnLogger::EndLine(ColumnLogger::OwnerId p_owner) 
{
    if (p_owner == m_primaryOwner)
    {
        if (m_headersChanged)
        {
            OutputHeaders();
        }
        for (vector<SharedColumn>::const_iterator it=m_shared.begin(); it != m_shared.end(); ++it)
        {
            InternalLog(it->value.c_str());
//TESTME            it->value.clear();
        }
        for (vector<RegularColumn>::iterator it=m_regular.begin(); it != m_regular.end(); ++it)
        {
            if (it->shared == -1)
            {
                InternalLog(it->value.c_str());
                it->value.clear();
            }
        }
        for (vector<string>::iterator it=m_extras.begin(); it != m_extras.end(); ++it)
        {
            InternalLog(it->c_str());
        }
        m_extras.clear();
        InternalLog("\n");
    }
}

void 
ColumnLogger::AddValue(ColumnLogger::OwnerId p_owner, unsigned int p_colId, const string p_str)
{
    for (vector<RegularColumn>::iterator it=m_regular.begin(); it != m_regular.end(); ++it)
    {
        if (it->ownerId == p_owner && it->id == p_colId)
        {
            if (it->shared == -1)
            {
                it->value=p_str;
            }
            else
            {
                m_shared[it->shared].value=p_str;
            }
            break;
        }
    }
}

void 
ColumnLogger::AddExtra(const string p_str)
{
    m_extras.push_back(p_str);
}

void 
ColumnLogger::AddOwner(OwnerId p_id, const char* p_name)
{
    m_owners[p_id]=p_name;
}
