/*
 *  Copyright (c) 2014 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#include "re_alt.h"

Alt::Alt()
{
    mList = new std::list<RE*>();
}

Alt::Alt(std::list<RE*>* lst)
{
    mList = new std::list<RE*>();
    std::list<RE*>::iterator it;
    it=lst->begin();
    mList->assign(it, lst->end());
    std::reverse(mList->begin(), mList->end());
}

Alt::Alt(std::list<RE*> lst)
{
    mList = new std::list<RE*>();
    std::list<RE*>::iterator it;
    it=lst.begin();
    mList->assign(it, lst.end());
    std::reverse(mList->begin(), mList->end());
}

Alt::~Alt()
{
    while(!mList->empty()) delete mList->back(), mList->pop_back();
    delete mList;
}

std::list<RE*>* Alt::GetREList()
{
    return mList;
}

void Alt::AddREListItem(RE* re)
{
    mList->push_back(re);
}


