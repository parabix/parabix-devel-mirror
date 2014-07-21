/*
 *  Copyright (c) 2014 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#include "re_seq.h"

Seq::Seq()
{
    mList = new std::list<RE*>();
}

Seq::Seq(std::list<RE*>* lst)
{
    mList = new std::list<RE*>();
    std::list<RE*>::iterator it;
    it=lst->begin();
    mList->assign(it, lst->end());
    mList->reverse();
}

Seq::~Seq()
{
    while(!mList->empty()) delete mList->back(), mList->pop_back();
    delete mList;
}

std::list<RE*>* Seq::GetREList()
{
    return mList;
}

void Seq::AddREListItem(RE *re)
{
    mList->push_back(re);
}

