/*
 *  Copyright (c) 2014 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#include "re_seq.h"

Seq::Seq()
{
    mList = new std::list<RE*>();
    mType = Seq::Normal;
}

Seq::Seq(std::list<RE*>* lst)
{
    mList = new std::list<RE*>();
    std::list<RE*>::iterator it;
    it=lst->begin();
    mList->assign(it, lst->end());
    mList->reverse();
    mType = Seq::Normal;
}

Seq::Seq(std::list<RE*> lst)
{
    mList = new std::list<RE*>();
    std::list<RE*>::iterator it;
    it=lst.begin();
    mList->assign(it, lst.end());
    mList->reverse();
    mType = Seq::Normal;
}

Seq::~Seq()
{
    while(!mList->empty()) delete mList->back(), mList->pop_back();
    delete mList;
}

std::string Seq::getName()
{
    if (mType == Seq::Byte)
    {
        std::string name = "Seq";

        std::list<RE*> re_list;
        std::list<RE*>::iterator it = mList->begin();

        for (it = mList->begin(); it != mList->end(); ++it)
        {
            if (CC* seq_cc = dynamic_cast<CC*>(*it))
            {
                name += seq_cc->getName();
            }
            else if (Name* seq_name = dynamic_cast<Name*>(*it))
            {
                name += seq_name->getName();
            }
            else
            {
                return "Bad Byte Sequence!";
            }
        }

        return name;
    }
    else
    {
        return "Unnamed Sequence";
    }
}

std::list<RE*>* Seq::GetREList()
{
    return mList;
}

void Seq::AddREListItem(RE *re)
{
    mList->push_back(re);
}

Seq::Type Seq::getType()
{
    return mType;
}

void Seq::setType(Seq::Type type)
{
    mType = type;
}
