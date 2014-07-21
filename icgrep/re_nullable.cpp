#include "re_nullable.h"

/*

 A regular expression is nullable if it (a) matches the empty
 string, and (b) applies everywhere.  Note that Start (^) and
 End ($) match the empty string, but not everywhere).

*/

RE* RE_Nullable::removeNullablePrefix(RE* re)
{
    RE* retVal = 0;

    if (Seq* re_seq = dynamic_cast<Seq*>(re))
    {
        std::list<RE*>* re_list = re_seq->GetREList();
        std::list<RE*>* t1_list = new std::list<RE*>();
        t1_list->assign(re_list->begin(), re_list->end());

        return new Seq(removeNullableSeqPrefix(t1_list));
    }
    else if (Alt* re_alt = dynamic_cast<Alt*>(re))
    {
        Alt* new_alt = new Alt();
        std::list<RE*>::iterator it;

        for (it = re_alt->GetREList()->begin(); it != re_alt->GetREList()->end(); ++it)
        {
            new_alt->AddREListItem(removeNullablePrefix(*it));
        }

        return new_alt;
    }
    else if (Rep* re_rep = dynamic_cast<Rep*>(re))
    {
        if ((re_rep->getLB() == 0) || (isNullable(re_rep->getRE())))
        {
            return new Seq();
        }
        else if (hasNullablePrefix(re_rep->getRE()))
        {
            //std::cout << "removeNullablePrefix->Rep->hasNullablePrefix:" << std::endl;
            std::list<RE*>* seq_lst = new std::list<RE*>();
            seq_lst->push_back(new Rep(re_rep->getRE(), re_rep->getLB()-1, re_rep->getLB()-1));
            seq_lst->push_back(removeNullablePrefix(re_rep->getRE()));

            return RE_Simplifier::mkSeq(seq_lst);
        }
        else
        {
            return RE_Simplifier::mkRep(re_rep->getRE(), re_rep->getLB(), re_rep->getLB());
        }
    }
    else
    {
        return re;
    }
}

std::list<RE*>* RE_Nullable::removeNullableSeqPrefix(std::list<RE*>* re_list)
{
    if (re_list->size() == 0)
    {
        return re_list;
    }
    else if(isNullable(re_list->front()))
    {
        re_list->pop_front();

        return removeNullableSeqPrefix(re_list);
    }
    else
    {
        RE* re = re_list->front();
        re_list->pop_front();
        re_list->push_front(removeNullablePrefix(re));
        re_list->reverse();

        return re_list;
    }
}

RE* RE_Nullable::removeNullableSuffix(RE* re)
{
    if (Seq* re_seq = dynamic_cast<Seq*>(re))
    {
        std::list<RE*>* re_list = re_seq->GetREList();
        std::list<RE*>* t1_list = new std::list<RE*>();
        t1_list->assign(re_list->begin(), re_list->end());

        return new Seq(removeNullableSeqSuffix(t1_list));
    }
    else if (Alt* re_alt = dynamic_cast<Alt*>(re))
    {
        Alt* new_alt = new Alt();
        std::list<RE*>::iterator it;

        for (it = re_alt->GetREList()->begin(); it != re_alt->GetREList()->end(); ++it)
        {
            new_alt->AddREListItem(removeNullableSuffix(*it));
        }

        return new_alt;
    }
    else if (Rep* re_rep = dynamic_cast<Rep*>(re))
    {
        if ((re_rep->getLB() == 0) || (isNullable(re_rep->getRE())))
        {
            //std::cout << "removeNullableSuffix->Rep->lb=0 or is nullable:" << std::endl;
            return new Seq();
        }
        else if (hasNullableSuffix(re_rep->getRE()))
        {
            //std::cout << "removeNullableSuffix->Rep->hasNullableSuffix:" << std::endl;
            std::list<RE*>* seq_lst = new std::list<RE*>();
            seq_lst->push_back(removeNullableSuffix(re_rep->getRE()));
            seq_lst->push_back(new Rep(re_rep->getRE(), re_rep->getLB()-1, re_rep->getLB()-1));

            return RE_Simplifier::mkSeq(seq_lst);
        }
        else
        {
            return RE_Simplifier::mkRep(re_rep->getRE(), re_rep->getLB(), re_rep->getLB());
        }
    }
    else
    {
        return re;
    }
}

std::list<RE*>* RE_Nullable::removeNullableSeqSuffix(std::list<RE*>* re_list)
{
    if (re_list->size() == 0)
    {
        return re_list;
    }

    RE* seq_re = re_list->front();
    re_list->pop_front();

    if (isNullableSeq(re_list))
    {

        std::list<RE*>* t1_list = new std::list<RE*>();
        t1_list->push_back(removeNullableSuffix(seq_re));

        return t1_list;
    }

    std::list<RE*>* t2_list;
    t2_list = removeNullableSeqSuffix(re_list);
    t2_list->push_back(seq_re);

    return t2_list;
}

bool RE_Nullable::isNullable(RE* re)
{
    if (CC* re_cc = dynamic_cast<CC*>(re))
    {
        return false;
    }
    else if (Start* re_start = dynamic_cast<Start*>(re))
    {
        return false;
    }
    else if (End* re_end = dynamic_cast<End*>(re))
    {
        return false;
    }
    else if (Seq* re_seq = dynamic_cast<Seq*>(re))
    {
        return isNullableSeq(re_seq->GetREList());
    }
    else if (Alt* re_alt = dynamic_cast<Alt*>(re))
    {
        return isNullableAlt(re_alt->GetREList());
    }
    else if (Rep* re_rep = dynamic_cast<Rep*>(re))
    {   
        return re_rep->getLB() == 0 ? true : isNullable(re_rep->getRE());
    }
}

bool RE_Nullable::isNullableSeq(std::list<RE*>* re_list)
{
    std::list<RE*>::iterator it = re_list->begin();
    return isNullableSeq_helper(re_list, it);
}

bool RE_Nullable::isNullableSeq_helper(std::list<RE*>* re_list, std::list<RE*>::iterator it)
{
    if (it != re_list->end())
    {
        return isNullable(*it) ? isNullableSeq_helper(re_list, ++it) : false;
    }
    else
    {
        return true;
    }
}

bool RE_Nullable::isNullableAlt(std::list<RE*>* re_list)
{
    std::list<RE*>::iterator it = re_list->begin();
    return isNullableAlt_helper(re_list, it);
}

bool RE_Nullable::isNullableAlt_helper(std::list<RE*>* re_list, std::list<RE*>::iterator it)
{
    if (it != re_list->end())
    {
        return isNullable(*it) ? true : isNullableAlt_helper(re_list, ++it);
    }
    else
    {
        return false;
    }
}

bool RE_Nullable::hasNullablePrefix(RE* re)
{
    if (Seq* seq = dynamic_cast<Seq*>(re))
    {
        if (isNullable(seq->GetREList()->back()))
        {
            return true;
        }
        else
        {
            return hasNullablePrefix(seq->GetREList()->back());
        }
    }
    else if (Alt* alt = dynamic_cast<Alt*>(re))
    {
        if (alt->GetREList()->size() == 0)
        {
            return false;
        }
        else
        {
            if (hasNullablePrefix(alt->GetREList()->back()))
            {
                return true;
            }
            else
            {
                std::list<RE*> alt_list;
                std::list<RE*>::iterator it;
                it = alt->GetREList()->end();
                it--;
                alt_list.insert(alt_list.end(), alt->GetREList()->begin(), it);
                alt_list.reverse();

                return hasNullablePrefix(new Alt(alt_list));
            }
        }
    }
    else if (Rep* rep = dynamic_cast<Rep*>(re))
    {
        return hasNullablePrefix(rep->getRE());
    }
    else
    {
        return false;
    }
}

bool RE_Nullable::hasNullableSuffix(RE* re)
{
    if (Seq* seq = dynamic_cast<Seq*>(re))
    {
        if (seq->GetREList()->size() == 1)
        {
            if (isNullable(seq->GetREList()->back()))
            {
                return true;
            }
            else
            {
                return hasNullableSuffix(seq->GetREList()->front());
            }
        }
        else
        {
            std::list<RE*> seq_list;
            std::list<RE*>::iterator it;
            it = seq->GetREList()->begin();
            it++;
            seq_list.insert(seq_list.end(), it, seq->GetREList()->end());
            seq_list.reverse();

            return hasNullableSuffix(new Seq(seq_list));
        }
    }
    else if (Alt* alt = dynamic_cast<Alt*>(re))
    {
        if (alt->GetREList()->size() == 0)
        {
            return false;
        }
        else
        {
            if (hasNullableSuffix(alt->GetREList()->front()))
            {
                return true;
            }
            else
            {
                std::list<RE*> alt_list;
                std::list<RE*>::iterator it;
                it = alt->GetREList()->begin();
                it++;
                alt_list.insert(alt_list.begin(), it, alt->GetREList()->end());
                alt_list.reverse();

                return hasNullableSuffix(new Alt(alt_list));
            }
        }
    }
    else if (Rep* rep = dynamic_cast<Rep*>(re))
    {
        return hasNullableSuffix(rep->getRE());
    }
    else
    {
        return false;
    }
}



