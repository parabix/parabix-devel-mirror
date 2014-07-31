#include "re_simplifier.h"


RE* RE_Simplifier::simplify(RE* re)
{
    RE* retVal = 0;

    if (Alt* re_alt = dynamic_cast<Alt*>(re))
    {
        std::list<RE*> re_list;
        std::list<RE*>::reverse_iterator rit = re_alt->GetREList()->rbegin();

        for (rit = re_alt->GetREList()->rbegin(); rit != re_alt->GetREList()->rend(); ++rit)
        {
            re_list.push_back(simplify(*rit));
        }

        retVal = mkAlt(&re_list);
    }
    else if (Seq* re_seq = dynamic_cast<Seq*>(re))
    {
        std::list<RE*> re_list;
        std::list<RE*>::iterator it;

        for (it = re_seq->GetREList()->begin(); it != re_seq->GetREList()->end(); ++it)
        {
            re_list.push_front(simplify(*it));
        }

        retVal = mkSeq(re_seq->getType(), &re_list);
    }
    else if (CC* re_cc = dynamic_cast<CC*>(re))
    {
        retVal = re_cc;
    }
    else if (Name* re_name = dynamic_cast<Name*>(re))
    {
        retVal = new Name(re_name->getName());
    }
    else if (Rep* re_rep = dynamic_cast<Rep*>(re))
    {
        retVal = mkRep(simplify(re_rep->getRE()), re_rep->getLB(), re_rep->getUB());
    }
    else if (Start* re_start = dynamic_cast<Start*>(re))
    {
        retVal = new Start();
    }
    else if (End* re_end = dynamic_cast<End*>(re))
    {
        retVal = new End();
    }

    return retVal;
}

RE* RE_Simplifier::mkSeq(Seq::Type type, std::list<RE*>* re_list)
{
    /*
      mkSeq - make a sequence, but flatten.  Result might not be a Seq. If
      there is only one component for a sequence, simply return that.
    */

    //We don't want to modify the AST that we are walking so we'll make a copy.
    std::list<RE*>* t1_list = new std::list<RE*>();
    //Linear in initial and final sizes.
    t1_list->assign(re_list->begin(), re_list->end());
    if (t1_list->size() > 0)
    {
        std::list<RE*>* t2_list = mkSeqList(t1_list);
        if (t2_list->size() > 1)
        {
            Seq* new_seq = new Seq(t2_list);
            new_seq->setType(type);
            return new_seq;
        }
        else
        {
            return t2_list->back();
        }
    }
    else
    {
        return 0;
    }
}

std::list<RE*>* RE_Simplifier::mkSeqList(std::list<RE*>* re_list)
{
    std::list<RE*>* ret_list = new std::list<RE*>();
    return mkSeqList_helper(ret_list, re_list);
}

std::list<RE*>* RE_Simplifier::mkSeqList_helper(std::list<RE*>* ret_list, std::list<RE*>* re_list)
{
    /*
      Build a list for Seq, flattening subsequences.
    */

    if (re_list->size() == 0)
    {
        return ret_list;
    }
    else if (Seq* seq = dynamic_cast<Seq*>(re_list->back()))
    {
        re_list->pop_back();
        seq->GetREList()->reverse();
        re_list->insert(re_list->end(), seq->GetREList()->begin(), seq->GetREList()->end());

        return mkSeqList_helper(ret_list, re_list);
    }
    else
    {
        ret_list->push_front(re_list->back());
        re_list->pop_back();
        return mkSeqList_helper(ret_list, re_list);
    }
}

RE* RE_Simplifier::mkAlt(std::list<RE*>* re_list)
{
    /*
      mkAlt - make a list of alternatives, but flatten, and combine character
      classes.  If there is only one alternative, simply return that.
    */

    std::list<RE*>* t1_list = new std::list<RE*>();
    t1_list->assign(re_list->begin(), re_list->end());
    if (t1_list->size() > 0)
    {
        std::list<RE*>* t2_list = mkAltList(t1_list);
        if (t2_list->size() > 1)
        {
            return new Alt(t2_list);
        }
        else
        {
            return t2_list->back();
        }
    }
    else
    {
        return 0;
    }
}

std::list<RE*>* RE_Simplifier::mkAltList(std::list<RE*>* re_list)
{
    std::list<RE*>* ret_list = new std::list<RE*>();
    return mkAltList_helper(ret_list, re_list);
}

std::list<RE*>* RE_Simplifier::mkAltList_helper(std::list<RE*>* ret_list, std::list<RE*>* re_list)
{
    /*
      Build a list for Alt, flattening alternative subgroups, and combining character classes.  We
      move character classes towards the end of the list to ensure that all combinations are found.
    */

    if (re_list->size() == 0)
    {
        return ret_list;
    }
    else if (Alt* alt = dynamic_cast<Alt*>(re_list->back()))
    {
        re_list->pop_back();
        re_list->insert(re_list->end(), alt->GetREList()->begin(), alt->GetREList()->end());
        return mkAltList_helper(ret_list, re_list);
    }
    else if (re_list->size() >= 2)
    {
        std::list<RE*>::iterator it;
        it = re_list->end();
        it--;
        if (CC* cc1 = dynamic_cast<CC*>(*it))
        {
            it--;
            if(CC* cc2 = dynamic_cast<CC*>(*it))
            {
                CC* cc = new CC(cc1, cc2);
                re_list->pop_back();
                re_list->pop_back();
                re_list->push_back(cc);
                return mkAltList_helper(ret_list, re_list);
            }
            else
            {
                std::list<RE*>::iterator item1 = re_list->end();
                --item1;
                std::list<RE*>::iterator item2 = item1;
                --item2;
                std::swap(*item1, *item2);
                return mkAltList_helper(ret_list, re_list);
            }
        }
        ret_list->push_front(re_list->back());
        re_list->pop_back();
        return mkAltList_helper(ret_list, re_list);
    }
    else
    {
        ret_list->push_front(re_list->back());
        re_list->pop_back();
        return mkAltList_helper(ret_list, re_list);
    }
}

int RE_Simplifier::ubCombine(int h1, int h2)
{
    if ((h1 == unboundedRep) || (h2 == unboundedRep))
    {
        return unboundedRep;
    }
    else
    {
        return h1 * h2;
    }
}

RE* RE_Simplifier::mkRep(RE* re, int lb2, int ub2)
{
    if (Rep* rep = dynamic_cast<Rep*>(re))
    {
        if (((rep->getUB() == unboundedRep) && (lb2 > 0)) ||
                ((rep->getUB() == unboundedRep) && (rep->getLB() <= 1)))
        {
            return new Rep(rep->getRE(), rep->getLB() * lb2, unboundedRep);
        }
        else if ((rep->getUB() == unboundedRep) && (lb2 == 0))
        {
            return new Rep(rep, 0, 1);
        }
        else if ((rep->getUB() * lb2) >= (rep->getLB() * (lb2 + 1) - 1))
        {
            return new Rep(rep->getRE(), rep->getLB() * lb2, ubCombine(rep->getUB(), ub2));
        }
        else
        {
            return new Rep(rep, lb2, ub2);
        }
    }
    else
    {
        if (Seq* seq = dynamic_cast<Seq*>(re))
        {
            if(seq->GetREList()->size() == 0)
            {
                return seq;
            }
        }

        if ((lb2 == 0) && (ub2 == 0))
        {
            return new Seq();
        }
        else if ((lb2 == 1) && (ub2 == 1))
        {
            return re;
        }
        else
        {
            return new Rep(re, lb2, ub2);
        }
    }
}
