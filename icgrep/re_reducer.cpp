#include "re_reducer.h"


RE* RE_Reducer::reduce(RE* re, std::map<std::string, RE*>& re_map)
{
    RE* retVal = 0;

    if (Alt* re_alt = dynamic_cast<Alt*>(re))
    {
        std::list<RE*> re_list;
        std::list<RE*>::reverse_iterator rit = re_alt->GetREList()->rbegin();

        for (rit = re_alt->GetREList()->rbegin(); rit != re_alt->GetREList()->rend(); ++rit)
        {
            re_list.push_back(reduce(*rit, re_map));
        }

        retVal = new Alt(&re_list);
    }
    else if (Seq* re_seq = dynamic_cast<Seq*>(re))
    {
/*
        if (re_seq->getType() == Seq::Byte)
        {
            //If this is a sequence of byte classes then this is a multibyte sequence for a Unicode character class.
            std::string seqname = re_seq->getName();
            re_map.insert(make_pair(seqname, re_seq));
            retVal = new Name(seqname);
        }
        else
        {
*/
            std::list<RE*> re_list;
            std::list<RE*>::iterator it;

            for (it = re_seq->GetREList()->begin(); it != re_seq->GetREList()->end(); ++it)
            {
                re_list.push_front(reduce(*it, re_map));
            }

            retVal = new Seq(&re_list);
//        }
    }
    else if (Rep* re_rep = dynamic_cast<Rep*>(re))
    {
        retVal = new Rep(reduce(re_rep->getRE(), re_map), re_rep->getLB(), re_rep->getUB());
    }
    else if (CC* re_cc = dynamic_cast<CC*>(re))
    {
        std::string ccname = re_cc->getName();
        //If the character class isn't in the map then add it.
        re_map.insert(make_pair(ccname, re_cc));
        //return a new name class with the name of the character class.
        retVal = new Name(ccname);
    }
    else if (Name* re_name = dynamic_cast<Name*>(re))
    {
        retVal = new Name(re_name->getName());
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
