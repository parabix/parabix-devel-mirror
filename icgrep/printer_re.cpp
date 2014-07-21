/*
 *  Copyright (c) 2014 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#include "printer_re.h"


std::string Printer_RE::PrintRE(RE* re)
{
    std::string retVal = "";

    if (Alt* re_alt = dynamic_cast<Alt*>(re))
    {
        retVal += "(Alt[";

        std::list<RE*>::iterator it;
        for (it = re_alt->GetREList()->begin(); it != re_alt->GetREList()->end(); ++it)
        {
            retVal += PrintRE(*it) + ",";
        }
        retVal = retVal.substr(0, retVal.size() - 1);
        retVal += "])";
    }
    else if (CC* re_cc = dynamic_cast<CC*>(re))
    {
        retVal += "CC \"";
        retVal += re_cc->getName();
        retVal += "\" ";

        std::vector<CharSetItem> items = re_cc->getItems();
        std::vector<CharSetItem>::iterator it;
        for (it = items.begin(); it != items.end(); ++it)
        {
            retVal += "[";
            retVal += INT2STRING(it->lo_codepoint) + ",";
            retVal += INT2STRING(it->hi_codepoint);
            retVal += "]";
        }

        //std::string member = (re_cc->is_member(47) ? "True" : "False");
        //retVal += " is codepoint 47 a member: " + member;

/*
        retVal += "CC \"";
        retVal += re_cc->getName();
        retVal += "\" ";

        retVal += " Removed: 100 ";
        re_cc->remove1(100);
        //re_cc->remove_range(40,50);

        std::vector<CharSetItem> r_items = re_cc->getItems();
        std::vector<CharSetItem>::iterator r_it;
        for (r_it = r_items.begin(); r_it != r_items.end(); ++r_it)
        {
            retVal += "[";
            retVal += INT2STRING(r_it->lo_codepoint) + ",";
            retVal += INT2STRING(r_it->hi_codepoint);
            retVal += "]";
        }
*/
    }
    else if (Name* re_name = dynamic_cast<Name*>(re))
    {
        retVal += "Name \"";
        retVal += re_name->getName();
        retVal += "\" ";
    }
    else if (End* re_end = dynamic_cast<End*>(re))
    {
        retVal += "End";
    }
    else if (Rep* re_rep = dynamic_cast<Rep*>(re))
    {
        retVal += "Rep("  + PrintRE(re_rep->getRE()) + "," + INT2STRING(re_rep->getLB()) + ",";
        retVal += (re_rep->getUB() == unboundedRep ? "Unbounded" : "UpperBound(" + INT2STRING(re_rep->getUB()) + ")");
    }
    else if (Seq* re_seq = dynamic_cast<Seq*>(re))
    {
        retVal += "(Seq[";
        std::list<RE*>::iterator it;
        for (it = re_seq->GetREList()->begin(); it != re_seq->GetREList()->end(); ++it)
        {
            retVal += PrintRE(*it) + ",";
        }
        retVal = retVal.substr(0, retVal.size() - 1);
        retVal += "])";
    }
    else if (Start* re_start = dynamic_cast<Start*>(re))
    {
        retVal += "Start";
    }
    else
    {
        retVal += "--> RE Unknown <--";
    }

    return retVal;
}
