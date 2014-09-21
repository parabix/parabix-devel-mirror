/*
 *  Copyright (c) 2014 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#include "printer_re.h"


std::string Printer_RE::PrintRE(RE * re)
{
    std::string retVal = "";

    if (re == nullptr) {
        retVal = "--> RE NullPtr! <--";
    }
    else if (Alt* re_alt = dynamic_cast<Alt*>(re))
    {
        retVal += "(Alt[";
        for (RE * re : *re_alt) {
            retVal += PrintRE(re) + ",";
        }
        retVal = retVal.substr(0, retVal.size() - 1);
        retVal += "])";
    }
    else if (CC* re_cc = dynamic_cast<CC*>(re))
    {
        retVal += "CC \"";
        retVal += re_cc->getName();
        retVal += "\" ";

        for (const CharSetItem & item : re_cc->getItems())
        {
            retVal += "[";
            retVal += std::to_string(item.lo_codepoint) + ",";
            retVal += std::to_string(item.hi_codepoint);
            retVal += "]";
        }
    }
    else if (Name* re_name = dynamic_cast<Name*>(re))
    {
        retVal += "Name \"";
        retVal += re_name->getName();
        retVal += "\" ";
    }
    else if (dynamic_cast<End*>(re))
    {
        retVal += "End";
    }
    else if (Rep* re_rep = dynamic_cast<Rep*>(re))
    {
        retVal += "Rep("  + PrintRE(re_rep->getRE()) + "," + std::to_string(re_rep->getLB()) + ",";
        retVal += (re_rep->getUB() == UNBOUNDED_REP ? "Unbounded" : "UpperBound(" + std::to_string(re_rep->getUB()) + ")");
    }
    else if (Seq* re_seq = dynamic_cast<Seq*>(re))
    {
        retVal += "(Seq[";
        for (RE * re : *re_seq) {
            retVal += PrintRE(re) + ",";
        }
        retVal = retVal.substr(0, retVal.size() - 1);
        retVal += "])";
    }
    else if (dynamic_cast<Start*>(re))
    {
        retVal += "Start";
    }
    else
    {
        retVal += "--> RE Unknown <--";
    }

    return retVal;
}
