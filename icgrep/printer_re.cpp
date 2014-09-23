/*
 *  Copyright (c) 2014 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#include "printer_re.h"

//Regular Expressions
#include "re/re_re.h"
#include "re/re_alt.h"
#include "re/re_cc.h"
#include "re/re_name.h"
#include "re/re_end.h"
#include "re/re_rep.h"
#include "re/re_seq.h"
#include "re/re_start.h"


const std::string Printer_RE::PrintRE(const RE * re)
{
    std::string retVal = "";

    if (re == nullptr) {
        retVal = "--> RE NullPtr! <--";
    }
    else if (const Alt* re_alt = dynamic_cast<const Alt*>(re))
    {
        retVal += "(Alt[";
        bool comma = false;
        for (const RE * re : *re_alt) {
            if (comma) {
                retVal += ',';
            }
            retVal += PrintRE(re);
            comma = true;
        }
        retVal += "])";
    }
    else if (const CC* re_cc = dynamic_cast<const CC*>(re))
    {
        retVal = "CC \"";
        retVal += re_cc->getName();
        retVal += "\" ";

        for (const CharSetItem & item : *re_cc)
        {
            retVal += "[";
            retVal += std::to_string(item.lo_codepoint) + ",";
            retVal += std::to_string(item.hi_codepoint);
            retVal += "]";
        }
    }
    else if (const Name* re_name = dynamic_cast<const Name*>(re))
    {
        retVal = "Name \"";
        retVal += re_name->getName();
        retVal += "\" ";
    }
    else if (dynamic_cast<const End*>(re))
    {
        retVal = "End";
    }
    else if (const Rep* re_rep = dynamic_cast<const Rep*>(re))
    {
        retVal = "Rep(";
        retVal.append(PrintRE(re_rep->getRE()));
        retVal.append(",");
        retVal.append(std::to_string(re_rep->getLB()));
        retVal.append(",");
        if (re_rep->getUB() == Rep::UNBOUNDED_REP) {
            retVal.append("Unbounded");
        }
        else {
            retVal.append(std::to_string(re_rep->getUB()));            
        }
        retVal.append(")");
    }
    else if (const Seq* re_seq = dynamic_cast<const Seq*>(re))
    {
        retVal = "(Seq[";
        bool comma = false;
        for (const RE * re : *re_seq) {
            if (comma) {
                retVal.append(",");
            }
            retVal.append(PrintRE(re));
            comma = true;
        }
        retVal.append("])");
    }
    else if (dynamic_cast<const Start*>(re))
    {
        retVal = "Start";
    }
    else
    {
        retVal = "--> RE Unknown <--";
    }
    return std::move(retVal);
}
