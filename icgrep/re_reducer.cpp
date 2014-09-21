#include "re_reducer.h"


RE* RE_Reducer::reduce(RE* re, std::map<std::string, RE*>& re_map) {
    RE* retVal = nullptr;
    if (Alt* re_alt = dynamic_cast<Alt*>(re)) {
        Alt * new_alt = new Alt();
        for (RE * re : *re_alt) {
            new_alt->push_back(reduce(re, re_map));
        }
        retVal = new_alt;
    }
    else if (Seq* re_seq = dynamic_cast<Seq*>(re)) {
        Seq * new_seq = new Seq();
        for (RE * re : *re_seq) {
            new_seq->push_back(reduce(re, re_map));
        }
        if (re_seq->getType() == Seq::Byte) {
            //If this is a sequence of byte classes then this is a multibyte sequence for a Unicode character class.
            new_seq->setType(Seq::Byte);
            std::string seqname = new_seq->getName();
            re_map.insert(make_pair(seqname, new_seq));
            Name* name = new Name(seqname);
            name->setType(Name::Unicode);
            retVal = name;
        }
        else {
            retVal = new_seq;
        }
    }
    else if (Rep* re_rep = dynamic_cast<Rep*>(re)) {
        retVal = new Rep(reduce(re_rep->getRE(), re_map), re_rep->getLB(), re_rep->getUB());
    }
    else if (CC* re_cc = dynamic_cast<CC*>(re)) {
        std::string ccname = re_cc->getName();
        //If the character class isn't in the map then add it.
        re_map.insert(make_pair(ccname, re_cc));
        //return a new name class with the name of the character class.
        retVal = new Name(ccname);
    }
    else if (Name* re_name = dynamic_cast<Name*>(re)) {
        Name* name = new Name(re_name->getName());
        name->setType(re_name->getType());
        name->setNegated(re_name->isNegated());   // TODO:  Hide this in the re_name module.
        retVal = name;
    }
    else if (dynamic_cast<Start*>(re)) {
        retVal = new Start();
    }
    else if (dynamic_cast<End*>(re)) {
        retVal = new End();
    }
    return retVal;
}
