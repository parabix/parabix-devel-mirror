/*
 *  Copyright (c) 2014 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#include "re_seq.h"


Seq::Seq()
: mType(Seq::Normal)
{

}

Seq::Seq(const Type type)
: mType(type)
{

}

Seq::Seq(const Type type, iterator begin, iterator end)
: std::vector<RE*>(begin, end)
, mType(type)
{

}

Seq::~Seq() {
    for (RE * re : *this) {
        delete re;
    }
}

std::string Seq::getName() const {
    if (mType == Seq::Byte) {
        std::string name = "Seq";
        for (RE * re : *this) {
            if (CC* seq_cc = dynamic_cast<CC*>(re)) {
                name += seq_cc->getName();
            }
            else if (Name* seq_name = dynamic_cast<Name*>(re)) {
                name += seq_name->getName();
            }
            else {
                return "Bad Byte Sequence!";
            }
        }
        return name;
    }
    else {
        return "Unnamed Sequence";
    }
}

Seq::Type Seq::getType() const {
    return mType;
}

void Seq::setType(Seq::Type type) {
    mType = type;
}
