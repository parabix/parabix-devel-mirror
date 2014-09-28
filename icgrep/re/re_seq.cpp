/*
 *  Copyright (c) 2014 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#include "re_seq.h"
#include "re_cc.h"
#include "re_name.h"

namespace re {

std::string Seq::getName() const {
    if (mType == Seq::Type::Byte) {
        std::string name = "Seq";
        for (const RE * re : *this) {
            if (const CC* seq_cc = dyn_cast<const CC>(re)) {
                name += seq_cc->getName();
            }
            else if (const Name* seq_name = dyn_cast<const Name>(re)) {
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

}
