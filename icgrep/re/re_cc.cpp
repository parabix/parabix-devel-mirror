/*
 *  Copyright (c) 2014 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#include "re_cc.h"
#include <llvm/Support/Compiler.h>
#include <UCD/CaseFolding_txt.h>
#include <sstream>

namespace re {

CC::CC(const CC * cc1, const CC * cc2)
: RE(ClassTypeId::CC)
, mSparseCharSet(std::move(cc1->mSparseCharSet + cc2->mSparseCharSet)) {

}

CC::CC(const CC & cc)
: RE(ClassTypeId::CC)
, mSparseCharSet(cc.mSparseCharSet) {

}

std::string CC::canonicalName(const CC_type type) const {
    std::stringstream name;
    name << std::hex;
    if ((type == ByteClass) && (max_codepoint() >= 0x80)) {
        name << "BC";
    } else {
        name << "CC";
    }
    char separator = '_';
    for (const interval_t & i : mSparseCharSet) {
        name << separator;
        if (lo_codepoint(i) == hi_codepoint(i)) {
            name << lo_codepoint(i);
        }
        else {
            name << lo_codepoint(i) << '_' << hi_codepoint(i);
        }
        separator = ',';
    }
    return name.str();
}

CC * subtractCC(const CC * a, const CC * b) {
    return makeCC(a->mSparseCharSet - b->mSparseCharSet);
}
    
CC * intersectCC(const CC * a, const CC * b) {
    return makeCC(a->mSparseCharSet & b->mSparseCharSet);
}
    
CC * caseInsensitize(const CC * cc) {
    CC * cci = makeCC();
    for (const interval_t & i : *cc) {
        caseInsensitiveInsertRange(cci, lo_codepoint(i), hi_codepoint(i));
    }
    return cci;
}
    
}
