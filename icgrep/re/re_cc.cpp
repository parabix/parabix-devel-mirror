/*
 *  Copyright (c) 2014 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#include "re_cc.h"
#include <llvm/Support/Compiler.h>
#include <sstream>

namespace re {

std::string CC::canonicalName(const CC_type type) const {
    std::stringstream name;
    name << std::hex;
    if ((type == CC_type::ByteClass) && (max_codepoint() >= 0x80)) {
        name << "BC";
    } else {
        name << "CC";
    }
    char separator = '_';
    for (const interval_t i : *this) {
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
    
CC::CC(const cc::Alphabet * alphabet)
: RE(ClassTypeId::CC)
, UnicodeSet()
, mAlphabet(alphabet) {}


CC::CC(const CC & cc)
: RE(ClassTypeId::CC)
, UCD::UnicodeSet(cc)
, mAlphabet(cc.getAlphabet()) {}


CC::CC(const codepoint_t codepoint, const cc::Alphabet * alphabet)
: RE(ClassTypeId::CC)
, UCD::UnicodeSet(codepoint)
, mAlphabet(alphabet) {}


CC::CC(const codepoint_t lo_codepoint, const codepoint_t hi_codepoint, const cc::Alphabet * alphabet)
: RE(ClassTypeId::CC)
, UCD::UnicodeSet(lo_codepoint, hi_codepoint)
, mAlphabet(alphabet) {}


CC::CC(const CC * cc1, const CC * cc2)
: RE(ClassTypeId::CC)
, UCD::UnicodeSet(std::move(*cc1 + *cc2))
, mAlphabet(cc1->getAlphabet()) {
    assert (cc1->getAlphabet() == cc2->getAlphabet());
}


CC::CC(const UCD::UnicodeSet && set, const cc::Alphabet * alphabet)
: RE(ClassTypeId::CC)
, UCD::UnicodeSet(std::move(set))
, mAlphabet(alphabet) {}


CC::CC(std::initializer_list<interval_t>::iterator begin, std::initializer_list<interval_t>::iterator end, const cc::Alphabet * alphabet)
: RE(ClassTypeId::CC)
, UCD::UnicodeSet(begin, end)
, mAlphabet(alphabet) {}


CC::CC(const std::vector<interval_t>::iterator begin, const std::vector<interval_t>::iterator end, const cc::Alphabet * alphabet)
: RE(ClassTypeId::CC)
, UCD::UnicodeSet(begin, end)
, mAlphabet(alphabet) {}

}
