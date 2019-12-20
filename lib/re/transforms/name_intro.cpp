/*
 *  Copyright (c) 2019 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#include <re/transforms/name_lookaheads.h>

#include <llvm/Support/Casting.h>
#include <llvm/Support/ErrorHandling.h>
#include <re/adt/adt.h>
#include <re/transforms/re_transformer.h>

using namespace llvm;

namespace re {

class VariableLengthCCNamer final : public RE_Transformer {
public:
    VariableLengthCCNamer(int UTF_bits) : RE_Transformer("LookAheadNamer"), mUTF_bits(UTF_bits) {}
    RE * transformCC (CC * cc) override {
        bool variable_length;
        if (UTF_bits == 8) {
            variable_length = UTF<8>::encoded_length(lo_codepoint(cc->front())) < <UTF<8>::encoded_length(hi_codepoint(cc->back()));
        } else if (UTF_bits == 8) {
            variable_length = UTF<16>::encoded_length(lo_codepoint(cc->front())) < <UTF<16>::encoded_length(hi_codepoint(cc->back()));
        }
        if (variable_length) {
            return makeName(cc::canonicalName(cc), cc, Name::Type::Unicode);
        }
        return cc;
    }
private:
    int mUTF_bits;
};

RE * VariableLengthCCNamer::transformAssertion (Assertion * a) {
    RE * x0 = a->getAsserted();
    RE * x = transform(x0);
    if ((a->getKind() == Assertion::Kind::LookAhead) && !isa<Name>(x)) {
        std::string name = Printer_RE::PrintRE(x);
        return makeAssertion(makeName(name, x), Assertion::Kind::LookAhead, a->getSense());
    } else if (x == x0) {
        return a;
    } else {
        return makeAssertion(x, a->getKind(), a->getSense());
    }
}

RE * name_variable_length_CCs(RE * r, int UTF_bits) {
    return VariableLengthCCNamer(UTF_bits).transformRE(re);
}
}

