/*
 *  Copyright (c) 2019 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#include <re/transforms/name_intro.h>

#include <llvm/Support/Casting.h>
#include <llvm/Support/ErrorHandling.h>
#include <re/adt/adt.h>
#include <re/transforms/re_transformer.h>
#include <unicode/utf/UTF.h>

using namespace llvm;

namespace re {

class VariableLengthCCNamer final : public RE_Transformer {
public:
    VariableLengthCCNamer(int UTF_bits) : RE_Transformer("VariableLengthCCNamer"), mUTF_bits(UTF_bits) {}
    RE * transformCC (CC * cc) override {
        bool variable_length;
        if (mUTF_bits == 8) {
            variable_length = UTF<8>::encoded_length(lo_codepoint(cc->front())) < UTF<8>::encoded_length(hi_codepoint(cc->back()));
        } else if (mUTF_bits == 16) {
            variable_length = UTF<16>::encoded_length(lo_codepoint(cc->front())) < UTF<16>::encoded_length(hi_codepoint(cc->back()));
        }
        if (variable_length) {
            return makeName(cc->canonicalName(), Name::Type::Unicode, cc);
        }
        return cc;
    }
private:
    int mUTF_bits;
};

RE * name_variable_length_CCs(RE * r, int UTF_bits) {
    return VariableLengthCCNamer(UTF_bits).transformRE(r);
}
}

