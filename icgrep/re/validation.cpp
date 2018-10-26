/*
 *  Copyright (c) 2018 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#include <re/validation.h>
#include <re/re_toolchain.h>
#include <re/re_any.h>
#include <re/re_name.h>
#include <re/re_cc.h>
#include <re/re_start.h>
#include <re/re_end.h>
#include <re/re_alt.h>
#include <re/re_seq.h>
#include <re/re_diff.h>
#include <re/re_intersect.h>
#include <re/re_group.h>
#include <re/re_range.h>
#include <re/re_assertion.h>
#include <cc/alphabet.h>
#include <re/printer_re.h>
#include <llvm/Support/raw_ostream.h>
#include <llvm/Support/ErrorHandling.h>

namespace re {

bool RE_Validator::validateRE(RE * re) {
    bool validated = validate(re);
    if ((mValidatorName != "") && (PrintOptionIsSet(ShowAllREs) || (PrintOptionIsSet(ShowREs) && (!validated))))  {
        llvm::errs() << mValidatorName << (validated ? " success:\n" : " failure:\n") << Printer_RE::PrintRE(re) << '\n';
    }
    return validated;
}

bool RE_Validator::validate(RE * const re) {
    using T = RE::ClassTypeId;
#define VALIDATE(Type) \
case T::Type: return validate##Type(llvm::cast<Type>(re)); break
    switch (re->getClassTypeId()) {
            VALIDATE(Alt);
            VALIDATE(Assertion);
            VALIDATE(CC);
            VALIDATE(Range);
            VALIDATE(Diff);
            VALIDATE(End);
            VALIDATE(Intersect);
            VALIDATE(Name);
            VALIDATE(Group);
            VALIDATE(Rep);
            VALIDATE(Seq);
            VALIDATE(Start);
        default: llvm_unreachable("Unknown RE type");
    }
#undef VALIDATE
}

bool RE_Validator::validateName(Name * n) {
    RE * def = n->getDefinition();
    return (def) && validate(def);
}

bool RE_Validator::validateCC(CC * cc) {
    return true;
}

bool RE_Validator::validateStart(Start * s) {
    return true;
}

bool RE_Validator::validateEnd(End * e) {
    return true;
}

bool RE_Validator::validateSeq(Seq * seq) {
    for (RE * e : *seq) {
        if (!validate(e)) return false;
    }
    return true;
}

bool RE_Validator::validateAlt(Alt * alt) {
    for (RE * e : *alt) {
        if (!validate(e)) return false;
    }
    return true;
}

bool RE_Validator::validateRep(Rep * r) {
    return validate(r->getRE());
}

bool RE_Validator::validateIntersect(Intersect * ix) {
    return validate(ix->getLH()) && validate(ix->getRH());
}

bool RE_Validator::validateDiff(Diff * d) {
    return validate(d->getLH()) && validate(d->getRH());
}

bool RE_Validator::validateRange(Range * rg) {
    return validate(rg->getLo()) && validate(rg->getHi());
}

bool RE_Validator::validateGroup(Group * g) {
    return validate(g->getRE());
}

bool RE_Validator::validateAssertion(Assertion * a) {
    return validate(a->getAsserted());
}

bool validateNamesDefined(RE * r) {
    return RE_Validator("NamesDefinedValidator").validateRE(r);
}
    
class AlphabetValidator : public RE_Validator {
public:
    AlphabetValidator(const cc::Alphabet * a) : RE_Validator("AlphabetValidator"), mAlphabet(a) {}
    
    bool validateCC(CC * cc) override {return cc->getAlphabet() == mAlphabet;}
private:
    const cc::Alphabet * mAlphabet;
};

bool validateAlphabet(const cc::Alphabet * a, RE * r) {
    return AlphabetValidator(a).validateRE(r);
}
    
class AssertionFreeValidator : public RE_Validator {
public:
    AssertionFreeValidator() : RE_Validator("AssertionFreeValidator") {}
    
    bool validateAssertion(Assertion * a) override {return false;}
    bool validateStart(Start * s) override {return false;}
    bool validateEnd(End * e) override {return false;}
};

bool validateAssertionFree(RE * r) {
    return AssertionFreeValidator().validateRE(r);
}

}
