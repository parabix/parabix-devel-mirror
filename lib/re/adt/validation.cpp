/*
 *  Copyright (c) 2018 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#include <re/adt/validation.h>

#include <llvm/Support/raw_ostream.h>
#include <llvm/Support/ErrorHandling.h>
#include <re/adt/adt.h>
#include <re/adt/printer_re.h>
#include <re/alphabet/alphabet.h>
#include <re/toolchain/toolchain.h>

namespace re {

bool RE_Validator::validateRE(const RE * re) {
    bool validated = validate(re);
    if ((mValidatorName != "") && (PrintOptionIsSet(ShowAllREs) || (PrintOptionIsSet(ShowREs) && (!validated))))  {
        llvm::errs() << mValidatorName << (validated ? " success:\n" : " failure:\n") << Printer_RE::PrintRE(re) << '\n';
    }
    return validated;
}

bool RE_Validator::validate(const RE * const re) {
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

bool RE_Validator::validateName(const Name * n) {
    RE * def = n->getDefinition();
    return (def) && validate(def);
}

bool RE_Validator::validateCC(const CC * cc) {
    return true;
}

bool RE_Validator::validateStart(const Start * s) {
    return true;
}

bool RE_Validator::validateEnd(const End * e) {
    return true;
}

bool RE_Validator::validateSeq(const Seq * seq) {
    for (RE * e : *seq) {
        if (!validate(e)) return false;
    }
    return true;
}

bool RE_Validator::validateAlt(const Alt * alt) {
    for (RE * e : *alt) {
        if (!validate(e)) return false;
    }
    return true;
}

bool RE_Validator::validateRep(const Rep * r) {
    return validate(r->getRE());
}

bool RE_Validator::validateIntersect(const Intersect * ix) {
    return validate(ix->getLH()) && validate(ix->getRH());
}

bool RE_Validator::validateDiff(const Diff * d) {
    return validate(d->getLH()) && validate(d->getRH());
}

bool RE_Validator::validateRange(const Range * rg) {
    return validate(rg->getLo()) && validate(rg->getHi());
}

bool RE_Validator::validateGroup(const Group * g) {
    return validate(g->getRE());
}

bool RE_Validator::validateAssertion(const Assertion * a) {
    return validate(a->getAsserted());
}

bool validateNamesDefined(const RE * r) {
    return RE_Validator("NamesDefinedValidator").validateRE(r);
}
    
class AlphabetValidator : public RE_Validator {
public:
    AlphabetValidator(const cc::Alphabet * a) : RE_Validator("AlphabetValidator"), mAlphabet(a) {}
    
    bool validateCC(const CC * cc) override {return cc->getAlphabet() == mAlphabet;}
private:
    const cc::Alphabet * mAlphabet;
};

bool validateAlphabet(const cc::Alphabet * a, const RE * r) {
    return AlphabetValidator(a).validateRE(r);
}
    
class AssertionFreeValidator : public RE_Validator {
public:
    AssertionFreeValidator() : RE_Validator("AssertionFreeValidator") {}
    
    bool validateAssertion(const Assertion * a) override {return false;}
    bool validateStart(const Start * s) override {return false;}
    bool validateEnd(const End * e) override {return false;}
};

bool validateAssertionFree(const RE * r) {
    return AssertionFreeValidator().validateRE(r);
}

}
