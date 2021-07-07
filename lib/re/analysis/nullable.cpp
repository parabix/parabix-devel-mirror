/*
 *  Copyright (c) 2019 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#include <re/analysis/nullable.h>

#include <re/adt/adt.h>
#include <re/analysis/validation.h>

using namespace llvm;

namespace re {

bool isNullable(const RE * re) {
    if (const Seq * re_seq = dyn_cast<const Seq>(re)) {
        for (const RE * re : *re_seq) {
            if (!isNullable(re)) {
                return false;
            }
        }
        return true;
    } else if (const Alt * re_alt = dyn_cast<const Alt>(re)) {
        for (const RE * re : *re_alt) {
            if (isNullable(re)) {
                return true;
            }
        }
    } else if (const Rep* re_rep = dyn_cast<const Rep>(re)) {
        return (re_rep->getLB() == 0) || isNullable(re_rep->getRE());
    } else if (isa<Diff>(re)) {
        // a Diff of Seq({}) and an Assertion represents a complemented assertion.
        //return isNullable(d->getLH()) && (!isNullable(d->getRH())) && (!isZeroWidth(d->getRH()));
        return false;
    } else if (const Intersect * e = dyn_cast<const Intersect>(re)) {
        return isNullable(e->getLH()) && isNullable(e->getRH());
    } else if (const Group * g = dyn_cast<const Group>(re)) {
        return isNullable(g->getRE());
    }
    return false;
}

struct ZeroWidthValidator : public RE_Validator {
    ZeroWidthValidator() : RE_Validator() {}
    bool validateName(const Name * n) override {
        RE * defn = n->getDefinition();
        return defn && validate(defn);
    }
    bool validateAssertion(const Assertion * a) override {return true;}
    bool validateAny(const Any *) override {return false;}
    bool validateCC(const CC *) override {return false;}
    bool validateRange(const Range *) override {return false;}
    bool validateDiff(const Diff * d) override {return validate(d->getLH());}
    bool validateIntersect(const Intersect * x) override {return validate(x->getLH()) || validate(x->getRH());}
};

bool isZeroWidth(const RE * re) {
    return ZeroWidthValidator().validateRE(re);
}

} // namespace re
