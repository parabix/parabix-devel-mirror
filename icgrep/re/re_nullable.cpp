#include "re_nullable.h"
#include <re/re_alt.h>             // for Alt, makeAlt
#include <re/re_any.h>             // for makeAny, Any
#include <re/re_assertion.h>       // for Assertion, Assertion::Sense, Asser...
#include <re/re_diff.h>            // for Diff, makeDiff
#include <re/re_intersect.h>       // for Intersect
#include <re/re_name.h>            // for Name
#include <re/re_rep.h>             // for Rep, makeRep
#include <re/re_seq.h>             // for Seq, makeSeq
#include <re/re_group.h>             // for Seq, makeSeq
#include <re/re_toolchain.h>
#include <re/validation.h>
#include <vector>                  // for vector, allocator
#include <llvm/Support/Casting.h>  // for dyn_cast, isa

/*

 A regular expression is nullable if it (a) matches the empty
 string, and (b) applies everywhere.  Note that Start (^) and
 End ($) match the empty string, but not everywhere).

*/

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
    bool validateCC(const CC *) override {return false;}
    bool validateRange(const Range *) override {return false;}
    bool validateDiff(const Diff * d) override {return validate(d->getLH());}
    bool validateIntersect(const Intersect * x) override {return validate(x->getLH()) || validate(x->getRH());}
};

bool isZeroWidth(const RE * re) {
    return ZeroWidthValidator().validateRE(re);
}

class NullablePrefixRemover: public RE_Transformer {
protected:
    RE * transformSeq(Seq * seq) override;
    RE * transformRep(Rep * rep) override;
    RE * transformAssertion(Assertion * a) override;
public:
    NullablePrefixRemover() : RE_Transformer("NullablePrefixRemoval") {}
};

RE * NullablePrefixRemover::transformSeq(Seq * seq) {
    std::vector<RE*> list;
    // if the sequence is empty, return it unmodified.
    if (isNullable(seq)) return seq;
    // Process the first element.
    auto i = seq->begin();
    auto e = transform(*i);
    while (isNullable(e)) {
        // Skip empty elements.
        i++;
        e = transform(*i);
    }
    // Special case: nothing skipped and first element unchanged.
    if ((i == seq->begin()) && (e == *i)) return seq;
    list.push_back(e);
    i++;
    while (i != seq->end()) {
        list.push_back(*i);
        i++;
    }
    return makeSeq(list.begin(), list.end());
}

RE * NullablePrefixRemover::transformRep(Rep * rep) {
    auto lb = rep->getLB();
    auto r = rep->getRE();
    if ((lb == 0) || isNullable(r)) {
        return makeSeq();
    }
    auto s = transform(r);
    if ((s == r) && (lb == rep->getUB())) return rep; // special case.  No transformation required.
    if (lb == 1) return s;
    if (lb == 2) return makeSeq({s, r});
    return makeSeq({s, makeRep(r, lb - 1, lb - 1)});
}

RE * NullablePrefixRemover::transformAssertion(Assertion * a) {
    return a;
}

RE * removeNullablePrefix(RE * r) {
    return NullablePrefixRemover().transformRE(r);
}

class NullableSuffixRemover: public RE_Transformer {
protected:
    RE * transformSeq(Seq * seq) override;
    RE * transformRep(Rep * rep) override;
    RE * transformAssertion(Assertion * a) override;
public:
    NullableSuffixRemover() : RE_Transformer("NullableSuffixRemoval") {}
};

RE * NullableSuffixRemover::transformSeq(Seq * seq) {
    std::vector<RE*> list;
    // if the sequence is empty, return it unmodified.
    if (isNullable(seq)) return seq;
    // Process the last element.
    auto ri = seq->rbegin();
    auto r = transform(*ri);
    while (isNullable(r)) {
        // Skip empty elements.
        ri++;
        r = transform(*ri);
    }
    // Special case: nothing skipped and first element unchanged.
    if ((ri == seq->rbegin()) && (r == *ri)) return seq;
    std::copy(seq->begin(), (ri + 1).base(), std::back_inserter(list));
    list.push_back(r);
    return makeSeq(list.begin(), list.end());
}

RE * NullableSuffixRemover::transformRep(Rep * rep) {
    auto lb = rep->getLB();
    auto r = rep->getRE();
    if ((lb == 0) || isNullable(r)) {
        return makeSeq();
    }
    auto s = transform(r);
    if ((s == r) && (lb == rep->getUB())) return rep; // special case.  No transformation required.
    if (lb == 1) return s;
    if (lb == 2) return makeSeq({r, s});
    return makeSeq({makeRep(r, lb - 1, lb - 1), s});
}

RE * NullableSuffixRemover::transformAssertion(Assertion * a) {
    return a;
}
RE * removeNullableSuffix(RE * r) {
    return NullableSuffixRemover().transformRE(r);
}

}
