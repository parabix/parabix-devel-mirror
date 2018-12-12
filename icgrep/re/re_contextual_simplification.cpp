/*
 *  Copyright (c) 2018 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#include "re_contextual_simplification.h"
#include <re/re_any.h>
#include <re/re_cc.h>
#include <re/re_diff.h>
#include <re/re_name.h>
#include <re/re_seq.h>
#include <re/re_assertion.h>
#include <re/validation.h>
#include <llvm/Support/Casting.h>
#include <llvm/Support/ErrorHandling.h>

namespace re {

class Context : private RE_Transformer {
public:
    Context(Seq * s, size_t idx);
    bool good() const noexcept;
    RE * simplify(RE * re);
private:
    std::vector<RE *> before;
    std::vector<RE *> after;

    Name * any;

    bool _good = true;

    template<typename Iterator>
    std::vector<RE *> makeContext(Iterator begin, Iterator end);
    RE * transformAssertion(Assertion * a) override;
    RE * simplifyAsserted(RE * asserted, std::vector<RE *> const & context);
    RE * simplifyForwardAssertion(RE * asserted);
    RE * simplifyBackwardAssertion(RE * asserted);
};


class ResolvesToCC : public RE_Validator {
public:
    inline bool validateCC(CC *) override { return true; }
    inline bool validateStart(Start *) override { return false; }
    inline bool validateEnd(End *) override { return false; }
    inline bool validateSeq(Seq * s) override { return (s) && s->size() <= 1; }
    inline bool validateIntersect(Intersect *) override { return false; }
    inline bool validateRange(Range *) override { return false; }
    inline bool validateAssertion(Assertion *) override { return false; }
};

class ValidateAsserted : public RE_Validator {
public:
    inline bool validateStart(Start * s) override { return false; }
    inline bool validateEnd(End * e) override { return false; }
    inline bool validateRep(Rep * r) override { return false; }
    inline bool validateIntersect(Intersect * i) override { return false; }
    inline bool validateRange(Range * r) override { return false; }
    inline bool validateAssertion(Assertion * a) override { return false; }
    inline bool validateDiff(Diff * d) override {
        return ResolvesToCC{}.validateRE(d);
    }
    inline bool validateSeq(Seq * s) {
        auto ccValidator = ResolvesToCC{};
        for (auto e : *s) {
            if (!ccValidator.validateRE(e) || !validateRE(e)) {
                return false;
            }
        }
        return true;
    }
};

class ValidateZeroWidth : public RE_Validator {
public:
    inline bool validateStart(Start * s) { return false; }
    inline bool validateEnd(End * e) { return false; }
    inline bool validateCC(CC * cc) { return false; }
    inline bool validateRep(Rep * rep) { return false; }
    inline bool validateIntersect(Intersect * e) { return false; }
    inline bool validateRange(Range * rg) { return false; }
    inline bool validateAssertion(Assertion * a) { return true; }

    inline bool validateSeq(Seq * s) { 
        for (auto e : *s) {
            if (!validateRE(e)) return false;
        }
        return s->size() == 0;
    }
};

class AssertionPrep : public RE_Transformer {
public:
    using Sense = Assertion::Sense;
    AssertionPrep(Assertion * assertion);
    RE * transformCC(CC * cc) override;
    RE * transformDiff(Diff * d) override;
private:
    Sense sense;
    Name * any;
};



Context::Context(Seq * s, size_t idx) 
: RE_Transformer("Contextual Engine", NameTransformationMode::TransformDefinition)
{
    if (idx < s->size()) {
        this->before = makeContext(s->rbegin() + (s->size() - idx), s->rend());
        this->after = makeContext(s->begin() + idx + 1, s->end());
        if (before.size() == 0 && after.size() == 0) {
            _good = false;
        }
    } else {
        this->_good = false;
    }

    this->any = llvm::cast<Name>(makeAny());
}

bool Context::good() const noexcept {
    return this->_good;
}

RE * Context::simplify(RE * re) {
    RE * rt = transformRE(re);
    return rt;
}

template<typename Iterator>
std::vector<RE *> Context::makeContext(Iterator begin, Iterator end) {
    std::vector<RE *> rt{};
    for (auto i = begin; i != end; i++) {
        if (llvm::isa<CC>(*i)) {
            rt.push_back(*i);
        } else if (llvm::isa<Diff>(*i)) {
            Diff * d = llvm::cast<Diff>(*i);
            if (llvm::isa<CC>(d->getLH()) && llvm::isa<CC>(d->getRH())) {
                rt.push_back(subtractCC(llvm::cast<CC>(d->getLH()), llvm::cast<CC>(d->getRH())));
            } else {
                break;
            }
        } else if (ValidateZeroWidth().validateRE(*i)) {
            continue;
        } else if (llvm::isa<Name>(*i)) {
            if (llvm::isa<CC>(llvm::cast<Name>(*i)->getDefinition())) {
                rt.push_back(llvm::cast<Name>(*i)->getDefinition());
            } else {
                break;
            }
        } else {
            break;
        }
    }
    return rt;
}

RE * reverseAsserted(RE * re) {
    if (llvm::isa<Seq>(re)) {
        Seq * seq = llvm::cast<Seq>(re);
        return makeSeq(seq->rbegin(), seq->rend());
    } else {
        return re;
    }
}

RE * Context::transformAssertion(Assertion * a) {
    using Kind = Assertion::Kind;
    if (good() && ValidateAsserted{}.validateRE(a->getAsserted())) {
        AssertionPrep prep{a};
        RE * asserted = prep.transformRE(a->getAsserted());
        RE * simplifiedAsserted = nullptr;
        if (a->getKind() == Kind::Lookahead) {
            simplifiedAsserted = simplifyAsserted(asserted, after);
        } else if (a->getKind() == Kind::Lookbehind) {
            asserted = reverseAsserted(asserted);
            simplifiedAsserted = reverseAsserted(simplifyAsserted(asserted, before));
        } else {
            return a;
        }

        if (simplifiedAsserted != nullptr) {
            if (simplifiedAsserted == asserted) {
                return a;
            } else if (isEmptySet(simplifiedAsserted)) {
                return makeAlt();
            } else if (isEmptySeq(simplifiedAsserted)) {
                return makeSeq();
            } else {
                return makeAssertion(simplifiedAsserted, a->getKind(), Assertion::Sense::Positive);
            }
        } else {
            return a;
        }
    } else {
        return a;
    }
}

Seq * latterSeq(Seq * s, size_t i) {
    Seq * seq = llvm::cast<Seq>(makeSeq());
    for (auto it = s->begin() + i; it != s->end(); it++) {
        seq->push_back(*it);
    }
    return s;
}

RE * Context::simplifyAsserted(RE * asserted, std::vector<RE *> const & context) {
    if (LLVM_LIKELY(llvm::isa<CC>(asserted))) {
        if (context.size() > 0) {
            CC * assertedCC = llvm::cast<CC>(asserted);
            CC * contextCC = llvm::cast<CC>(context[0]);
            CC * intersect = intersectCC(assertedCC, contextCC);
            if (intersect->empty()) {
                return makeAlt();
            } else if (subtractCC(contextCC, assertedCC)->empty()) {
                return makeSeq();
            } else if (*intersectCC(contextCC, assertedCC) == *assertedCC) {
                return asserted;
            } else {
                return intersect;
            }
        }
        return asserted;
    } else if (llvm::isa<Seq>(asserted)) {
        Seq * a = llvm::cast<Seq>(asserted);
        std::vector<RE *> elems{a->begin(), a->end()};
        bool isWholeSeqSuperSet = context.size() >= a->size();
        bool didChange = false;
        elems.reserve(a->size());

        for (size_t i = 0; i < std::min(a->size(), context.size()); ++i) {
            RE * simplifiedElem = simplifyAsserted((*a)[i], std::vector<RE *>{context[i]});
            if (isEmptySet(simplifiedElem)) {
                return makeAlt();
            }
            if (simplifiedElem != (*a)[i]) {
                didChange = true;
                if (!isEmptySeq(simplifiedElem)) {
                    isWholeSeqSuperSet = false;
                    elems[i] = simplifiedElem;
                } else {
                    // a[i] is supperset of context[i]. Can't change a[i] to 
                    // Seq[] as it will invalidate the Seq structure. Instead,
                    // set a[i] to context[i] the smaller of the two CCs.
                    elems[i] = context[i];
                }
            } else {
                isWholeSeqSuperSet = false;
            }
        }

        if (isWholeSeqSuperSet) {
            return makeSeq();
        }

        if (didChange) {
            return makeSeq(elems.begin(), elems.end());
        } else {
            return asserted;
        }
    } else if (llvm::isa<Alt>(asserted)) {
        Alt * a = llvm::cast<Alt>(asserted);
        std::vector<RE *> elems{};
        elems.reserve(a->size());
        bool did_change = false;
        for (auto e : *a) {
            RE * e0 = simplifyAsserted(e, context);
            if (e0 != e) {
                elems.push_back(e0);
                did_change = true;
            } else {
                elems.push_back(e);
            }
        }

        if (did_change) {
            return makeAlt(elems.begin(), elems.end());
        } else {
            return asserted;
        }
    } else if (llvm::isa<Name>(asserted)) {
        RE * def = llvm::cast<Name>(asserted)->getDefinition();
        if (LLVM_UNLIKELY(def == nullptr)) {
            llvm_unreachable("undefined name");
        }
        RE * simp = simplifyAsserted(def, context);
        if (simp == def) {
            return asserted;
        } else {
            return simp;
        }
    } else {
        llvm_unreachable("Context::simplifyAsserted: Unexpected asserted value");
    }
}



AssertionPrep::AssertionPrep(Assertion * assertion)
: RE_Transformer("", NameTransformationMode::TransformDefinition), 
  sense(assertion->getSense())
{
    this->any = llvm::cast<Name>(makeAny());
}

RE * AssertionPrep::transformCC(CC * cc) {
    if (sense == Sense::Negative) {
        return subtractCC(llvm::dyn_cast<CC>(any->getDefinition()), cc);
    } else {
        return cc;
    }
}

RE * AssertionPrep::transformDiff(Diff * d) {
    CC * lh = llvm::dyn_cast<CC>(transform(d->getLH()));
    CC * rh = llvm::dyn_cast<CC>(transform(d->getRH()));
    if (LLVM_UNLIKELY(lh == nullptr || rh == nullptr)) {
        llvm_unreachable("Nonvalidated use of Diff");
    }
    return transform(subtractCC(lh, rh));
}


RE * RE_ContextSimplifier::transformSeq(Seq * s) {
    std::vector<RE *> seq{};
    seq.reserve(s->size());
    bool hasChanged = false;
    for (size_t i = 0; i < s->size(); ++i) {
        if (!validateAssertionFree((*s)[i])) {
            Context context{s, i};
            if (!context.good()) {  // abort simplification is context is bad
                seq.push_back((*s)[i]);
                continue;
            }
            RE * simp = context.simplify((*s)[i]);
            if (simp != (*s)[i]) {
                hasChanged = true;
                if (isEmptySet(simp)) {
                    return makeAlt();
                } else {
                    seq.push_back(simp);
                }
            } else {
                seq.push_back((*s)[i]);
            }
        } else {
            seq.push_back((*s)[i]);
        }
    }

    
    if (hasChanged) {
        auto rt = makeSeq(seq.begin(), seq.end());
        return rt;
    } else {
        return s;
    }
}

} // namespace re
