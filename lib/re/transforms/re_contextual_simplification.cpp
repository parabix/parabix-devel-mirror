/*
 *  Copyright (c) 2018 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#include <re/transforms/re_contextual_simplification.h>

#include <boost/range/adaptor/reversed.hpp>
#include <llvm/Support/Casting.h>
#include <llvm/Support/ErrorHandling.h>
#include <llvm/Support/raw_ostream.h>
#include <re/adt/adt.h>
#include <re/adt/printer_re.h>
#include <re/analysis/validation.h>
#include <re/analysis/re_analysis.h>

using namespace llvm;
namespace re {

class GCB_Any_Free_Validator final : public RE_Validator {
public:
    GCB_Any_Free_Validator() : RE_Validator(""), mAnySeen(false), mGCBseen(false) {}
    bool validateName(const Name * n) override {
        std::string nm = n->getName();
        if (nm == ".") {
            if (mGCBseen) return false;
            mAnySeen = true;
        }
        if (nm == "\\b{g}") {
            if (mAnySeen) return false;
            mGCBseen = true;
        }
        return true;
    }
    bool validatePropertyExpression(const PropertyExpression * p) override {
        if ((p->getKind() == PropertyExpression::Kind::Boundary) && (p->getPropertyIdentifier() == "g")) {
            if (mAnySeen) return false;
            mGCBseen = true;
        }
        return true;
    }
private:
    bool mAnySeen;
    bool mGCBseen;
};

bool has_GCB_Any(RE * r) {
    return !GCB_Any_Free_Validator().validateRE(r);
}

struct NonEmptyValidator final : public RE_Validator {
    NonEmptyValidator() : RE_Validator() {}
    bool validateStart(const Start *) override {return false;}
    bool validateEnd(const End *) override {return false;}
    bool validateAssertion(const Assertion *) override {return false;}
    bool validateName(const Name * n) override {
        RE * defn = n->getDefinition();
        return defn && validate(defn);
    }
    bool validateDiff(const Diff * d) override {return validate(d->getLH());}
    bool validateSeq(const Seq * seq) override {
        for (RE * e : *seq) {
            if (validate(e)) return true;
        }
        return false;
    }
};

bool nonEmpty(const RE * r) {
    return NonEmptyValidator().validateRE(r);
}

RE * firstSym(RE * re) {
    if (Name * name = dyn_cast<Name>(re)) {
        if (LLVM_LIKELY(name->getDefinition() != nullptr)) {
            return firstSym(name->getDefinition());
        } else {
            UndefinedNameError(name);
        }
    } else if (isa<CC>(re) || isa<Start>(re) || isa<End>(re)) {
        return re;
    } else if (Seq * seq = dyn_cast<Seq>(re)) {
        CC * cc = makeCC();
        for (auto & si : *seq) {
            RE * fi = firstSym(si);
            cc = makeCC(cc, cast<CC>(fi));
            if (!isNullable(si)) {
                break;
            }
        }
        return cc;
    } else if (Alt * alt = dyn_cast<Alt>(re)) {
        CC * cc = makeCC();
        for (auto & ai : *alt) {
            RE * fi = firstSym(ai);
            cc = makeCC(cc, cast<CC>(fi));
        }
        return cc;
    } else if (Rep * rep = dyn_cast<Rep>(re)) {
        return firstSym(rep->getRE());
    } else if (Diff * diff = dyn_cast<Diff>(re)) {
        RE * lh = firstSym(diff->getLH());
        RE * rh = firstSym(diff->getRH());
        return subtractCC(cast<CC>(lh), cast<CC>(rh));
    } else if (Intersect * ix = dyn_cast<Intersect>(re)) {
        RE * lh = firstSym(ix->getLH());
        RE * rh = firstSym(ix->getRH());
        return intersectCC(cast<CC>(lh), cast<CC>(rh));
    }
    return makeCC();
}

RE * finalSym(RE * re) {
    if (Name * name = dyn_cast<Name>(re)) {
        if (LLVM_LIKELY(name->getDefinition() != nullptr)) {
            return finalSym(name->getDefinition());
        } else {
            UndefinedNameError(name);
        }
    } else if (isa<CC>(re) || isa<Start>(re) || isa<End>(re)) {
        return re;
    } else if (Seq * seq = dyn_cast<Seq>(re)) {
        CC * cc = makeCC();
        for (auto & si : boost::adaptors::reverse(*seq)) {
            RE * fi = finalSym(si);
            cc = makeCC(cc, cast<CC>(fi));
            if (!isNullable(si)) {
                break;
            }
        }
        return cc;
    } else if (Alt * alt = dyn_cast<Alt>(re)) {
        CC * cc = makeCC();
        for (auto & ai : *alt) {
            RE * fi = finalSym(ai);
            cc = makeCC(cc, cast<CC>(fi));
        }
        return cc;
    } else if (Rep * rep = dyn_cast<Rep>(re)) {
        return finalSym(rep->getRE());
    } else if (Diff * diff = dyn_cast<Diff>(re)) {
        RE * lh = finalSym(diff->getLH());
        RE * rh = finalSym(diff->getRH());
        return subtractCC(cast<CC>(lh), cast<CC>(rh));
    } else if (Intersect * ix = dyn_cast<Intersect>(re)) {
        RE * lh = finalSym(ix->getLH());
        RE * rh = finalSym(ix->getRH());
        return intersectCC(cast<CC>(lh), cast<CC>(rh));
    }
    return makeCC();
}


// RE_Context identifies the position of a regular expression in its
// containing sequence, as well as the context of that containing sequence
// recursively.

class RE_Context {
public:
    RE_Context(RE_Context * c, Seq * s, unsigned pos) : parentContext(c), contextSeq(s), contextPos(pos) {}
    // Return the preceding context item.
    bool empty() {return contextSeq == nullptr;}
    RE_Context priorContext();
    // Return the preceding Unicode CC item, if any.  Return a null context
    // if the current item is not a single Unicode unit length item, or if the
    // prior item is nullable.
    RE_Context priorCCorNull();
    RE_Context followingContext();
    // Return the following Unicode CC item, if any.  Return a null context
    // if the current item is not a single Unicode unit length item, or if the
    // following item is nullable.
    RE_Context followingCCorNull();
    RE * currentItem();

private:
    RE_Context * parentContext;
    Seq * contextSeq;
    unsigned contextPos;
};

RE_Context RE_Context::priorContext() {
    if (contextPos == 0) {
        if (parentContext) {
            return parentContext->priorContext();
        }
        return RE_Context{nullptr, nullptr, 0};
    }
    return RE_Context{parentContext, contextSeq, contextPos - 1};
}


RE_Context RE_Context::priorCCorNull() {
    if (!contextSeq || !isUnicodeUnitLength((*contextSeq)[contextPos])) return RE_Context{nullptr, nullptr, 0};
    RE_Context prior = priorContext();
    // Skip over zero width assertions which do not provide any matchable context.
    while ((prior.contextSeq) && isZeroWidth((*prior.contextSeq)[prior.contextPos]) && !isa<Start>((*prior.contextSeq)[prior.contextPos])) {
        prior = prior.priorContext();
    }
    if ((!prior.contextSeq) || isNullable((*prior.contextSeq)[prior.contextPos])) {
        return RE_Context{nullptr, nullptr, 0};
    }
    return prior;
}

RE_Context RE_Context::followingContext() {
    if (!contextSeq) return RE_Context{nullptr, nullptr, 0};
    if (contextPos >= contextSeq->size()-1) {
        if (parentContext) {
            return parentContext->followingContext();
        }
        return RE_Context{nullptr, nullptr, 0};
    }
    return RE_Context{parentContext, contextSeq, contextPos + 1};
}

RE_Context RE_Context::followingCCorNull() {
    if (!contextSeq || !isUnicodeUnitLength((*contextSeq)[contextPos])) return RE_Context{nullptr, nullptr, 0};
    RE_Context following = followingContext();
    // Skip over zero width assertions which do not provide any matchable context.
    while ((following.contextSeq) && isZeroWidth((*following.contextSeq)[following.contextPos]) && !isa<End>((*following.contextSeq)[following.contextPos])) {
        following = following.followingContext();
    }
    if ((!following.contextSeq) || isNullable((*following.contextSeq)[following.contextPos])) {
        return RE_Context{nullptr, nullptr, 0};
    }
    return following;
}

RE * RE_Context::currentItem() {
    if (contextSeq) return (*contextSeq)[contextPos];
    return nullptr;
}

//enum class MatchResult {Fail, Possible, Success};

struct ContextMatchCursor {
    RE_Context ctxt;
    RE * rslt;
};

ContextMatchCursor ctxt_match(RE * re, Assertion::Kind kind, ContextMatchCursor cursor) {
    if (cursor.ctxt.empty()) return ContextMatchCursor{cursor.ctxt, re};
    if (CC * cc = dyn_cast<CC>(re)) {
        RE_Context nextContext = cursor.ctxt;
        RE * item = cursor.ctxt.currentItem();
        if (isa<Start>(item) || isa<End>(item)) {
            return ContextMatchCursor{cursor.ctxt, makeAlt()};
        }
        if (!nonEmpty(item)) {
            return ContextMatchCursor{RE_Context{nullptr, nullptr, 0}, re};
        }
        RE * contextSym = nullptr;
        if (kind == Assertion::Kind::LookBehind) {
            contextSym = finalSym(item);
            nextContext = cursor.ctxt.priorCCorNull();
        } else {
            contextSym = firstSym(item);
            nextContext = cursor.ctxt.followingCCorNull();
        }
        //errs() << "assertedCC:" << Printer_RE::PrintRE(cc) << "\n";
        //errs() << "contextSym:" << Printer_RE::PrintRE(contextSym) << "\n";
        if (CC * contextCC = dyn_cast<CC>(contextSym)) {
            if (!(*contextCC).intersects(*cc)) return ContextMatchCursor{cursor.ctxt, makeAlt()};
            if (isEmptySeq(cursor.rslt) && (*contextCC).subset(*cc)) {
                return ContextMatchCursor{nextContext, makeSeq()};
            }
            return ContextMatchCursor{nextContext, intersectCC(contextCC, cc)};
        }
        return ContextMatchCursor{nextContext, re};
    } else if (isa<Start>(re)) {
        if (kind == Assertion::Kind::LookBehind) {
            RE * contextSym = finalSym(cursor.ctxt.currentItem());
            RE_Context nextContext = cursor.ctxt.priorCCorNull();
            if (isa<Start>(contextSym)) return ContextMatchCursor{nextContext, makeSeq()};
        }
        return ContextMatchCursor{cursor.ctxt, makeAlt()};
    } else if (isa<End>(re)) {
        if (kind == Assertion::Kind::LookAhead) {
            RE * contextSym = firstSym(cursor.ctxt.currentItem());
            RE_Context nextContext = cursor.ctxt.followingCCorNull();
            if (isa<End>(contextSym)) return ContextMatchCursor{nextContext, makeSeq()};
        }
        return ContextMatchCursor{cursor.ctxt, makeAlt()};
    } else if (Name * n = dyn_cast<Name>(re)) {
        RE * def = n->getDefinition();
        ContextMatchCursor submatch = ctxt_match(def, kind, cursor);
        return submatch;
    } else if (Capture * c = dyn_cast<Capture>(re)) {
        RE * def = c->getCapturedRE();
        ContextMatchCursor submatch = ctxt_match(def, kind, cursor);
        return submatch;
    } else if (Reference * r = dyn_cast<Reference>(re)) {
        RE * capture = r->getCapture();
        ContextMatchCursor submatch = ctxt_match(capture, kind, cursor);
        if (isEmptySet(submatch.rslt)) return submatch;
        return ContextMatchCursor{submatch.ctxt, re};
    } else if (Alt * alt = dyn_cast<Alt>(re)) {
        ContextMatchCursor bestSoFar = ContextMatchCursor{cursor.ctxt, makeAlt()};
        for (RE * a: *alt) {
            ContextMatchCursor a_match = ctxt_match(a, kind, cursor);
            if (isEmptySeq(a_match.rslt)) return a_match;
            if (!isEmptySet(a_match.rslt)) bestSoFar = a_match;
        }
        return bestSoFar;
    } else if (Seq * seq = dyn_cast<Seq>(re)) {
        ContextMatchCursor working = cursor;
        if (kind == Assertion::Kind::LookAhead) {
            for (RE * s: *seq) {
                working = ctxt_match(s, kind, working);
                if (isEmptySet(working.rslt)) return working;
            }
        } else {
            for (auto i = seq->rbegin(); i != seq->rend(); ++i) {
                working = ctxt_match(*i, kind, working);
                //errs() << "*i: " << Printer_RE::PrintRE(*i) << "\n";
                //errs() << "working.rslt: " << Printer_RE::PrintRE(working.rslt) << "\n";
                if (isEmptySet(working.rslt)) return working;
            }
        }
        if (isEmptySeq(working.rslt)) {
            return working;
        } else {
            return ContextMatchCursor{working.ctxt, seq};
        }

    } else if (const Rep * rep = dyn_cast<Rep>(re)) {
        int lb = rep->getLB();
        int ub = rep->getUB();
        RE * repeated = rep->getRE();
        ContextMatchCursor lb_cursor = cursor;
        for (int i = 0; i < lb; i++) {
            lb_cursor = ctxt_match(repeated, kind, lb_cursor);
            if (isEmptySet(lb_cursor.rslt)) return lb_cursor;
        }
        if (ub == Rep::UNBOUNDED_REP) {
            ContextMatchCursor star_rslt = lb_cursor;
            for (;;) {
                ContextMatchCursor next = ctxt_match(repeated, kind, star_rslt);
                if (isEmptySet(next.rslt)) return star_rslt;
                if (next.ctxt.empty()) return next;
                star_rslt = next;
            }
        } else {
            ContextMatchCursor ub_cursor = lb_cursor;
            for (int i = lb; i < ub; i++) {
                ContextMatchCursor next = ctxt_match(repeated, kind, ub_cursor);
                if (isEmptySet(next.rslt)) return ub_cursor;
                if (next.ctxt.empty()) return next;
                ub_cursor = next;
            }
            return ub_cursor;
        }
    } else if (const Diff * d = dyn_cast<Diff>(re)) {
        ContextMatchCursor lh_match = ctxt_match(d->getLH(), kind, cursor);
        ContextMatchCursor rh_match = ctxt_match(d->getRH(), kind, cursor);
        if (isEmptySet(rh_match.rslt)) return lh_match;
        if (isEmptySeq(rh_match.rslt)) return ContextMatchCursor{cursor.ctxt, makeAlt()};
        if (isEmptySet(lh_match.rslt)) return lh_match;
        return ContextMatchCursor{lh_match.ctxt, re};
    } else if (const Intersect * x = dyn_cast<Intersect>(re)) {
        ContextMatchCursor lh_match = ctxt_match(x->getLH(), kind, cursor);
        ContextMatchCursor rh_match = ctxt_match(x->getRH(), kind, cursor);
        if (isEmptySet(lh_match.rslt)) return lh_match;
        if (isEmptySet(rh_match.rslt)) return rh_match;
        if (isEmptySeq(lh_match.rslt)) return rh_match;
        if (isEmptySeq(rh_match.rslt)) return lh_match;
        return lh_match;
    } else if (const Assertion * a = dyn_cast<Assertion>(re)) {
        if (a->getKind() == kind) {
            ContextMatchCursor assertResult = ctxt_match(a->getAsserted(), kind, cursor);
            return ContextMatchCursor{cursor.ctxt, makeAssertion(assertResult.rslt, kind, a->getSense())};
        }
        return ContextMatchCursor{cursor.ctxt, re};
    }
    return ContextMatchCursor{cursor.ctxt, re};
}

class ContextualAssertionSimplifier final : public RE_Transformer {
public:
    ContextualAssertionSimplifier() :
        RE_Transformer("ContextualAssertionSimplifier", NameTransformationMode::TransformDefinition),
        mContext(RE_Context(nullptr, nullptr, 0)) {}
    ContextualAssertionSimplifier(RE_Context ctxt) :
        RE_Transformer("", NameTransformationMode::TransformDefinition),
        mContext(ctxt) {}
    RE * transformSeq(Seq * s) override {
        std::vector<RE *> t;
        t.reserve(s->size());
        bool changeSeen = false;
        for (unsigned i = 0; i < s->size(); i++) {
            RE * s_i = (*s)[i];
            RE * t_i = ContextualAssertionSimplifier(RE_Context(&mContext, s, i)).transformRE(s_i);
            t.push_back(t_i);
            if (t_i != s_i) changeSeen = true;
        }
        if (changeSeen) return makeSeq(t.begin(), t.end());
        return s;
    }

    RE * transformRep(Rep * r) override {
        RE * repeated = r->getRE();
        // The repeated subexpression can be transformed, but only with an
        // empty context.
        RE * t = ContextualAssertionSimplifier(RE_Context(nullptr, nullptr, 0)).transformRE(repeated);
        if (t == repeated) return r;
        return makeRep(t, r->getLB(), r->getUB());
    }

    RE * transformStart(Start * s) override {
        RE * prior = mContext.priorContext().currentItem();
        if (prior) {
            if (isa<Start>(prior)) {
                return makeSeq();
            } else if (nonEmpty(prior)) {
                return makeAlt();
            }
        }
        return s;
    }

    RE * transformEnd(End * e) override {
        RE * following = mContext.followingContext().currentItem();
        if (following) {
            if (isa<End>(following)) {
                return makeSeq();
            } else if (nonEmpty(following)) {
                return makeAlt();
            }
        }
        return e;
    }

    RE * transformAssertion(Assertion * a) override {
        RE * asserted = a->getAsserted();
        Assertion::Kind k = a->getKind();
        Assertion::Sense s = a->getSense();
        RE_Context ctxt = mContext;
        if (k == Assertion::Kind::LookAhead) {
            ctxt = mContext.followingContext();
        } else if (k == Assertion::Kind::LookBehind) {
            ctxt = mContext.priorContext();
        } else {
            // BoundaryAssertion - TODO - evaluate if possible
            //RE * prior = mContext.priorContext().currentItem();
            //RE * following = mContext.followingContext().currentItem();
            return a;
        }
        ContextMatchCursor x = ctxt_match(asserted, k, ContextMatchCursor{ctxt, makeSeq()});
        return makeAssertion(x.rslt, k, s);
    }

    private:
        RE_Context mContext;
};


RE * simplifyAssertions(RE * r) {
    // If a regular expression has an Any and a GCB, it is unlikely
    // to be of much use to eliminate assertions and possibly very
    // costly to inline multiple partially optimized assertions.
    if (has_GCB_Any(r)) return r;
    return ContextualAssertionSimplifier().transformRE(r);
}

} // namespace re
