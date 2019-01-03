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
#include <re/re_start.h>
#include <re/re_end.h>
#include <re/re_reverse.h>
#include <re/validation.h>
#include <llvm/Support/Casting.h>
#include <llvm/Support/ErrorHandling.h>
#include <llvm/Support/raw_ostream.h>
#include <re/printer_re.h>

namespace re {
    
// RE_Context identifies the position of a regular expression in its
// containing sequence, as well as the context of that containing sequence
// recursively.

class RE_Context {
public:
    RE_Context(RE_Context * c, Seq * s, unsigned pos) : parentContext(c), contextSeq(s), contextPos(pos) {}
    RE_Context priorContext();
    RE_Context followingContext();
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

RE * RE_Context::currentItem() {
    if (contextSeq) return (*contextSeq)[contextPos];
    return nullptr;
}

RE * simplifyCC(CC * assertedCC, CC * contextCC) {
    //llvm::errs() << "assertedCC:" << Printer_RE::PrintRE(assertedCC) << "\n";
    //llvm::errs() << "contextCC:" << Printer_RE::PrintRE(contextCC) << "\n";
    if ((*contextCC).subset(*assertedCC)) return makeSeq();
    if ((*contextCC).intersects(*assertedCC)) return assertedCC;
    return makeAlt();
}

class SimplifyAsserted : public RE_Transformer {
public:
    SimplifyAsserted(RE_Context & context, Assertion::Kind kind) :
        RE_Transformer("SimplifyAsserted"),
        mContext(context), mKind(kind) {}
        
    RE * transformName(Name * name) override {
        RE * defn = name->getDefinition();
        if (defn) {
            RE * s = transform(defn);
            if (s == defn) return name;
            else return s;
        }
        return name;
    }
    
    RE * transformSeq(Seq * s) override {
        RE_Context ctxt = mContext;
        bool allSatisfied = true;
        if (mKind == Assertion::Kind::Lookbehind) {
            for (unsigned i = s->size() - 1; i >= 0; i--) {
                RE * simplifiedElem = SimplifyAsserted(ctxt, mKind).transform((*s)[i]);
                if (isEmptySet(simplifiedElem)) {
                    return makeAlt();
                } else if (!isEmptySeq(simplifiedElem)) {
                    allSatisfied = false;
                } else if (allSatisfied && (i == 0)) {
                    return makeSeq();
                }
                ctxt = ctxt.priorContext();
                RE * prev = ctxt.currentItem();
                if (!prev || !llvm::isa<CC>(ctxt.currentItem())) {
                    return s;
                }
            }
        } else if (mKind == Assertion::Kind::Lookahead) {
            for (unsigned i = 0; i < s->size(); i++) {
                RE * simplifiedElem = SimplifyAsserted(ctxt, mKind).transform((*s)[i]);
                if (isEmptySet(simplifiedElem)) {
                    return makeAlt();
                } else if (!isEmptySeq(simplifiedElem)) {
                    allSatisfied = false;
                } else if (allSatisfied && (i == s->size() - 1)) {
                    return makeSeq();
                }
                ctxt = ctxt.followingContext();
                RE * next = ctxt.currentItem();
                if (!next || !llvm::isa<CC>(next)) {
                    return s;
                }
            }
        }
        return s;
    }
    
    RE * transformCC(CC * cc) override {
        RE * context = nullptr;
        if (mKind == Assertion::Kind::Lookbehind) {
            context = mContext.priorContext().currentItem();
        } else if (mKind == Assertion::Kind::Lookahead) {
            context = mContext.followingContext().currentItem();
        }
        if (context) {
            if (CC * contextCC = llvm::dyn_cast<CC>(context)) {
                return simplifyCC(cc, contextCC);
            }
        }
        return cc;
    }
private:
    RE_Context mContext;
    Assertion::Kind mKind;
};

class ContextualAssertionSimplifier : public RE_Transformer {
public:
    ContextualAssertionSimplifier(RE_Context ctxt = RE_Context(nullptr, nullptr, 0)) :
        RE_Transformer("ContextualAssertionSimplifier", NameTransformationMode::TransformDefinition),
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
        RE * t = ContextualAssertionSimplifier().transformRE(repeated);
        if (t == repeated) return r;
        return makeRep(t, r->getLB(), r->getUB());
    }
    
    RE * transformStart(Start * s) override {
        RE * prior = mContext.priorContext().currentItem();
        if (prior) {
            if (llvm::isa<Start>(prior)) {
                return makeSeq();
            } else if (llvm::isa<CC>(prior)) {
                return makeAlt();
            }
        }
        return s;
    }
    
    RE * transformEnd(End * e) override {
        RE * following = mContext.followingContext().currentItem();
        if (following) {
            if (llvm::isa<End>(following)) {
                return makeSeq();
            } else if (llvm::isa<CC>(following)) {
                return makeAlt();
            }
        }
        return e;
    }
    
    RE * transformAssertion(Assertion * a) override {
        RE * asserted = a->getAsserted();
        Assertion::Kind k = a->getKind();
        Assertion::Sense s = a->getSense();
        RE * t = SimplifyAsserted(mContext, k).transformRE(asserted);
        if (t == asserted) return a;
        return makeAssertion(t, k, s);
    }
    
    private:
        RE_Context mContext;
};
    
    
RE * simplifyAssertions(RE * r) {
    return ContextualAssertionSimplifier().transformRE(r);
}

} // namespace re
