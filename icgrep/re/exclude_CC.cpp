/*
 *  Copyright (c) 2017 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#include <re/exclude_CC.h>
#include <re/re_cc.h>
#include <re/re_name.h>
#include <re/re_start.h>
#include <re/re_end.h>
#include <re/re_any.h>
#include <re/re_seq.h>
#include <re/re_alt.h>
#include <re/re_rep.h>
#include <re/re_group.h>
#include <re/re_range.h>
#include <re/re_diff.h>
#include <re/re_intersect.h>
#include <re/re_assertion.h>
#include <llvm/Support/Casting.h>
#include <llvm/Support/ErrorHandling.h>

using namespace llvm;

namespace re {

bool mayMatchCC(RE * re, CC * cc) {
    if (CC * cc0 = dyn_cast<CC>(re)) {
        return intersects(cc0, cc);
    } else if (Seq * seq = dyn_cast<Seq>(re)) {
        for (auto s : * seq) {
            if (mayMatchCC(s, cc)) return true;
        }
        return false;
    } else if (Alt * alt = dyn_cast<Alt>(re)) {
        for (auto a : * alt) {
            if (mayMatchCC(a, cc)) return true;
        }
        return false;
    } else if (Rep * rep = dyn_cast<Rep>(re)) {
        return mayMatchCC(rep->getRE(), cc);
    } else if (Group * g = dyn_cast<Group>(re)) {
        return mayMatchCC(g->getRE(), cc);
    } else if (Diff * diff = dyn_cast<Diff>(re)) {
        // We only need exclude from the LH operand.
        return mayMatchCC(diff->getLH(), cc);
    } else if (Intersect * e = dyn_cast<Intersect>(re)) {
        // We only need check  one of the operands.
        return mayMatchCC(e->getLH(), cc);
    } else if (isa<Start>(re) || isa<End>(re) || isa<Assertion>(re)) {
        return false;
    } else if (Name * n = dyn_cast<Name>(re)) {
        if (n->getType() ==  Name::Type::ZeroWidth) {
            return false;
        }
        RE * defn = n->getDefinition();
        return mayMatchCC(defn, cc);
    } else {
        report_fatal_error("exclude_CC: unhandled regexp type");
    }
}
 
    
RE * exclude_CC(RE * re, CC * cc) {
    if (!mayMatchCC(re, cc)) return re;
    if (CC * cc0 = dyn_cast<CC>(re)) {
        if (intersects(cc0, cc)) return subtractCC(cc0, cc);
        else return cc0;
    } else if (Seq * seq = dyn_cast<Seq>(re)) {
        std::vector<RE*> list;
        for (auto s : * seq) {
            list.push_back(exclude_CC(s, cc));
        }
        return makeSeq(list.begin(), list.end());
    } else if (Alt * alt = dyn_cast<Alt>(re)) {
        std::vector<RE*> list;
        for (auto a : * alt) {
            list.push_back(exclude_CC(a, cc));
        }
        return makeAlt(list.begin(), list.end());
    } else if (Rep * rep = dyn_cast<Rep>(re)) {
        return makeRep(exclude_CC(rep->getRE(), cc), rep->getLB(), rep->getUB());
    } else if (Group * g = dyn_cast<Group>(re)) {
        return makeGroup(g->getMode(), exclude_CC(g->getRE(), cc), g->getSense());
    } else if (Diff * diff = dyn_cast<Diff>(re)) {
        // We only need exclude from the LH operand.
        return makeDiff(exclude_CC(diff->getLH(), cc), diff->getRH());
    } else if (Intersect * e = dyn_cast<Intersect>(re)) {
        // We only need exclude from the one of the operands.
        return makeIntersect(exclude_CC(e->getLH(), cc), e->getRH());
    } else if (isa<Start>(re) || isa<End>(re) || isa<Assertion>(re)) {
        return re;
    } else if (Name * n = dyn_cast<Name>(re)) {
        switch (n->getType()) {
            case Name::Type::Reference:
            case Name::Type::ZeroWidth:
                return re;
            case Name::Type::Capture:
                return makeCapture(n->getName(), exclude_CC(n->getDefinition(), cc));
            default:
                RE * defn = n->getDefinition();
                if (const CC * cc0 = dyn_cast<CC>(defn)) {
                    if (!intersects(cc0, cc)) return re;
                }
                std::string cc_name = n->getName() + "--" + cc->canonicalName();
                return makeName(cc_name, Name::Type::Unicode, exclude_CC(defn, cc));
                /*
                return exclude_CC(defn, cc);
                */
        }
    } else {
        report_fatal_error("exclude_CC: unhandled regexp type");
    }
}
}

