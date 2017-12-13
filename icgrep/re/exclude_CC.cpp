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

RE * exclude_CC(RE * re, CC * cc) {
    if (CC * cc0 = dyn_cast<CC>(re)) {
        return subtractCC(cc0, cc);
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
            case Name::Type::Byte:
            case Name::Type::Unicode:
            case Name::Type::UnicodeProperty:
                return exclude_CC(n->getDefinition(), cc);
            case Name::Type::ZeroWidth:
                return re;
            case Name::Type::Capture:
                return makeCapture(n->getName(), exclude_CC(n->getDefinition(), cc));
            case Name::Type::Reference:
                return re;
            default:
                report_fatal_error("exclude_CC: unhandled Name type");
        }
    } else {
        report_fatal_error("exclude_CC: unhandled regexp type");
    }
}
}

