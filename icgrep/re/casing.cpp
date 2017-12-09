#include <re/casing.h>
#include <re/re_cc.h>
#include <re/re_alt.h>             // for Alt, makeAlt
#include <re/re_any.h>             // for makeAny, Any
#include <re/re_assertion.h>       // for Assertion, Assertion::Sense, Asser...
#include <re/re_start.h>
#include <re/re_end.h>
#include <re/re_diff.h>            // for Diff, makeDiff
#include <re/re_group.h>
#include <re/re_intersect.h>       // for Intersect
#include <re/re_name.h>            // for Name
#include <re/re_rep.h>             // for Rep, makeRep
#include <re/re_seq.h>             // for Seq, makeSeq
#include <vector>                  // for vector, allocator
#include <llvm/Support/Casting.h>  // for dyn_cast, isa
#include <llvm/Support/ErrorHandling.h>


using namespace llvm;

namespace re {
RE * resolveCaseInsensitiveMode(RE * re, bool inCaseInsensitiveMode) {
    if (isa<CC>(re)) {
        if (inCaseInsensitiveMode) return caseInsensitize(cast<CC>(re));
        else return re;
    }
    else if (Name * name = dyn_cast<Name>(re)) {
        if (!inCaseInsensitiveMode || (name->getDefinition() == nullptr)) return re;
        RE * r = resolveCaseInsensitiveMode(name->getDefinition(), true);
        Name * n = makeName(name->getNamespace(), name->getName() + "/i", name->getType());
        n->setDefinition(r);
        return n;
    }
    else if (Seq * seq = dyn_cast<Seq>(re)) {
        std::vector<RE*> list;
        for (auto i = seq->begin(); i != seq->end(); ++i) {
            list.push_back(resolveCaseInsensitiveMode(*i, inCaseInsensitiveMode));
        }
        return makeSeq(list.begin(), list.end());
    } else if (Group * g = dyn_cast<Group>(re)) {
        if (g->getMode() == Group::Mode::CaseInsensitiveMode) {
            return resolveCaseInsensitiveMode(g->getRE(), g->getSense() == Group::Sense::On);
        }
        else {
            return makeGroup(g->getMode(), resolveCaseInsensitiveMode(g->getRE(), inCaseInsensitiveMode), g->getSense());
        }
    } else if (Alt * alt = dyn_cast<Alt>(re)) {
        std::vector<RE*> list;
        for (auto i = alt->begin(); i != alt->end(); ++i) {
            list.push_back(resolveCaseInsensitiveMode(*i, inCaseInsensitiveMode));
        }
        return makeAlt(list.begin(), list.end());
    } else if (Rep * rep = dyn_cast<Rep>(re)) {
        return makeRep(resolveCaseInsensitiveMode(rep->getRE(), inCaseInsensitiveMode), rep->getLB(), rep->getUB());
    } else if (const Diff * diff = dyn_cast<const Diff>(re)) {
        return makeDiff(resolveCaseInsensitiveMode(diff->getLH(), inCaseInsensitiveMode),
                        resolveCaseInsensitiveMode(diff->getRH(), inCaseInsensitiveMode));
    } else if (const Intersect * e = dyn_cast<const Intersect>(re)) {
        return makeIntersect(resolveCaseInsensitiveMode(e->getLH(), inCaseInsensitiveMode),
                             resolveCaseInsensitiveMode(e->getRH(), inCaseInsensitiveMode));
    } else if (const Assertion * a = dyn_cast<Assertion>(re)) {
        return makeAssertion(resolveCaseInsensitiveMode(a->getAsserted(), inCaseInsensitiveMode), a->getKind(), a->getSense());
    } else if (isa<Start>(re) || isa<End>(re)) {
        return re;
    } else llvm_unreachable("Unknown RE type");
}

}
