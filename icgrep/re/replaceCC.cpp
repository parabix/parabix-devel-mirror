#include "replaceCC.h"
#include <cc/alphabet.h>
#include <UCD/unicode_set.h>
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

using namespace llvm;
namespace re {

RE * replaceCC(RE * re, CC * toReplace, RE * replacement) {
    if (CC * cc = dyn_cast<CC>(re)) {
        if ((cc->getAlphabet() == toReplace->getAlphabet()) &&
              (cast<UCD::UnicodeSet>(cc) == cast<UCD::UnicodeSet>(toReplace))) {
            return replacement;
        } 
        return re;
    } else if (Seq * seq = dyn_cast<Seq>(re)) {
        std::vector<RE *> list;
        list.reserve(seq->size());
        for (RE * item : *seq) {
            item = replaceCC(item, toReplace, replacement);
            list.push_back(item);
        }
        re = makeSeq(list.begin(), list.end());
    } else if (Alt * alt = dyn_cast<Alt>(re)) {
        std::vector<RE *> list;
        list.reserve(alt->size());
        for (RE * item : *alt) {
            item = replaceCC(item, toReplace, replacement);
            list.push_back(item);
        }
        re = makeAlt(list.begin(), list.end());
    } else if (Assertion * a = dyn_cast<Assertion>(re)) {
        re = makeAssertion(replaceCC(a->getAsserted(), toReplace, replacement), a->getKind(), a->getSense());
    } else if (Rep * rep = dyn_cast<Rep>(re)) {
        RE * expr = replaceCC(rep->getRE(), toReplace, replacement);
        re = makeRep(expr, rep->getLB(), rep->getUB());
    } else if (Diff * diff = dyn_cast<Diff>(re)) {
        re = makeDiff(replaceCC(diff->getLH(), toReplace, replacement),
                      replaceCC(diff->getRH(), toReplace, replacement));
    } else if (Intersect * e = dyn_cast<Intersect>(re)) {
        re = makeIntersect(replaceCC(e->getLH(), toReplace, replacement),
                           replaceCC(e->getRH(), toReplace, replacement));
    } else if (Group * g = dyn_cast<Group>(re)) {
        re = makeGroup(g->getMode(), replaceCC(g->getRE(), toReplace, replacement), g->getSense());
    }
    return re;
};
}
