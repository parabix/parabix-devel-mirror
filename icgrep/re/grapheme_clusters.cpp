#include "grapheme_clusters.h"
#include <re/re_cc.h>
#include <re/re_name.h>
#include <re/re_alt.h>             // for Alt, makeAlt
#include <re/re_any.h>             // for makeAny, Any
#include <re/re_assertion.h>       // for Assertion, Assertion::Sense, Asser...
#include <re/re_diff.h>            // for Diff, makeDiff
#include <re/re_group.h>
#include <re/re_intersect.h>       // for Intersect
#include <re/re_name.h>            // for Name
#include <re/re_rep.h>             // for Rep, makeRep
#include <re/re_seq.h>             // for Seq, makeSeq
#include <re/re_start.h>
#include <re/re_end.h>
#include <re/re_range.h>
#include <re/printer_re.h>
#include <re/re_name_resolve.h>
#include <vector>                  // for vector, allocator
#include <llvm/Support/Casting.h>  // for dyn_cast, isa
#include <llvm/Support/ErrorHandling.h>
#include <llvm/Support/raw_ostream.h>

/*
 Unicode Technical Standard #18 defines grapheme cluster mode, signified by the (?g) switch.
 The mode is defined in terms of the assertion of grapheme cluster boundary assertions \b{g}
 after every atomic literal.
 
 resolveGraphemeMode transforms a regular expression to perform the required insertion of
 grapheme cluster boundaries, and the elimination of grapheme cluster mode groups.

*/

using namespace llvm;

namespace re {
bool hasGraphemeClusterBoundary(const RE * re) {
    if (isa<CC>(re)) {
        return false;
    } else if (const Name * n = dyn_cast<Name>(re)) {
        if (n->getType() == Name::Type::ZeroWidth) {
            const std::string nameString = n->getName();
            return nameString == "\\b{g}";
        }
        return false;
    } else if (const Alt * alt = dyn_cast<Alt>(re)) {
        for (const RE * re : *alt) {
            if (hasGraphemeClusterBoundary(re)) return true;
        }
        return false;
    } else if (const Seq * seq = dyn_cast<Seq>(re)) {
        for (const RE * re : *seq) {
            if (hasGraphemeClusterBoundary(re)) return true;
        }
        return false;
    } else if (const Rep * rep = dyn_cast<Rep>(re)) {
        return hasGraphemeClusterBoundary(rep->getRE());
    } else if (const Diff * diff = dyn_cast<Diff>(re)) {
        return hasGraphemeClusterBoundary(diff->getLH()) || hasGraphemeClusterBoundary(diff->getRH());
    } else if (const Intersect * e = dyn_cast<Intersect>(re)) {
        return hasGraphemeClusterBoundary(e->getLH()) || hasGraphemeClusterBoundary(e->getRH());
    } else if (isa<Start>(re) || isa<End>(re)) {
        return false;
    } else if (const Assertion * a = dyn_cast<Assertion>(re)) {
        return hasGraphemeClusterBoundary(a->getAsserted());
    } else if (const Group * g = dyn_cast<Group>(re)) {
        if ((g->getMode() == Group::Mode::GraphemeMode) && (g->getSense() == Group::Sense::On)) return true;
        else return hasGraphemeClusterBoundary(g->getRE());
    }
    else llvm_unreachable("Unknown RE type");
}
    
RE * resolveGraphemeMode(RE * re, bool inGraphemeMode) {
    if (isa<Name>(re)) {
        if (inGraphemeMode && (cast<Name>(re)->getName() == ".")) {
            RE * GCB = makeZeroWidth("\\b{g}");
            RE * nonGCB = makeDiff(makeSeq({}), GCB);
            return makeSeq({makeAny(), makeRep(makeSeq({nonGCB, makeAny()}), 0, Rep::UNBOUNDED_REP), GCB});
        }
        else return re;
    }
    else if (isa<CC>(re) || isa<Range>(re)) {
        if (inGraphemeMode) return makeSeq({re, makeZeroWidth("\\b{g}")});
        else return re;
    }
    else if (Seq * seq = dyn_cast<Seq>(re)) {
        std::vector<RE*> list;
        bool afterSingleChar = false;
        for (auto i = seq->begin(); i != seq->end(); ++i) {
            bool atSingleChar = isa<CC>(re) && (cast<CC>(re)->count() == 1);
            if (afterSingleChar && inGraphemeMode && !atSingleChar)
                list.push_back(makeZeroWidth("\\b{g}"));
            if (isa<CC>(re)) list.push_back(*i);
            else {
                list.push_back(resolveGraphemeMode(*i, inGraphemeMode));
            }
            afterSingleChar = atSingleChar;
        }
        if (afterSingleChar && inGraphemeMode) list.push_back(makeZeroWidth("\\b{g}"));
        return makeSeq(list.begin(), list.end());
    } else if (Group * g = dyn_cast<Group>(re)) {
        if (g->getMode() == Group::Mode::GraphemeMode) {
            return resolveGraphemeMode(g->getRE(), g->getSense() == Group::Sense::On);
        }
        else {
            return makeGroup(g->getMode(), resolveGraphemeMode(g->getRE(), inGraphemeMode), g->getSense());
        }
    } else if (Alt * alt = dyn_cast<Alt>(re)) {
        std::vector<RE*> list;
        for (auto i = alt->begin(); i != alt->end(); ++i) {
            list.push_back(resolveGraphemeMode(*i, inGraphemeMode));
        }
        return makeAlt(list.begin(), list.end());
    } else if (Rep * rep = dyn_cast<Rep>(re)) {
        return makeRep(resolveGraphemeMode(rep->getRE(), inGraphemeMode), rep->getLB(), rep->getUB());
    } else if (const Diff * diff = dyn_cast<const Diff>(re)) {
        return makeDiff(resolveGraphemeMode(diff->getLH(), inGraphemeMode),
                        resolveGraphemeMode(diff->getRH(), inGraphemeMode));
    } else if (const Intersect * e = dyn_cast<const Intersect>(re)) {
        return makeIntersect(resolveGraphemeMode(e->getLH(), inGraphemeMode),
                             resolveGraphemeMode(e->getRH(), inGraphemeMode));
    } else if (const Assertion * a = dyn_cast<Assertion>(re)) {
        return makeAssertion(resolveGraphemeMode(a->getAsserted(), inGraphemeMode), a->getKind(), a->getSense());
    } else if (isa<Start>(re) || isa<End>(re)) {
        return re;
    } else llvm_unreachable("Unknown RE type");
}


#define Behind(x) makeLookBehindAssertion(x)
#define Ahead(x) makeLookAheadAssertion(x)

RE * generateGraphemeClusterBoundaryRule() {
    // 3.1.1 Grapheme Cluster Boundary Rules
    
    //    RE * GCB_Control = makeName("gcb", "cn", Name::Type::UnicodeProperty);
    RE * GCB_CR = makeName("gcb", "cr", Name::Type::UnicodeProperty);
    RE * GCB_LF = makeName("gcb", "lf", Name::Type::UnicodeProperty);
    RE * GCB_Control_CR_LF = makeAlt({GCB_CR, GCB_LF});
    
    // Break at the start and end of text.
    RE * GCB_1 = makeStart();
    RE * GCB_2 = makeEnd();
    // Do not break between a CR and LF.
    RE * GCB_3 = makeSeq({Behind(GCB_CR), Ahead(GCB_LF)});
    // Otherwise, break before and after controls.
    RE * GCB_4 = Behind(GCB_Control_CR_LF);
    RE * GCB_5 = Ahead(GCB_Control_CR_LF);
    RE * GCB_1_5 = makeAlt({GCB_1, GCB_2, makeDiff(makeAlt({GCB_4, GCB_5}), GCB_3)});
    
    RE * GCB_L = makeName("gcb", "l", Name::Type::UnicodeProperty);
    RE * GCB_V = makeName("gcb", "v", Name::Type::UnicodeProperty);
    RE * GCB_LV = makeName("gcb", "lv", Name::Type::UnicodeProperty);
    RE * GCB_LVT = makeName("gcb", "lvt", Name::Type::UnicodeProperty);
    RE * GCB_T = makeName("gcb", "t", Name::Type::UnicodeProperty);
    RE * GCB_RI = makeName("gcb", "ri", Name::Type::UnicodeProperty);
    // Do not break Hangul syllable sequences.
    RE * GCB_6 = makeSeq({Behind(GCB_L), Ahead(makeAlt({GCB_L, GCB_V, GCB_LV, GCB_LVT}))});
    RE * GCB_7 = makeSeq({Behind(makeAlt({GCB_LV, GCB_V})), Ahead(makeAlt({GCB_V, GCB_T}))});
    RE * GCB_8 = makeSeq({Behind(makeAlt({GCB_LVT, GCB_T})), Ahead(GCB_T)});
    // Do not break between regional indicator symbols.
    RE * GCB_8a = makeSeq({Behind(GCB_RI), Ahead(GCB_RI)});
    // Do not break before extending characters.
    RE * GCB_9 = Ahead(makeName("gcb", "ex", Name::Type::UnicodeProperty));
    // Do not break before SpacingMarks, or after Prepend characters.
    RE * GCB_9a = Ahead(makeName("gcb", "sm", Name::Type::UnicodeProperty));
    RE * GCB_9b = Behind(makeName("gcb", "pp", Name::Type::UnicodeProperty));
    RE * GCB_6_9b = makeAlt({GCB_6, GCB_7, GCB_8, GCB_8a, GCB_9, GCB_9a, GCB_9b});
    // Otherwise, break everywhere.
    RE * GCB_10 = makeSeq({Behind(makeAny()), Ahead(makeAny())});
    
    //Name * gcb = makeName("gcb", Name::Type::UnicodeProperty);
    RE * gcb = makeAlt({GCB_1_5, makeDiff(GCB_10, GCB_6_9b)});
    return gcb;
}

}
