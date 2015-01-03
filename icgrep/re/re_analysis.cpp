#include "re_analysis.h"
#include "re_cc.h"
#include "re_name.h"
#include "re_start.h"
#include "re_end.h"
#include "re_any.h"
#include "re_seq.h"
#include "re_alt.h"
#include "re_rep.h"
#include "re_diff.h"
#include "re_intersect.h"

namespace re {

bool isByteLength(RE * re) {
    if (Alt * alt = dyn_cast<Alt>(re)) {
        std::vector<RE*> list;
        for (RE * re : *alt) {
            if (!isByteLength(re)) return false;
        }
        return true;
    }
    else if (Seq * seq = dyn_cast<Seq>(re)) {
        return (seq->size() == 1) && isByteLength(&seq[0]);
    }
    else if (Rep * rep = dyn_cast<Rep>(re)) {
        return (rep->getLB() == 1) && (rep->getUB() == 1) && isByteLength(rep->getRE());
    }
    else if (Diff * diff = dyn_cast<Diff>(re)) {
        return isByteLength(diff->getLH()) && isByteLength(diff->getRH());
    }
    else if (Intersect * e = dyn_cast<Intersect>(re)) {
        return isByteLength(e->getLH()) && isByteLength(e->getRH());
    }
    else if (isa<Any>(re)) return false;
    else if (Name * n = dyn_cast<Name>(re)) {
        return (n->getType() == Name::Type::Byte);
    }
    return false; // otherwise
}

bool isUnicodeUnitLength(RE * re) {
    if (Alt * alt = dyn_cast<Alt>(re)) {
        std::vector<RE*> list;
        for (RE * re : *alt) {
            if (!isUnicodeUnitLength(re)) return false;
        }
        return true;
    }
    else if (Seq * seq = dyn_cast<Seq>(re)) {
        return (seq->size() == 1) && isUnicodeUnitLength(&seq[0]);
    }
    else if (Rep * rep = dyn_cast<Rep>(re)) {
        return (rep->getLB() == 1) && (rep->getUB() == 1) && isUnicodeUnitLength(rep->getRE());
    }
    else if (Diff * diff = dyn_cast<Diff>(re)) {
        return isUnicodeUnitLength(diff->getLH()) && isUnicodeUnitLength(diff->getRH());
    }
    else if (Intersect * e = dyn_cast<Intersect>(re)) {
        return isUnicodeUnitLength(e->getLH()) && isUnicodeUnitLength(e->getRH());
    }
    else if (isa<Any>(re)) return true;
    else if (Name * n = dyn_cast<Name>(re)) {
        // Eventually names might be set up for not unit length items.
        return (n->getType() == Name::Type::Unicode || n->getType() == Name::Type::UnicodeProperty || n->getType() == Name::Type::Byte);
    }
    return false; // otherwise
}
    
}
