#include "re_analysis.h"
#include <re/re_cc.h>
#include <re/re_name.h>
#include <re/re_start.h>
#include <re/re_end.h>
#include <re/re_any.h>
#include <re/re_seq.h>
#include <re/re_alt.h>
#include <re/re_rep.h>
#include <re/re_diff.h>
#include <re/re_intersect.h>
#include <re/re_assertion.h>
#include <re/re_grapheme_boundary.hpp>
#include <iostream>
#include <re/printer_re.h>
#include <limits.h>

namespace re {

bool isByteLength(const RE * re) {
    if (const Alt * alt = dyn_cast<Alt>(re)) {
        for (const RE * re : *alt) {
            if (!isByteLength(re)) {
                return false;
            }
        }
        return true;
    } else if (const Seq * seq = dyn_cast<Seq>(re)) {
        return (seq->size() == 1) && isByteLength(&seq[0]);
    } else if (const Rep * rep = dyn_cast<Rep>(re)) {
        return (rep->getLB() == 1) && (rep->getUB() == 1) && isByteLength(rep->getRE());
    } else if (const Diff * diff = dyn_cast<Diff>(re)) {
        return isByteLength(diff->getLH()) && isByteLength(diff->getRH());
    } else if (const Intersect * e = dyn_cast<Intersect>(re)) {
        return isByteLength(e->getLH()) && isByteLength(e->getRH());
    } else if (const Name * n = dyn_cast<Name>(re)) {
        return (n->getType() == Name::Type::Byte);
    }
    return false; // otherwise
}

bool isUnicodeUnitLength(const RE * re) {
    if (const Alt * alt = dyn_cast<Alt>(re)) {
        for (const RE * re : *alt) {
            if (!isUnicodeUnitLength(re)) {
                return false;
            }
        }
        return true;
    } else if (const Seq * seq = dyn_cast<Seq>(re)) {
        return (seq->size() == 1) && isUnicodeUnitLength(&seq[0]);
    } else if (const Rep * rep = dyn_cast<Rep>(re)) {
        return (rep->getLB() == 1) && (rep->getUB() == 1) && isUnicodeUnitLength(rep->getRE());
    } else if (isa<Assertion>(re)) {
        return false;
    } else if (const Diff * diff = dyn_cast<Diff>(re)) {
        return isUnicodeUnitLength(diff->getLH()) && isUnicodeUnitLength(diff->getRH());
    } else if (const Intersect * e = dyn_cast<Intersect>(re)) {
        return isUnicodeUnitLength(e->getLH()) && isUnicodeUnitLength(e->getRH());
    } else if (isa<Any>(re)) {
        return true;
    } else if (const Name * n = dyn_cast<Name>(re)) {
        // Eventually names might be set up for not unit length items.
        return (n->getType() == Name::Type::Unicode || n->getType() == Name::Type::UnicodeProperty || n->getType() == Name::Type::Byte);
    }
    return false; // otherwise
}

std::pair<int, int> getUnicodeUnitLengthRange(const RE * re) {
    if (const Alt * alt = dyn_cast<Alt>(re)) {
        std::pair<int, int> range = std::make_pair(std::numeric_limits<int>::max(), 0);
        for (const RE * re : *alt) {
            auto r = getUnicodeUnitLengthRange(re);
            range.first = std::min<int>(range.first, r.first);
            range.second = std::max<int>(range.second, r.second);
        }
        return range;
    } else if (const Seq * seq = dyn_cast<Seq>(re)) {
        std::pair<int, int> range = std::make_pair(0, 0);
        for (const RE * re : *seq) {
            auto tmp = getUnicodeUnitLengthRange(re);
            if (LLVM_LIKELY(tmp.first < std::numeric_limits<int>::max() - range.first)) {
                range.first += tmp.first;
            } else {
                range.first = std::numeric_limits<int>::max();
            }
            if (LLVM_LIKELY(tmp.second < std::numeric_limits<int>::max() - range.second)) {
                range.second += tmp.second;
            } else {
                range.second = std::numeric_limits<int>::max();
            }
        }
        return range;
    } else if (const Rep * rep = dyn_cast<Rep>(re)) {
        auto range = getUnicodeUnitLengthRange(rep->getRE());
        if (LLVM_LIKELY(rep->getLB() != Rep::UNBOUNDED_REP && range.first < std::numeric_limits<int>::max())) {
            range.first *= rep->getLB();
        } else {
            range.first = std::numeric_limits<int>::max();
        }
        if (LLVM_LIKELY(rep->getUB() != Rep::UNBOUNDED_REP && range.second < std::numeric_limits<int>::max())) {
            range.second *= rep->getUB();
        } else {
            range.second = std::numeric_limits<int>::max();
        }
        return range;
    } else if (isa<Assertion>(re) || isa<Start>(re) || isa<End>(re)) {
        return std::make_pair(0, 0);
    } else if (const Diff * diff = dyn_cast<Diff>(re)) {
        const auto r1 = getUnicodeUnitLengthRange(diff->getLH());
        const auto r2 = getUnicodeUnitLengthRange(diff->getRH());
        return std::make_pair(std::min(r1.first, r2.first), std::max(r1.second, r2.second));
    } else if (const Intersect * i = dyn_cast<Intersect>(re)) {
        const auto r1 = getUnicodeUnitLengthRange(i->getLH());
        const auto r2 = getUnicodeUnitLengthRange(i->getRH());
        return std::make_pair(std::min(r1.first, r2.first), std::max(r1.second, r2.second));
    } else if (const Name * n = dyn_cast<Name>(re)) {
        // Eventually names might be set up for not unit length items.
        switch (n->getType()) {
            case Name::Type::Byte:
            case Name::Type::Unicode:
            case Name::Type::UnicodeProperty:
                return std::make_pair(1, 1);
            case Name::Type::Unknown:
                return std::make_pair(0, std::numeric_limits<int>::max());
        }
    } else if (const GraphemeBoundary * gp = dyn_cast<GraphemeBoundary>(re)) {
        if (gp->getExpression()) {
            return getUnicodeUnitLengthRange(gp->getExpression());
        }
        return std::make_pair(0, 0);
    }
    return std::make_pair(1, 1);
}
   
int minMatchLength(RE * re) {
    if (Alt * alt = dyn_cast<Alt>(re)) {
        int minAltLength = INT_MAX;
        for (RE * re : *alt) {
            minAltLength = std::min(minAltLength, minMatchLength(re));
        }
        return minAltLength;
    }
    else if (Seq * seq = dyn_cast<Seq>(re)) {
        int minSeqLength = 0;
        for (RE * re : *seq) {
            minSeqLength += minMatchLength(re);
        }
        return minSeqLength;
    }
    else if (Rep * rep = dyn_cast<Rep>(re)) {
        if (rep->getLB() == 0) return 0;
        else return (rep->getLB()) * minMatchLength(rep->getRE());
    }
    else if (isa<Assertion>(re)) {
        return 0;
    }
    else if (Diff * diff = dyn_cast<Diff>(re)) {
        return minMatchLength(diff->getLH());
    }
    else if (Intersect * e = dyn_cast<Intersect>(re)) {
        return std::min(minMatchLength(e->getLH()), minMatchLength(e->getRH()));
    }
    else if (isa<Any>(re)) {
        return 1;
    }
    else if (isa<Name>(re)) {
        return 1;
    }
    return 0; // otherwise
}


}
