#include "re_analysis.h"
#include <UCD/unicode_set.h>
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
#include <re/re_group.h>
#include <re/re_nullable.h>
#include <re/to_utf8.h>
#include <re/printer_re.h>
#include <cc/alphabet.h>
#include <cc/multiplex_CCs.h>
#include <limits.h>
#include <llvm/Support/ErrorHandling.h>
#include <llvm/Support/raw_ostream.h>

using namespace llvm;

namespace re {
    
bool matchesEmptyString(const RE * re) {
    if (const Alt * alt = dyn_cast<Alt>(re)) {
        for (const RE * re : *alt) {
            if (matchesEmptyString(re)) {
                return true;
            }
        }
        return false;
    } else if (const Seq * seq = dyn_cast<Seq>(re)) {
        for (const RE * re : *seq) {
            if (!matchesEmptyString(re)) {
                return false;
            }
        }
        return true;
    } else if (const Rep * rep = dyn_cast<Rep>(re)) {
        return (rep->getLB() == 0) || matchesEmptyString(rep->getRE());
    } else if (isa<Start>(re)) {
        return true;
    } else if (isa<End>(re)) {
        return true;
    } else if (isa<Assertion>(re)) {
        return false;
    } else if (const Diff * diff = dyn_cast<Diff>(re)) {
        return matchesEmptyString(diff->getLH()) && !matchesEmptyString(diff->getRH());
    } else if (const Intersect * e = dyn_cast<Intersect>(re)) {
        return matchesEmptyString(e->getLH()) && matchesEmptyString(e->getRH());
    } else if (isa<Any>(re)) {
        return false;
    } else if (isa<CC>(re)) {
        return false;
    } else if (const Group * g = dyn_cast<Group>(re)) {
        return matchesEmptyString(g->getRE());
    } else if (const Name * n = dyn_cast<Name>(re)) {
        return matchesEmptyString(n->getDefinition());
    }
    return false; // otherwise
}

const CC* matchableCodepoints(const RE * re) {
    if (const CC * cc = dyn_cast<CC>(re)) {
        return cc;
    } else if (const Alt * alt = dyn_cast<Alt>(re)) {
        CC * matchable = makeCC();
        for (const RE * re : *alt) {
            matchable = makeCC(matchable, matchableCodepoints(re));
        }
        return matchable;
    } else if (const Seq * seq = dyn_cast<Seq>(re)) {
        CC * matchable = makeCC();
        bool pastCC = false;
        for (const RE * re : *seq) {
            if (pastCC) {
                if (!(isa<End>(re) || matchesEmptyString(re))) return makeCC();
            }
            else if (isa<End>(re)) return makeCC();
            else {
                matchable = makeCC(matchable, matchableCodepoints(re));
                pastCC = !matchesEmptyString(re);
            }
        }
        return matchable;
    } else if (const Rep * rep = dyn_cast<Rep>(re)) {
        if ((rep->getLB() <= 1) || matchesEmptyString(rep->getRE())) {
            return matchableCodepoints(rep->getRE());
        }
        else return makeCC();
    } else if (const Diff * diff = dyn_cast<Diff>(re)) {
        return subtractCC(matchableCodepoints(diff->getLH()), matchableCodepoints(diff->getRH()));
    } else if (const Intersect * e = dyn_cast<Intersect>(re)) {
        return intersectCC(matchableCodepoints(e->getLH()), matchableCodepoints(e->getRH()));
    } else if (isa<Any>(re)) {
        return makeCC(0, 0x10FFFF);
    } else if (const Name * n = dyn_cast<Name>(re)) {
        return matchableCodepoints(n->getDefinition());
    }
    return makeCC(); // otherwise = Start, End, Assertion
}



bool isByteLength(const RE * re) {
    if (const Alt * alt = dyn_cast<Alt>(re)) {
        for (const RE * re : *alt) {
            if (!isByteLength(re)) {
                return false;
            }
        }
        return true;
    } else if (const Seq * seq = dyn_cast<Seq>(re)) {
        bool byteLengthSeen = false;
        for (const RE * e : *seq) {
            if (isa<Assertion>(e)) continue;
            else if (byteLengthSeen) return false;
            else if (isByteLength(e)) byteLengthSeen = true;
            else return false;
        }
        return byteLengthSeen;
    } else if (const Rep * rep = dyn_cast<Rep>(re)) {
        return (rep->getLB() == 1) && (rep->getUB() == 1) && isByteLength(rep->getRE());
    } else if (const Diff * diff = dyn_cast<Diff>(re)) {
        return isByteLength(diff->getLH()) && isByteLength(diff->getRH());
    } else if (const Intersect * e = dyn_cast<Intersect>(re)) {
        return isByteLength(e->getLH()) && isByteLength(e->getRH());
    } else if (const CC * cc = dyn_cast<CC>(re)) {
        const cc::Alphabet * a = cc->getAlphabet();
        if (a == &cc::Unicode) {
            return (cc->max_codepoint() <= 0x7F);
        } else if (a == &cc::Byte) {
            return true;
        } else if (isa<cc::MultiplexedAlphabet>(a)) {
            const cc::Alphabet * srcA = cast<cc::MultiplexedAlphabet>(a)->getSourceAlphabet();
            if (srcA == &cc::Byte) {
                return true;
//            } else if (srcA == &cc::Unicode) {
//                return cast<cc::MultiplexedAlphabet>(a)->invertCC(cc)->max_codepoint() <= 0x7F;
            } else return (a == &cc::Byte);
        }
        return false;
    } else if (const Name * n = dyn_cast<Name>(re)) {
        if (n->getType() == Name::Type::ZeroWidth) {
            return false;
        }
        return isByteLength(n->getDefinition());
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
        bool unitLengthSeen = false;
        for (const RE * e : *seq) {
            if (isa<Assertion>(e)) continue;
            else if (unitLengthSeen) return false;
            else if (isUnicodeUnitLength(e)) unitLengthSeen = true;
            else return false;
        }
        return unitLengthSeen;
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
    } else if (isa<CC>(re)) {
        return true;
    } else if (const Name * n = dyn_cast<Name>(re)) {
        // Eventually names might be set up for not unit length items.
        if (n->getType() == Name::Type::Unicode || n->getType() == Name::Type::UnicodeProperty) {
            return true;
        } else if (n->getType() == Name::Type::ZeroWidth) {
            return false;
        }
        return isUnicodeUnitLength(n->getDefinition());
    }
    return false; // otherwise
}

std::pair<int, int> getUnicodeUnitLengthRange(const RE * re) {
    if (const Alt * alt = dyn_cast<Alt>(re)) {
        std::pair<int, int> range = std::make_pair(INT_MAX, 0);
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
            if (LLVM_LIKELY(tmp.first < (INT_MAX - range.first))) {
                range.first += tmp.first;
            } else {
                range.first = INT_MAX;
            }
            if (LLVM_LIKELY(tmp.second < (INT_MAX - range.second))) {
                range.second += tmp.second;
            } else {
                range.second = INT_MAX;
            }
        }
        return range;
    } else if (const Rep * rep = dyn_cast<Rep>(re)) {
        auto range = getUnicodeUnitLengthRange(rep->getRE());
        if (LLVM_LIKELY(rep->getLB() != Rep::UNBOUNDED_REP && range.first < INT_MAX)) {
            range.first *= rep->getLB();
        } else {
            range.first = INT_MAX;
        }
        if (LLVM_LIKELY(rep->getUB() != Rep::UNBOUNDED_REP && range.second < INT_MAX)) {
            range.second *= rep->getUB();
        } else {
            range.second = INT_MAX;
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
    } else if (isa<CC>(re)) {
        return std::make_pair(1, 1);
    } else if (const Name * n = dyn_cast<Name>(re)) {
        // Eventually names might be set up for not unit length items.
        switch (n->getType()) {
            case Name::Type::Unicode:
            case Name::Type::UnicodeProperty:
                return std::make_pair(1, 1);
            case Name::Type::Capture:
            case Name::Type::Reference:
                return getUnicodeUnitLengthRange(n->getDefinition());
            case Name::Type::ZeroWidth:
                return std::make_pair(0, 0);
            case Name::Type::Unknown:
                return std::make_pair(0, INT_MAX);
        }
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
    } else if (Seq * seq = dyn_cast<Seq>(re)) {
        int minSeqLength = 0;
        for (RE * re : *seq) {
            minSeqLength += minMatchLength(re);
        }
        return minSeqLength;
    } else if (Rep * rep = dyn_cast<Rep>(re)) {
        if (rep->getLB() == 0) return 0;
        else return (rep->getLB()) * minMatchLength(rep->getRE());
    } else if (isa<Assertion>(re)) {
        return 0;
    } else if (Diff * diff = dyn_cast<Diff>(re)) {
        return minMatchLength(diff->getLH());
    } else if (Intersect * e = dyn_cast<Intersect>(re)) {
        return std::min(minMatchLength(e->getLH()), minMatchLength(e->getRH()));
    } else if (isa<Any>(re)) {
        return 1;
    } else if (isa<CC>(re)) {
        return 1;
    } else if (const Name * n = dyn_cast<Name>(re)) {
        // Eventually names might be set up for not unit length items.
        switch (n->getType()) {
            case Name::Type::Unicode:
            case Name::Type::UnicodeProperty:
                return 1;
            case Name::Type::Capture:
            case Name::Type::Reference:
                return minMatchLength(n->getDefinition());
            default:
                return 0;
        }
    }
    return 0; // otherwise
}
    
//If a regular expression contains unit and not byteLength bounded repetition type, we select a different pipeline to utilize the log2 technique.
bool unitBoundedRep(const RE * re) {
    if (const Alt * alt = dyn_cast<Alt>(re)) {
        for (const RE * re : *alt) {
            if (unitBoundedRep(re)) {
                return true;
            }
        }
        return false;
    } else if (const Seq * seq = dyn_cast<Seq>(re)) {
	for (const RE * re : *seq) {
	    if (unitBoundedRep(re)) {
		return true;
	    }
	}
        return false;
    } else if (const Rep * rep = dyn_cast<Rep>(re)) {
	if (rep->getLB() == 0 && rep->getUB() == Rep::UNBOUNDED_REP) {
	    return false;
	} else {
	    return (!isByteLength(rep->getRE()) && isUnicodeUnitLength(rep->getRE()));
	}
    } else if (const Diff * diff = dyn_cast<Diff>(re)) {
        return unitBoundedRep(diff->getLH()) || unitBoundedRep(diff->getRH());
    } else if (const Intersect * e = dyn_cast<Intersect>(re)) {
        return unitBoundedRep(e->getLH()) || unitBoundedRep(e->getRH());
    } else if (const Name * n = dyn_cast<Name>(re)) {
	if (n->getType() == Name::Type::Capture || n->getType() == Name::Type::Reference) {
            return unitBoundedRep(n->getDefinition());
        }
        return false;
    }
    return false; // otherwise
  
}

//Cases that not include bounded repetition, assertion, start and end type can suit for local language compile pipeline. 
bool isTypeForLocal(const RE * re) {
    if (const Name * n = dyn_cast<Name>(re)) {
        return isTypeForLocal(n->getDefinition());
    } else if (const Alt * alt = dyn_cast<Alt>(re)) {
        for (const RE * re : *alt) {
            if (!isTypeForLocal(re)) {
                return false;
            }
        }
        return true;
    } else if (const Seq * seq = dyn_cast<Seq>(re)) {
        if (seq->empty()) return false;
        for (const RE * re : *seq) {
            if (!isTypeForLocal(re)) {
                return false;
            }
        }
        return true;
    } else if (const Rep * rep = dyn_cast<Rep>(re)) {
        if (rep->getLB() != 0 || rep->getUB() != Rep::UNBOUNDED_REP) {
            return false;
        }
        return true;
    } else if (const Diff * diff = dyn_cast<Diff>(re)) {
        return isTypeForLocal(diff->getLH()) && isTypeForLocal(diff->getRH());
    } else if (const Intersect * e = dyn_cast<Intersect>(re)) {
        return isTypeForLocal(e->getLH()) && isTypeForLocal(e->getRH());
    } else if (isa<Start>(re) || isa<End>(re) || isa<Assertion>(re)) {
        return false;
    }
    return true; // otherwise
}

bool hasAssertion(const RE * re) {
    if (isa<CC>(re)) {
        return false;
    } else if (const Name * n = dyn_cast<Name>(re)) {
        return hasAssertion(n->getDefinition());
    } else if (const Alt * alt = dyn_cast<Alt>(re)) {
        for (const RE * re : *alt) {
            if (hasAssertion(re)) return true;
        }
        return false;
    } else if (const Seq * seq = dyn_cast<Seq>(re)) {
        for (const RE * re : *seq) {
            if (hasAssertion(re)) return true;
        }
        return false;
    } else if (const Rep * rep = dyn_cast<Rep>(re)) {
        return hasAssertion(rep->getRE());
    } else if (const Diff * diff = dyn_cast<Diff>(re)) {
        return hasAssertion(diff->getLH()) || hasAssertion(diff->getRH());
    } else if (const Intersect * e = dyn_cast<Intersect>(re)) {
        return hasAssertion(e->getLH()) || hasAssertion(e->getRH());
    } else if (isa<Start>(re) || isa<End>(re) || isa<Assertion>(re)) {
        return true;
    } else if (const Group * g = dyn_cast<Group>(re)) {
        if ((g->getMode() == Group::Mode::GraphemeMode) && (g->getSense() == Group::Sense::On)) {
            return true;
        }
        else {
            return hasAssertion(g->getRE());
        }
    }
    else llvm_unreachable("Unknown RE type");
}

struct ByteTestComplexity {
    
    void gatherTests(RE * re);
    
    UCD::UnicodeSet equalityTests;
    UCD::UnicodeSet lessThanTests;
    unsigned testCount;
    unsigned testLimit;
};

void ByteTestComplexity::gatherTests(RE * re) {
    if (CC * cc = dyn_cast<CC>(re)) {
        if (cc->getAlphabet() == &cc::Unicode) {
            gatherTests(toUTF8(re));
        } else {
            for (const auto range : *cc) {
                const auto lo = re::lo_codepoint(range);
                const auto hi = re::hi_codepoint(range);
                if (lo == hi) {
                    if (!equalityTests.contains(lo)) {
                        equalityTests.insert(lo);
                        testCount++;
                    }
                } else {
                    if (lo > 0) {
                        if (!lessThanTests.contains(lo)) {
                            lessThanTests.insert(lo);
                            testCount++;
                        }
                    }
                    if (hi < 0xFF) {
                        if (!lessThanTests.contains(hi+1)) {
                            lessThanTests.insert(hi+1);
                            testCount++;
                        }
                    }
                }
                if (testCount > testLimit) return;
            }
        }
    } else if (const Name * n = dyn_cast<Name>(re)) {
        gatherTests(n->getDefinition());
    } else if (Alt * alt = dyn_cast<Alt>(re)) {
        for (RE * item : *alt) {
            gatherTests(item);
        }
    } else if (Seq * seq = dyn_cast<Seq>(re)) {
        for (RE * item : *seq) {
            gatherTests(item);
        }
    } else if (Assertion * a = dyn_cast<Assertion>(re)) {
        gatherTests(a->getAsserted());
    } else if (Rep * rep = dyn_cast<Rep>(re)) {
        gatherTests(rep->getRE());
    } else if (Diff * diff = dyn_cast<Diff>(re)) {
        gatherTests(diff->getLH());
        gatherTests(diff->getRH());
    } else if (Intersect * e = dyn_cast<Intersect>(re)) {
        gatherTests(e->getLH());
        gatherTests(e->getRH());
    } else if (Group * g = dyn_cast<Group>(re)) {
        gatherTests(g->getRE());
    }
}

bool byteTestsWithinLimit(RE * re, unsigned limit) {
    ByteTestComplexity btc_object;
    btc_object.testCount = 0;
    btc_object.testLimit = limit;
    btc_object.gatherTests(re);
    return btc_object.testCount <= btc_object.testLimit;
}


void UndefinedNameError(const Name * n) {
    report_fatal_error("Error: Undefined name in regular expression: \"" + n->getName() + "\".");
}
}
