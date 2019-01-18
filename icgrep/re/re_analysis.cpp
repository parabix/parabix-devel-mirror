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
#include <set>
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

bool isRequireNonFinal(const RE * re, bool checkByteLength) {
    if (checkByteLength) {
        bool allCcByteLength = isAllCcByteLength(re);
        if (allCcByteLength) {
            return false;
        }
    }

    if (const Alt * alt = dyn_cast<Alt>(re)) {
        for (unsigned i = 0; i < alt->size(); i++) {
            if (isRequireNonFinal((*alt)[i], false)) {
                return true;
            }
        }
        return false;
    } else if (const Seq * seq = dyn_cast<Seq>(re)) {
        if (seq->size() == 0) {
            return false;
        } else if (seq->size() == 1) {
            return isRequireNonFinal((*seq)[0], false);
        }
    } else if (dyn_cast<Name>(re)) {
        return false;
    } else if (dyn_cast<CC>(re)) {
        return false;
    }
    return true;
}

bool isAllCcByteLength(const RE * re) {
    if (const Alt * alt = dyn_cast<Alt>(re)) {
        for (const RE * re : *alt) {
            if (!isAllCcByteLength(re)) {
                return false;
            }
        }
        return true;
    } else if (const Seq * seq = dyn_cast<Seq>(re)) {
        for (const RE * e : *seq) {
            if (!isAllCcByteLength(e)) return false;
        }
        return true;
    } else if (const Rep * rep = dyn_cast<Rep>(re)) {
        return isAllCcByteLength(rep->getRE());
    }  else if (const Name * n = dyn_cast<Name>(re)) {
        if (n->getType() == Name::Type::ZeroWidth) {
            return false;
        }
        return isAllCcByteLength(n->getDefinition());
    } else {
        return isByteLength(re);
    }
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
        if (cc->empty()) return false;
        const cc::Alphabet * a = cc->getAlphabet();
        if (a == &cc::Unicode) {
            return (cc->max_codepoint() <= 0x7F);
        } else if (a == &cc::Byte) {
            return true;
        } else if (isa<cc::MultiplexedAlphabet>(a)) {
            const cc::Alphabet * srcA = cast<cc::MultiplexedAlphabet>(a)->getSourceAlphabet();
            if (srcA == &cc::Byte) {
                return true;
            } else if (cc->sourceCC) {
                return isByteLength(cc->sourceCC);
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
    } else if (const CC * cc = dyn_cast<CC>(re)) {
        return !(cc->empty());
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
        // The range is determined by the first operand only.
        return getUnicodeUnitLengthRange(diff->getLH());
    } else if (const Intersect * i = dyn_cast<Intersect>(re)) {
        const auto r1 = getUnicodeUnitLengthRange(i->getLH());
        const auto r2 = getUnicodeUnitLengthRange(i->getRH());
        // The matched string cannot be shorter than the largest of the min lengths
        // nor can it be longer than the smallest of the max lengths.
        return std::make_pair(std::max(r1.first, r2.first), std::min(r1.second, r2.second));
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
    
bool isFixedLength(const RE * re) {
    if (isa<Alt>(re)) {
        auto range = getUnicodeUnitLengthRange(re);
        return range.first == range.second;
    } else if (const Seq * seq = dyn_cast<Seq>(re)) {
        for (const RE * e : *seq) {
            if (!isFixedLength(e)) return false;
        }
        return true;
    } else if (const Rep * rep = dyn_cast<Rep>(re)) {
        return (rep->getLB() == rep->getUB()) && isFixedLength(rep->getRE());
    } else if (const Diff * diff = dyn_cast<Diff>(re)) {
        return isFixedLength(diff->getLH());
    } else if (const Intersect * e = dyn_cast<Intersect>(re)) {
        return isFixedLength(e->getLH()) || isFixedLength(e->getRH());
    } else if (const Group * g = dyn_cast<Group>(re)) {
        return isFixedLength(g->getRE());
    } else if (const Name * n = dyn_cast<Name>(re)) {
        return isFixedLength(n->getDefinition());
    }
    return true; // otherwise = CC, Any, Start, End, Range, Assertion
}

   
int minMatchLength(const RE * re) {
    if (const Alt * alt = dyn_cast<Alt>(re)) {
        int minAltLength = INT_MAX;
        for (RE * re : *alt) {
            minAltLength = std::min(minAltLength, minMatchLength(re));
        }
        return minAltLength;
    } else if (const Seq * seq = dyn_cast<Seq>(re)) {
        int minSeqLength = 0;
        for (RE * re : *seq) {
            minSeqLength += minMatchLength(re);
        }
        return minSeqLength;
    } else if (const Rep * rep = dyn_cast<Rep>(re)) {
        if (rep->getLB() == 0) return 0;
        else return (rep->getLB()) * minMatchLength(rep->getRE());
    } else if (isa<Assertion>(re)) {
        return 0;
    } else if (const Diff * diff = dyn_cast<Diff>(re)) {
        return minMatchLength(diff->getLH());
    } else if (const Intersect * e = dyn_cast<Intersect>(re)) {
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
    if (const CC * cc = dyn_cast<CC>(re)) {
        if (cc->getAlphabet() == &cc::Unicode) {
            gatherTests(UTF8_Transformer().transformRE(re));
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
    } else if (const Alt * alt = dyn_cast<Alt>(re)) {
        for (RE * item : *alt) {
            gatherTests(item);
        }
    } else if (const Seq * seq = dyn_cast<Seq>(re)) {
        for (RE * item : *seq) {
            gatherTests(item);
        }
    } else if (const Assertion * a = dyn_cast<Assertion>(re)) {
        gatherTests(a->getAsserted());
    } else if (const Rep * rep = dyn_cast<Rep>(re)) {
        gatherTests(rep->getRE());
    } else if (const Diff * diff = dyn_cast<Diff>(re)) {
        gatherTests(diff->getLH());
        gatherTests(diff->getRH());
    } else if (const Intersect * e = dyn_cast<Intersect>(re)) {
        gatherTests(e->getLH());
        gatherTests(e->getRH());
    } else if (const Group * g = dyn_cast<Group>(re)) {
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

bool hasTriCCwithinLimit(RE * r, unsigned byteCClimit, RE * & prefixRE, RE * & suffixRE) {
    if (const Seq * seq = dyn_cast<Seq>(r)) {
        if (seq->size() < 4) return false;
        if (!isa<CC>(seq->front())) return false;
        if (!isa<CC>((*seq)[1])) return false;
        if (!isa<CC>((*seq)[2])) return false;
        prefixRE = makeSeq(seq->begin(), seq->begin()+3);
        if (byteTestsWithinLimit(prefixRE, byteCClimit)) {
            suffixRE = makeSeq(seq->begin()+3, seq->end());
            return true;
        }
        return false;
    }
    return false;
}

    
bool hasEndAnchor(const RE * re) {
    if (const Alt * alt = dyn_cast<Alt>(re)) {
        for (const RE * re : *alt) {
            if (!hasEndAnchor(re)) {
                return false;
            }
        }
        return true;
    } else if (const Seq * seq = dyn_cast<Seq>(re)) {
        return (!seq->empty()) && isa<End>(seq->back());
    } else if (const Rep * rep = dyn_cast<Rep>(re)) {
        return hasEndAnchor(rep->getRE());
    } else if (const Diff * diff = dyn_cast<Diff>(re)) {
        return hasEndAnchor(diff->getLH()) && !hasEndAnchor(diff->getRH());
    } else if (const Intersect * e = dyn_cast<Intersect>(re)) {
        return hasEndAnchor(e->getLH()) && hasEndAnchor(e->getRH());
    } else if (isa<End>(re)) {
        return true;
    }
    return false; // otherwise
}
    
//
//  Back Reference Analysis
//
//  The definite-length back-reference implementation strategy requires that
//  each capture that has a back-reference be fixed in length and that
//  that each back-reference is a fixed length from its corresponding capture.
//
//   In analyzing a sequences of regular expression elements
//   e_0, e_1, ..., e_i, ..., e_j, ..., e_n
//   we say that e_j is within fixed-length range of e_i if
//   for all k: i < k < j, e_k is fixed length.
//

bool DefiniteLengthBackReferencesOnly(const RE * re) {
    if (const Alt * alt = dyn_cast<Alt>(re)) {
        for (const RE * a : *alt) {
            if (!DefiniteLengthBackReferencesOnly(a)) return false;
        }
        return true;
    } else if (const Seq * seq = dyn_cast<Seq>(re)) {
        // As we iterate through sequence elements, we keep track of captures
        // that are encountered and are still reachable through a series of
        // fixed length elements back from the current position.  As soon as
        // a variable length element is encounterd, the list of available_captures
        // is cleared.
        //
        std::set<const Name *> available_captures;
        for (const RE * e : *seq) {
            if (const Name * n = dyn_cast<Name>(e)) {
                auto T = n->getType();
                auto defn = n->getDefinition();
                if (T == Name::Type::Reference) {
                    if (available_captures.find(cast<Name>(defn)) == available_captures.end()) {
                        // Capture is not available.
                        return false;
                    } else {
                        continue;
                    }
                } else {
                    if (!DefiniteLengthBackReferencesOnly(defn)) return false;
                    if (isFixedLength(defn)) {
                        if (T == Name::Type::Capture) {
                            available_captures.emplace(n);
                        }
                    } else {
                        available_captures.clear();
                    }
                }
            }
            if (!DefiniteLengthBackReferencesOnly(e)) return false;
            if (!isFixedLength(e)) available_captures.clear();
        }
        return true;
    } else if (const Rep * rep = dyn_cast<Rep>(re)) {
        return DefiniteLengthBackReferencesOnly(rep->getRE());
    } else if (const Diff * diff = dyn_cast<Diff>(re)) {
        return DefiniteLengthBackReferencesOnly(diff->getLH()) && DefiniteLengthBackReferencesOnly(diff->getRH());
    } else if (const Intersect * e = dyn_cast<Intersect>(re)) {
        return DefiniteLengthBackReferencesOnly(e->getLH()) && DefiniteLengthBackReferencesOnly(e->getRH());
    } else if (const Assertion * a = dyn_cast<Assertion>(re)) {
        return DefiniteLengthBackReferencesOnly(a->getAsserted());
    } else if (const Group * g = dyn_cast<Group>(re)) {
        return DefiniteLengthBackReferencesOnly(g->getRE());
    } else if (const Name * n = dyn_cast<Name>(re)) {
        if (n->getType() == Name::Type::Reference) return false;
        return DefiniteLengthBackReferencesOnly(n->getDefinition());
    }
    return true; // otherwise = CC, Any, Start, End, Range
}

void UndefinedNameError(const Name * n) {
    report_fatal_error("Error: Undefined name in regular expression: \"" + n->getName() + "\".");
}
}
