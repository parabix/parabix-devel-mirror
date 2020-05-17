#include <re/analysis/re_analysis.h>

#include <limits.h>
#include <llvm/Support/ErrorHandling.h>
#include <llvm/Support/raw_ostream.h>
#include <re/adt/adt.h>
#include <re/adt/printer_re.h>
#include <re/cc/multiplex_CCs.h>
#include <re/analysis/validation.h>
#include <re/transforms/remove_nullable.h>
#include <re/transforms/to_utf8.h>
#include <unicode/core/unicode_set.h>
#include <unicode/utf/UTF.h>

#include <util/small_flat_set.hpp>

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
    } else if (const Capture * c = dyn_cast<Capture>(re)) {
        return matchesEmptyString(c->getCapturedRE());
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

std::pair<int, int> getLengthRange(const RE * re, const cc::Alphabet * indexAlphabet) {
    if (const Alt * alt = dyn_cast<Alt>(re)) {
        std::pair<int, int> range = std::make_pair(INT_MAX, 0);
        for (const RE * a : *alt) {
            auto a_range = getLengthRange(a, indexAlphabet);
            range.first = std::min<int>(range.first, a_range.first);
            range.second = std::max<int>(range.second, a_range.second);
        }
        return range;
    } else if (const Seq * seq = dyn_cast<Seq>(re)) {
        std::pair<int, int> range = std::make_pair(0, 0);
        for (const RE * re : *seq) {
            auto tmp = getLengthRange(re, indexAlphabet);
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
        auto range = getLengthRange(rep->getRE(), indexAlphabet);
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
        return getLengthRange(diff->getLH(), indexAlphabet);
    } else if (const Intersect * i = dyn_cast<Intersect>(re)) {
        const auto r1 = getLengthRange(i->getLH(), indexAlphabet);
        const auto r2 = getLengthRange(i->getRH(), indexAlphabet);
        // The matched string cannot be shorter than the largest of the min lengths
        // nor can it be longer than the smallest of the max lengths.
        return std::make_pair(std::max(r1.first, r2.first), std::min(r1.second, r2.second));
    } else if (const CC * cc = dyn_cast<CC>(re)) {
        auto alphabet = cc->getAlphabet();
        if (const cc::MultiplexedAlphabet * a = dyn_cast<cc::MultiplexedAlphabet>(alphabet)) {
            alphabet = a->getSourceAlphabet();
        }
        if (isa<cc::CodeUnitAlphabet>(alphabet)) return std::make_pair(1, 1);
        if (indexAlphabet == alphabet) return std::make_pair(1, 1);
        if ((indexAlphabet == &cc::UTF16) && (alphabet == &cc::Unicode)) {
            return std::make_pair(UTF<16>::encoded_length(lo_codepoint(cc->front())),
                                  UTF<16>::encoded_length(hi_codepoint(cc->back())));
        }
        return std::make_pair(0, INT_MAX);
    } else if (const Name * n = dyn_cast<Name>(re)) {
        RE * defn = n->getDefinition();
        if (defn) return getLengthRange(defn, indexAlphabet);
        return std::make_pair(0, INT_MAX);
    }
    return std::make_pair(1, 1);
}

bool isFixedLength(const RE * re) {
    if (isa<Alt>(re)) {
        auto range = getLengthRange(re, &cc::Unicode);
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
            default:
                return 0;
        }
    } else if (const Capture * c = dyn_cast<Capture>(re)) {
        return minMatchLength(c->getCapturedRE());
    } else if (const Reference * r = dyn_cast<Reference>(re)) {
        return minMatchLength(r->getCapture());
    }
    return 0; // otherwise
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

struct FixedUTF8Validator : public RE_Validator {
    FixedUTF8Validator() : RE_Validator("FixedUTF8Validator") {}

    bool validateCC(const CC * cc) override {
        auto alphabet = cc->getAlphabet();
        if (const cc::MultiplexedAlphabet * a = dyn_cast<cc::MultiplexedAlphabet>(alphabet)) {
            alphabet = a->getSourceAlphabet();
        }
        if (alphabet == &cc::Unicode) {
            auto min_lgth = UTF<8>::encoded_length(lo_codepoint(cc->front()));
            auto max_lgth = UTF<8>::encoded_length(hi_codepoint(cc->back()));
            return min_lgth == max_lgth;
        }
        return (alphabet == &cc::UTF8) || (alphabet == &cc::Byte);
    }

    bool validateName(const Name * name) override {
        RE * defn = name->getDefinition();
        return (defn != nullptr) && validate(defn);
    }
};

bool validateFixedUTF8(const RE * r) {
    return FixedUTF8Validator().validateRE(r);
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
        SmallFlatSet<const Capture *, 8> available_captures;
        for (const RE * e : *seq) {
            if (const Reference * r = dyn_cast<Reference>(e)) {
                auto capture = r->getCapture();
                if (available_captures.count(cast<Capture>(capture)) == 0) {
                    // Capture is not available.
                    return false;
                } else {
                    continue;
                }
            } else if (const Capture * c = dyn_cast<Capture>(e)) {
                available_captures.emplace(c);
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
        return DefiniteLengthBackReferencesOnly(n->getDefinition());
    } else if (const Capture * c = dyn_cast<Capture>(re)) {
        return DefiniteLengthBackReferencesOnly(c->getCapturedRE());
    } else if (isa<Reference>(re)) {
        return false;
    }
    return true; // otherwise = CC, Any, Start, End, Range
}

}
