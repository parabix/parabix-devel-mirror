#include <re/transforms/re_minimizer.h>

#include <re/adt/adt.h>
#include <re/transforms/re_memoizing_transformer.h>
#include <boost/container/flat_set.hpp>
#include <boost/container/small_vector.hpp>

#include <llvm/ADT/SmallVector.h>

using namespace llvm;
using namespace boost::container;

template <typename T, unsigned N>
using small_flat_set = flat_set<T, std::less<T>, small_vector<T, N>>;

namespace re {

using Set = small_flat_set<RE *, 16>;
using List = std::vector<RE *>;
using Combiner = SmallVector<RE *, 16>;

struct RE_Minimizer final : public RE_MemoizingTransformer {

    RE * transformAlt(Alt * alt) override {
        Set set;
        Combiner pendingSet;
        set.reserve(alt->size());
        assert ("combine list must be empty!" && pendingSet.empty());
        for (RE * item : *alt) {
            // since transform will memorize every item/nestedItem, set insert is sufficient here
            item = transform(item);
            if (LLVM_UNLIKELY(isa<Alt>(item))) {
                const Alt & nestedAlt = *cast<Alt>(item);
                set.reserve(set.capacity() + nestedAlt.size());
                for (RE * const nestedItem : nestedAlt) {
                    assert ("nested Alts should already be flattened!" && !isa<Alt>(nestedItem));
                    if (LLVM_LIKELY(!foundCCToInsertIntoPendingSet(item, pendingSet))) {
                        set.insert(nestedItem);
                    }
                }
            } else if (!foundCCToInsertIntoPendingSet(item, pendingSet)) {
                set.insert(item);
            }
        }
        // insert any CC objects into the alternation
        for (auto cc : pendingSet) {
            // transform(cc) for memoization.
            set.insert(transform(cc));
        }
        pendingSet.clear();
        // Pablo CSE may identify common prefixes but cannot identify common suffixes.
        extractCommonSuffixes(set, pendingSet);
        extractCommonPrefixes(set, pendingSet);
        if (unchanged(alt, set)) {
            return alt;
        } else {
            return makeAlt(set.begin(), set.end());
        }
    }

    RE * transformSeq(Seq * seq) override {

        List list;
        list.reserve(seq->size());
        for (RE * item : *seq) {
            item = transform(item);
            if (LLVM_UNLIKELY(isa<Seq>(item))) {
                for (RE * const innerItem : *cast<Seq>(item)) {
                    list.push_back(innerItem);
                }
            } else {
                list.push_back(item);
            }
        }
        for (unsigned i = 1, run = 0; i < list.size(); ) {
            if (LLVM_UNLIKELY(list[i - 1] == list[i])) {
                ++run;
            } else if (LLVM_UNLIKELY(run != 0)) {
                // If we have a run of the same RE, make a bounded repetition for it
                const auto j = i - run; assert (j > 0);
                list[j - 1] = transform(makeRep(list[j - 1], run + 1, run + 1));
                list.erase(list.begin() + j, list.begin() + i);
                i = j;
                run = 0;
                continue;
            } else if (LLVM_UNLIKELY(isa<Rep>(list[i - 1]) && isa<Rep>(list[i]))){
                // If we have adjacent repetitions of the same RE, merge them
                Rep * const r1 = cast<Rep>(list[i - 1]);
                Rep * const r2 = cast<Rep>(list[i]);
                if (LLVM_UNLIKELY(r1->getRE() == r2->getRE())) {
                    list[i - 1] = transform(combineTwoReps(r1, r2));
                    list.erase(list.begin() + i);
                    continue;
                }
            }
            ++i;
        }
        if (unchanged(seq, list)) {
            return seq;
        } else {
            return makeSeq(list.begin(), list.end());
        }
    }

    RE_Minimizer() : RE_MemoizingTransformer("Minimizer", NameTransformationMode::TransformDefinition) { }

protected:

    void extractCommonPrefixes(Set & alts, Combiner & pendingSet) {
        if (LLVM_LIKELY(alts.size() > 1)) {
            SmallVector<RE *, 8> optimized;
            for (auto i = alts.begin(); i != alts.end(); ) {
                assert ("combine list must be empty!" && pendingSet.empty());
                RE * const head = headOf(*i);
                for (auto j = i + 1; j != alts.end(); ) {
                    if (LLVM_UNLIKELY(head == headOf(*j))) {
                        pendingSet.push_back(*j);
                        j = alts.erase(j);
                    } else {
                        ++j;
                    }
                }
                if (LLVM_LIKELY(pendingSet.empty())) {
                    ++i;
                } else {
                    pendingSet.push_back(*i);
                    i = alts.erase(i);
                    Set tailSet;
                    tailSet.reserve(pendingSet.size());
                    bool nullable = false;
                    for (RE * const re : pendingSet) {
                        if (LLVM_LIKELY(isa<Seq>(re))) {
                            Seq * const seq = cast<Seq>(re);
                            if (LLVM_LIKELY(seq->size() > 1)) {
                                assert (head == seq->front());
                                tailSet.insert(transform(tailOf(seq)));
                                continue;
                            }
                        } else if (LLVM_UNLIKELY(isa<Rep>(re))) {
                            Rep * const rep = cast<Rep>(re);
                            if (head != rep) {
                                assert (head == rep->getRE());
                                tailSet.insert(transform(makeRepWithOneFewerRepitition(rep)));
                                continue;
                            }
                        }
                        nullable = true;
                    }
                    pendingSet.clear();
                    if (LLVM_UNLIKELY(nullable)) {
                        tailSet.insert(transform(makeSeq()));
                    }
                    RE * const tail = makeAlt(tailSet.begin(), tailSet.end());
                    optimized.push_back(transform(makeSeq({ head, tail })));
                }
            }
            alts.insert(optimized.begin(), optimized.end());
        }
    }

    void extractCommonSuffixes(Set & alts, Combiner & pendingSet) {
        if (LLVM_LIKELY(alts.size() > 1)) {
            SmallVector<RE *, 8> optimized;
            for (auto i = alts.begin(); i != alts.end(); ) {
                assert ("combine list must be empty!" && pendingSet.empty());
                RE * const last = lastOf(*i);
                for (auto j = i + 1; j != alts.end(); ) {
                    if (LLVM_UNLIKELY(last == lastOf(*j))) {
                        pendingSet.push_back(*j);
                        j = alts.erase(j);
                    } else {
                        ++j;
                    }
                }
                if (LLVM_LIKELY(pendingSet.empty())) {
                    ++i;
                } else {
                    pendingSet.push_back(*i);
                    i = alts.erase(i);
                    Set initSet;
                    initSet.reserve(pendingSet.size());
                    bool nullable = false;
                    for (RE * const re : pendingSet) {
                        if (LLVM_LIKELY(isa<Seq>(re))) {
                            Seq * const seq = cast<Seq>(re);
                            if (LLVM_LIKELY(seq->size() > 1)) {
                                assert (last == seq->back());
                                initSet.insert(transform(initOf(seq)));
                                continue;
                            }
                        } else if (LLVM_UNLIKELY(isa<Rep>(re))) {
                            Rep * const rep = cast<Rep>(re);
                            if (last != rep) {
                                assert (last == rep->getRE());
                                initSet.insert(transform(makeRepWithOneFewerRepitition(rep)));
                                continue;
                            }
                        }
                        nullable = true;
                    }
                    pendingSet.clear();
                    if (LLVM_UNLIKELY(nullable)) {
                        initSet.insert(transform(makeSeq()));
                    }
                    RE * const init = makeAlt(initSet.begin(), initSet.end());
                    optimized.push_back(transform(makeSeq({ init, last })));
                }
            }
            alts.insert(optimized.begin(), optimized.end());
        }
    }

    static bool foundCCToInsertIntoPendingSet(RE * const re, Combiner & pendingSet) {
        if (isa<CC>(re)) {
            combineCC(cast<CC>(re), pendingSet);
            return true;
        } else if (isa<Name>(re)) {
            RE * const def = cast<Name>(re)->getDefinition();
            if (LLVM_LIKELY(isa<CC>(def))) {
                combineCC(cast<CC>(def), pendingSet);
                return true;
            }
        }
        return false;
    }

    static void combineCC(CC * const cc, Combiner & pendingSet) {
        for (RE *& existing : pendingSet) {
            if (LLVM_LIKELY(cast<CC>(existing)->getAlphabet() == cc->getAlphabet())) {
                existing = makeCC(cast<CC>(existing), cc);
                return;
            }
        }
        pendingSet.push_back(cc);
    }

    static RE * combineTwoReps(Rep * const r1, Rep * const r2) {
        assert (r1->getRE() == r2->getRE());
        assert (r1->getLB() != Rep::UNBOUNDED_REP);
        assert (r2->getLB() != Rep::UNBOUNDED_REP);
        const int lb = r1->getLB() + r2->getLB();
        int ub = Rep::UNBOUNDED_REP;
        if ((r1->getUB() != Rep::UNBOUNDED_REP) && (r2->getUB() != Rep::UNBOUNDED_REP)) {
            assert (r1->getUB() < (std::numeric_limits<int>::max() - r2->getUB()));
            ub = r1->getUB() + r2->getUB();
        }
        return makeRep(r1->getRE(), lb, ub);
    }

    static RE * makeRepWithOneFewerRepitition(Rep * const re) {
        assert (re->getLB() > 0);
        const auto lb = re->getLB() - 1;
        assert (re->getUB() != 0);
        const auto ub = (re->getUB() == Rep::UNBOUNDED_REP) ? Rep::UNBOUNDED_REP : re->getUB() - 1;
        return makeRep(re->getRE(), lb, ub);
    }

    static RE * headOf(RE * const re) {
        if (Seq * seq = dyn_cast<Seq>(re)) {
            if (LLVM_LIKELY(!seq->empty())) {
                return seq->front();
            }
        } else if (Rep * const rep = dyn_cast<Rep>(re)) {
            if (rep->getLB() > 0) {
                return rep->getRE();
            }
        }
        return re;
    }

    static RE * tailOf(Seq * const seq) {
        return makeSeq(seq->begin() + 1, seq->end());
    }

    static RE * lastOf(RE * const re) {
        if (Seq * seq = dyn_cast<Seq>(re)) {
            if (LLVM_LIKELY(!seq->empty())) {
                return seq->back();
            }
        } else if (Rep * const rep = dyn_cast<Rep>(re)) {
            if (rep->getLB() > 0) {
                return rep->getRE();
            }
        }
        return re;
    }

    static RE * initOf(Seq * const seq) {
        return makeSeq(seq->begin(), seq->end() - 1);
    }


    template <typename T1, typename T2>
    static bool unchanged(const T1 * A, const T2 & B) {
        if (A->size() != B.size()) {
            return false;
        }
        auto i = A->cbegin();
        for (const auto j : B) {
            if (*i != j) {
                return false;
            }
            ++i;
        }
        return true;
    }

};

RE * minimizeRE(RE * re) {
    return RE_Minimizer().transformRE(re);
}

}
