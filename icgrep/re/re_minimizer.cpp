#include "re_minimizer.h"
#include <re/re_name.h>
#include <re/re_alt.h>
#include <re/re_cc.h>
#include <re/re_seq.h>
#include <re/re_rep.h>
#include <re/re_diff.h>
#include <re/re_intersect.h>
#include <re/re_assertion.h>
#include <re/re_memoizer.hpp>
#include <boost/container/flat_set.hpp>
#include <boost/container/flat_map.hpp>
#include <llvm/ADT/SmallVector.h>


using namespace llvm;
using namespace boost::container;

namespace re {

using Set = flat_set<RE *>;
using Map = flat_map<RE *, RE *>;
using List = std::vector<RE *>;

struct PassContainer : private Memoizer {

    RE * minimize(RE * re) {
        const auto f = find(re);
        if (LLVM_UNLIKELY(f != end())) {
            return *f;
        }
        if (Alt * const alt = dyn_cast<Alt>(re)) {
            Set set;
            set.reserve(alt->size());
            for (RE * item : *alt) {
                item = minimize(item);
                if (LLVM_UNLIKELY(isa<Alt>(item))) {
                    for (RE * const innerItem : *cast<Alt>(item)) {
                        set.insert(innerItem);
                    }
                } else if (CC * const cc = extractCC(item)) {
                    combineCC(cc);
                } else {
                    set.insert(item);
                }
            }
            // insert any CC objects into the alternation
            for (auto cc : mCombine) {
                set.insert(memoize(cc));
            }
            mCombine.clear();
            // Pablo CSE may identify common prefixes but cannot identify common suffixes.
            extractCommonSuffixes(set);
            extractCommonPrefixes(set);
            re = makeAlt(set.begin(), set.end());
        } else if (Seq * const seq = dyn_cast<Seq>(re)) {
            List list;
            list.reserve(seq->size());
            for (RE * item : *seq) {
                item = minimize(item);
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
                    list[j - 1] = memoize(makeRep(list[j - 1], run + 1, run + 1));
                    list.erase(list.begin() + j, list.begin() + i);
                    i = j;
                    run = 0;
                    continue;
                } else if (LLVM_UNLIKELY(isa<Rep>(list[i - 1]) && isa<Rep>(list[i]))){
                    // If we have adjacent repetitions of the same RE, merge them
                    Rep * const r1 = cast<Rep>(list[i - 1]);
                    Rep * const r2 = cast<Rep>(list[i]);
                    if (LLVM_UNLIKELY(r1->getRE() == r2->getRE())) {
                        list[i - 1] = memoize(combineTwoReps(r1, r2));
                        list.erase(list.begin() + i);
                        continue;
                    }
                }
                ++i;
            }
            re = makeSeq(list.begin(), list.end());
        } else if (Assertion * const a = dyn_cast<Assertion>(re)) {
            re = makeAssertion(minimize(a->getAsserted()), a->getKind(), a->getSense());
        } else if (Rep * const rep = dyn_cast<Rep>(re)) {
            re = makeRep(minimize(rep->getRE()), rep->getLB(), rep->getUB());
        } else if (Diff * const diff = dyn_cast<Diff>(re)) {
            re = makeDiff(minimize(diff->getLH()), minimize(diff->getRH()));
        } else if (Intersect * const ix = dyn_cast<Intersect>(re)) {
            re = makeIntersect(minimize(ix->getLH()), minimize(ix->getRH()));
        } else if (Name * const n = dyn_cast<Name>(re)) {
            RE * const def = n->getDefinition();
            if (LLVM_LIKELY(def != nullptr)) {
                n->setDefinition(minimize(def));
            }
        }
        return memoize(re);
    }

protected:

    void extractCommonPrefixes(Set & alts) {        
        if (LLVM_LIKELY(alts.size() > 1)) {
            SmallVector<RE *, 8> optimized;
            for (auto i = alts.begin(); i != alts.end(); ) {
                assert ("combine list must be empty!" && mCombine.empty());
                RE * const head = headOf(*i);
                for (auto j = i + 1; j != alts.end(); ) {
                    if (LLVM_UNLIKELY(head == headOf(*j))) {
                        mCombine.push_back(*j);
                        j = alts.erase(j);
                    } else {
                        ++j;
                    }
                }
                if (LLVM_LIKELY(mCombine.empty())) {
                    ++i;
                } else {
                    mCombine.push_back(*i);
                    i = alts.erase(i);
                    Set tailSet;
                    tailSet.reserve(mCombine.size());
                    bool nullable = false;
                    for (RE * const re : mCombine) {
                        if (LLVM_LIKELY(isa<Seq>(re))) {
                            Seq * const seq = cast<Seq>(re);
                            if (LLVM_LIKELY(seq->size() > 1)) {
                                assert (head == seq->front());
                                tailSet.insert(memoize(makeSeq(seq->begin() + 1, seq->end())));
                                continue;
                            }
                        } else if (LLVM_UNLIKELY(isa<Rep>(re))) {
                            Rep * const rep = cast<Rep>(re);
                            if (head != rep) {
                                assert (head == rep->getRE());
                                tailSet.insert(memoize(makeRepWithOneFewerRepitition(rep)));
                                continue;
                            }
                        }
                        nullable = true;
                    }
                    mCombine.clear();
                    if (LLVM_UNLIKELY(nullable)) {
                        tailSet.insert(memoize(makeSeq()));
                    }
                    RE * const tail = makeAlt(tailSet.begin(), tailSet.end());
                    optimized.push_back(minimize(makeSeq({ head, tail })));
                }
            }
            alts.insert(optimized.begin(), optimized.end());
        }        
    }

    void extractCommonSuffixes(Set & alts) {
        if (LLVM_LIKELY(alts.size() > 1)) {
            SmallVector<RE *, 8> optimized;
            for (auto i = alts.begin(); i != alts.end(); ) {
                assert ("combine list must be empty!" && mCombine.empty());
                RE * const last = lastOf(*i);
                for (auto j = i + 1; j != alts.end(); ) {
                    if (LLVM_UNLIKELY(last == lastOf(*j))) {
                        mCombine.push_back(*j);
                        j = alts.erase(j);
                    } else {
                        ++j;
                    }
                }
                if (LLVM_LIKELY(mCombine.empty())) {
                    ++i;
                } else {
                    mCombine.push_back(*i);
                    i = alts.erase(i);
                    Set initSet;
                    initSet.reserve(mCombine.size());
                    bool nullable = false;
                    for (RE * const re : mCombine) {
                        if (LLVM_LIKELY(isa<Seq>(re))) {
                            Seq * const seq = cast<Seq>(re);
                            if (LLVM_LIKELY(seq->size() > 1)) {
                                assert (last == seq->back());
                                initSet.insert(memoize(makeSeq(seq->begin(), seq->end() - 1)));
                                continue;
                            }
                        } else if (LLVM_UNLIKELY(isa<Rep>(re))) {
                            Rep * const rep = cast<Rep>(re);
                            if (last != rep) {
                                assert (last == rep->getRE());
                                initSet.insert(memoize(makeRepWithOneFewerRepitition(rep)));
                                continue;
                            }
                        }
                        nullable = true;
                    }
                    mCombine.clear();
                    if (LLVM_UNLIKELY(nullable)) {
                        initSet.insert(memoize(makeSeq()));
                    }
                    RE * const init = makeAlt(initSet.begin(), initSet.end());
                    optimized.push_back(minimize(makeSeq({ init, last })));
                }
            }
            alts.insert(optimized.begin(), optimized.end());
        }        
    }

    static CC * extractCC(RE * const re) {
        if (isa<CC>(re)) {
            return cast<CC>(re);
        } else if (isa<Name>(re)) {
            RE * const def = cast<Name>(re)->getDefinition();
            if (LLVM_LIKELY(isa<CC>(def))) {
                return cast<CC>(def);
            }
        }
        return nullptr;
    }

    void combineCC(CC * const cc) {
        for (RE *& existing : mCombine) {
            if (LLVM_LIKELY(cast<CC>(existing)->getAlphabet() == cc->getAlphabet())) {
                existing = makeCC(cast<CC>(existing), cc);
                return;
            }
        }
        mCombine.push_back(cc);
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

    SmallVector<RE *, 16> mCombine;
};


RE * RE_Minimizer::minimize(RE * re) {
    PassContainer pc;
    return pc.minimize(re);
}

}
