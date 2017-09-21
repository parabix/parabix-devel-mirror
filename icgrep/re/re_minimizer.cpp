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

#include <llvm/Support/raw_ostream.h>
#include <re/printer_re.h>

using namespace llvm;

namespace re {

using Set = boost::container::flat_set<RE *>;
using Map = boost::container::flat_map<RE *, RE *>;

struct PassContainer : private Memoizer {

    RE * minimize(RE * original) {
        const auto f = mMap.find(original);
        if (LLVM_UNLIKELY(f != mMap.end())) {
            return f->second;
        }
        RE * re = original;
repeat: if (Alt * alt = dyn_cast<Alt>(re)) {
            Set list;
            list.reserve(alt->size());
            RE * namedCC = nullptr;
            bool repeat = false;
            for (RE * item : *alt) {
                item = minimize(item);
                if (LLVM_UNLIKELY(isa<Vector>(item) && cast<Vector>(item)->empty())) {
                    continue;
                } else if (LLVM_UNLIKELY(isa<Alt>(item))) {
                    repeat = true;
                } else { // if we have an alternation containing multiple CCs, combine them
                    CC * const cc = extractCC(item);
                    if (cc) {
                        namedCC = namedCC ? makeCC(extractCC(namedCC), cc) : item;
                        continue;
                    }
                }
                list.insert(item);
            }
            // Combine and/or insert any named CC object into the alternation
            if (namedCC) {
                if (LLVM_UNLIKELY(isa<CC>(namedCC))) {
                    namedCC = memoize(cast<CC>(namedCC));
                }
                list.insert(cast<Name>(namedCC));
            }
            // Pablo CSE may identify common prefixes but cannot identify common suffixes.
            extractCommonSuffixes(list);
            extractCommonPrefixes(list);
            re = makeAlt(list.begin(), list.end());
            if (LLVM_UNLIKELY(repeat)) {
                goto repeat;
            }
        } else if (Seq * seq = dyn_cast<Seq>(re)) {
            std::vector<RE *> list;
            list.reserve(seq->size());
            bool repeat = false;
            for (RE * item : *seq) {
                item = minimize(item);
                if (LLVM_UNLIKELY(isa<Vector>(item) && cast<Vector>(item)->empty())) {
                    continue;
                } else if (LLVM_UNLIKELY(isa<Seq>(item))) {
                    repeat = true;
                }
                list.push_back(item);
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
                    // If we have adjacent repetitions of the same RE, we can merge them
                    Rep * const r1 = cast<Rep>(list[i - 1]);
                    Rep * const r2 = cast<Rep>(list[i]);
                    if (LLVM_UNLIKELY(r1->getRE() == r2->getRE())) {
                        list[i - 1] = memoize(combineRep(r1, r2));
                        list.erase(list.begin() + i);
                        continue;
                    }
                }
                ++i;
            }
            re = makeSeq(list.begin(), list.end());
            if (LLVM_UNLIKELY(repeat)) {
                goto repeat;
            }
        } else if (Assertion * a = dyn_cast<Assertion>(re)) {
            re = makeAssertion(minimize(a->getAsserted()), a->getKind(), a->getSense());
        } else if (Rep * rep = dyn_cast<Rep>(re)) {
            re = makeRep(minimize(rep->getRE()), rep->getLB(), rep->getUB());
        } else if (Diff * diff = dyn_cast<Diff>(re)) {
            re = makeDiff(minimize(diff->getLH()), minimize(diff->getRH()));
        } else if (Intersect * ix = dyn_cast<Intersect>(re)) {
            re = makeIntersect(minimize(ix->getLH()), minimize(ix->getRH()));
        } else if (Name * n = dyn_cast<Name>(re)) {
            RE * const def = n->getDefinition(); assert (def);
            if (LLVM_UNLIKELY(!isa<CC>(def))) {
                n->setDefinition(minimize(def));
            }
        }
        re = memoize(re);
        mMap.emplace(original, re);
        if (LLVM_LIKELY(original != re)) {
            mMap.emplace(re, re);
        }
        return re;
    }

protected:

    void extractCommonPrefixes(Set & source) {
        std::vector<RE *> combine;
restart:if (LLVM_UNLIKELY(source.size() < 2)) {
            return;
        }
        for (auto i = source.begin(); i != source.end(); ++i) {
            assert (combine.empty());            
            RE * const head = getHead(*i);
            for (auto j = i + 1; j != source.end(); ) {
                if (LLVM_UNLIKELY(head == getHead(*j))) {
                    combine.push_back(*j);
                    j = source.erase(j);
                    continue;
                }
                ++j;
            }

            if (LLVM_UNLIKELY(!combine.empty())) {
                combine.push_back(*i);
                source.erase(i);
                Set tailSet;
                tailSet.reserve(combine.size());
                bool isOptional = false;
                for (RE * const re : combine) {
                    if (LLVM_LIKELY(isa<Seq>(re))) {
                        Seq * const seq = cast<Seq>(re);
                        if (LLVM_LIKELY(seq->size() > 1)) {
                            tailSet.insert(minimize(makeSeq(seq->begin() + 1, seq->end())));
                            continue;
                        }
                    } else if (LLVM_UNLIKELY(isa<Rep>(re))) {
                        Rep * const rep = cast<Rep>(re);
                        if (head != rep) {
                            assert (head == rep->getRE());
                            assert (rep->getLB() > 0);
                            const auto lb = rep->getLB() - 1;
                            const auto ub = rep->getUB() == Rep::UNBOUNDED_REP ? Rep::UNBOUNDED_REP : rep->getUB() - 1;
                            tailSet.insert(minimize(makeRep(rep->getRE(), lb, ub)));
                            continue;
                        }
                    }
                    isOptional = true;
                }
                combine.clear();
                extractCommonPrefixes(tailSet);
                RE * tail = makeAlt(tailSet.begin(), tailSet.end());
                if (LLVM_UNLIKELY(isOptional)) {
                    tail = makeRep(tail, 0, 1);
                }
                source.insert(minimize(makeSeq({ head, tail })));
                goto restart;
            }
        }
    }

    void extractCommonSuffixes(Set & source) {
        std::vector<RE *> combine;
restart:if (LLVM_UNLIKELY(source.size() < 2)) {
            return;
        }
        for (auto i = source.begin(); i != source.end(); ++i) {
            assert (combine.empty());
            assert (*i);
            RE * const tail = getTail(*i);
            for (auto j = i + 1; j != source.end(); ) {
                if (LLVM_UNLIKELY(tail == getTail(*j))) {
                    combine.push_back(*j);
                    j = source.erase(j);
                    continue;
                }
                ++j;
            }
            if (LLVM_UNLIKELY(!combine.empty())) {
                combine.push_back(*i);
                source.erase(i);
                Set headSet;
                headSet.reserve(combine.size());
                bool isOptional = false;
                for (RE * const re : combine) {
                    if (LLVM_LIKELY(isa<Seq>(re))) {
                        Seq * const seq = cast<Seq>(re);
                        if (LLVM_LIKELY(seq->size() > 1)) {
                            headSet.insert(minimize(makeSeq(seq->begin(), seq->end() - 1)));
                            continue;
                        }
                    } else if (LLVM_UNLIKELY(isa<Rep>(re))) {
                        Rep * const rep = cast<Rep>(re);
                        if (tail != rep) {
                            assert (tail == rep->getRE());
                            assert (rep->getUB() != 0);
                            assert (rep->getLB() > 0);
                            const auto lb = rep->getLB() - 1;
                            const auto ub = (rep->getUB() == Rep::UNBOUNDED_REP) ? Rep::UNBOUNDED_REP : rep->getUB() - 1;
                            headSet.insert(minimize(makeRep(rep->getRE(), lb, ub)));
                            continue;
                        }
                    }
                    isOptional = true;
                }
                combine.clear();
                extractCommonSuffixes(headSet);
                extractCommonPrefixes(headSet);
                RE * head = makeAlt(headSet.begin(), headSet.end());
                if (LLVM_UNLIKELY(isOptional)) {
                    head = makeRep(head, 0, 1);
                }
                source.insert(minimize(makeSeq({ head, tail })));
                goto restart;
            }
        }
    }

    static CC * extractCC(RE * const re) {
        if (LLVM_UNLIKELY(isa<CC>(re))) {
            return cast<CC>(re);
        } else if (isa<Name>(re)) {
            RE * const def = cast<Name>(re)->getDefinition();
            if (LLVM_LIKELY(isa<CC>(def))) {
                return cast<CC>(def);
            }
        }
        return nullptr;
    }

    static RE * combineRep(Rep * const r1, Rep * const r2) {
        assert (r1->getRE() == r2->getRE());
        assert (r1->getLB() != Rep::UNBOUNDED_REP);
        assert (r2->getLB() != Rep::UNBOUNDED_REP);
        const int lb = r1->getLB() + r2->getLB();
        int ub = Rep::UNBOUNDED_REP;
        if ((r1->getUB() != Rep::UNBOUNDED_REP) && (r2->getUB() != Rep::UNBOUNDED_REP)) {
            ub = r1->getUB() + r2->getUB();
        }
        return makeRep(r1->getRE(), lb, ub);
    }

    static RE * getHead(RE * re) {
        assert (re);
        if (isa<Seq>(re)) {
            re = cast<Seq>(re)->front(); assert (re);
        } else if (isa<Rep>(re)) {
            Rep * const rep = cast<Rep>(re);
            assert (rep->getLB() != Rep::UNBOUNDED_REP);
            if (rep->getLB() > 0) {
                re = rep->getRE(); assert (re);
            }
        }
        return re;
    }

    static RE * getTail(RE * re) {
        assert (re);
        if (isa<Seq>(re)) {
            re = cast<Seq>(re)->back();
            assert (re);
        } else if (isa<Rep>(re)) {
            Rep * const rep = cast<Rep>(re);
            assert (rep->getLB() != Rep::UNBOUNDED_REP);
            assert (rep->getUB() == Rep::UNBOUNDED_REP || rep->getUB() >= rep->getLB());
            if (rep->getLB() > 0) {
                re = rep->getRE(); assert (re);
            }
        }
        return re;
    }

private:

    Map mMap;
};


RE * RE_Minimizer::minimize(RE * re) {
    PassContainer pc;
    return pc.minimize(re);
}

}
