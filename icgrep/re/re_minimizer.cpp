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

using namespace llvm;

namespace re {

using AlternationSet = boost::container::flat_set<RE *, MemoizerComparator>;

using Map = boost::container::flat_map<RE *, RE *>;

struct PassContainer {

    RE * minimize(RE * const original) {
        const auto f = mMapping.find(original);
        if (LLVM_UNLIKELY(f != mMapping.end())) {
            return f->second;
        }
        RE * re = original;
        if (Alt * alt = dyn_cast<Alt>(re)) {
            AlternationSet list;
            list.reserve(alt->size());
            for (RE * item : *alt) {
                item = minimize(item);
                if (LLVM_UNLIKELY(isa<Vector>(item) && cast<Vector>(item)->empty())) {
                    continue;
                }
                list.insert(item);
            }
            // When compiling Pablo code, CSE can identify common prefixes but cannot
            // identify common suffixes. Prioritize this optimization accordingly.
            extractCommonSuffixes(list);
            extractCommonPrefixes(list);
            re = makeAlt(list.begin(), list.end());
        } else if (Seq * seq = dyn_cast<Seq>(re)) {
            std::vector<RE *> list;
            list.reserve(seq->size());
            for (RE * item : *seq) {
                item = minimize(item);
                if (LLVM_UNLIKELY(isa<Vector>(item) && cast<Vector>(item)->empty())) {
                    continue;
                }
                list.push_back(item);
            }
            re = makeSeq(list.begin(), list.end());
        } else if (Assertion * a = dyn_cast<Assertion>(re)) {
            minimize(a->getAsserted());
        } else if (Rep * rep = dyn_cast<Rep>(re)) {
            minimize(rep->getRE());
        } else if (Diff * diff = dyn_cast<Diff>(re)) {
            minimize(diff->getLH());
            minimize(diff->getRH());
        } else if (Intersect * e = dyn_cast<Intersect>(re)) {
            minimize(e->getLH());
            minimize(e->getRH());
        } else if (Name * n = dyn_cast<Name>(re)) {
            assert (n->getDefinition());
            if (!isa<CC>(n->getDefinition())) {
                n->setDefinition(minimize(n->getDefinition()));
            }
        }
        mMapping.emplace(original, re);
        return re;
    }

    void extractCommonPrefixes(AlternationSet & source) {
        if (LLVM_UNLIKELY(source.size() < 2)) {
            return;
        }
        for (auto i = source.begin(); i != source.end(); ++i) {
            assert (mCombine.empty());
            assert (*i);
            if (Seq * seq_i = dyn_cast<Seq>(*i)) {
                assert (seq_i);
                RE * const head = seq_i->front();
                assert (head);
                for (auto j = i + 1; j != source.end(); ) {
                    if (Seq * seq_j = dyn_cast<Seq>(*j)) {
                        if (LLVM_UNLIKELY(head == seq_j->front())) {
                            mCombine.push_back(seq_j);
                            j = source.erase(j);
                            continue;
                        }
                    }
                    ++j;
                }
                if (LLVM_UNLIKELY(!mCombine.empty())) {
                    AlternationSet tailSet;
                    tailSet.reserve(mCombine.size() + 1);
                    for (Seq * seq_j : mCombine) {
                        if (LLVM_LIKELY(seq_j->size() > 1)) {
                            tailSet.insert(makeSeq(seq_j->begin() + 1, seq_j->end()));
                        }
                    }
                    mCombine.clear();
                    if (LLVM_LIKELY(seq_i->size() > 1)) {
                        tailSet.insert(makeSeq(seq_i->begin() + 1, seq_i->end()));
                    }
                    extractCommonPrefixes(tailSet);
                    source.erase(i);
                    RE * const tail = makeAlt(tailSet.begin(), tailSet.end());
                    source.insert(makeSeq({ head, tail }));
                    extractCommonPrefixes(source);
                    break;
                }
            }
        }
    }

    void extractCommonSuffixes(AlternationSet & source) {
        if (LLVM_UNLIKELY(source.size() < 2)) {
            return;
        }
        for (auto i = source.begin(); i != source.end(); ++i) {
            assert (mCombine.empty());
            assert (*i);
            if (Seq * seq_i = dyn_cast<Seq>(*i)) {
                assert (seq_i);
                assert (!seq_i->empty());
                RE * const tail = seq_i->back();
                for (auto j = i + 1; j != source.end(); ) {
                    if (Seq * seq_j = dyn_cast<Seq>(*j)) {
                        if (LLVM_UNLIKELY(tail == seq_j->back())) {
                            mCombine.push_back(seq_j);
                            j = source.erase(j);
                            continue;
                        }
                    }
                    ++j;
                }
                if (LLVM_UNLIKELY(!mCombine.empty())) {
                    AlternationSet headSet;
                    headSet.reserve(mCombine.size() + 1);
                    for (Seq * seq_j : mCombine) {
                        if (LLVM_LIKELY(seq_j->size() > 1)) {
                            headSet.insert(makeSeq(seq_j->begin(), seq_j->end() - 1));
                        }
                    }
                    mCombine.clear();
                    if (LLVM_LIKELY(seq_i->size() > 1)) {
                        headSet.insert(makeSeq(seq_i->begin(), seq_i->end() - 1));
                    }
                    extractCommonSuffixes(headSet);
                    extractCommonPrefixes(headSet);
                    source.erase(i);
                    RE * head = makeAlt(headSet.begin(), headSet.end());
                    source.insert(makeSeq({ head, tail }));
                    extractCommonSuffixes(source);
                    break;
                }
            }
        }
    }

private:

    std::vector<Seq *>  mCombine;
    Map                 mMapping;
};


RE * RE_Minimizer::minimize(RE * re) {
    PassContainer pc;
    return pc.minimize(re);
}

}
