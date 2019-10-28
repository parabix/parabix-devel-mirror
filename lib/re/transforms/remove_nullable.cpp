/*
 *  Copyright (c) 2019 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#include <re/transforms/remove_nullable.h>

#include <vector>
#include <re/adt/adt.h>
#include <re/analysis/nullable.h>
#include <re/transforms/re_transformer.h>

/*

 A regular expression is nullable if it (a) matches the empty
 string, and (b) applies everywhere.  Note that Start (^) and
 End ($) match the empty string, but not everywhere).

*/

using namespace llvm;

namespace re {

class NullablePrefixRemover: public RE_Transformer {
protected:
    RE * transformSeq(Seq * seq) override;
    RE * transformRep(Rep * rep) override;
    RE * transformAssertion(Assertion * a) override;
public:
    NullablePrefixRemover() : RE_Transformer("NullablePrefixRemoval") {}
};

RE * NullablePrefixRemover::transformSeq(Seq * seq) {
    std::vector<RE*> list;
    // if the sequence is empty, return it unmodified.
    if (isNullable(seq)) return seq;
    // Process the first element.
    auto i = seq->begin();
    auto e = transform(*i);
    while (isNullable(e)) {
        // Skip empty elements.
        i++;
        e = transform(*i);
    }
    // Special case: nothing skipped and first element unchanged.
    if ((i == seq->begin()) && (e == *i)) return seq;
    list.push_back(e);
    i++;
    while (i != seq->end()) {
        list.push_back(*i);
        i++;
    }
    return makeSeq(list.begin(), list.end());
}

RE * NullablePrefixRemover::transformRep(Rep * rep) {
    auto lb = rep->getLB();
    auto r = rep->getRE();
    if ((lb == 0) || isNullable(r)) {
        return makeSeq();
    }
    auto s = transform(r);
    if ((s == r) && (lb == rep->getUB())) return rep; // special case.  No transformation required.
    if (lb == 1) return s;
    if (lb == 2) return makeSeq({s, r});
    return makeSeq({s, makeRep(r, lb - 1, lb - 1)});
}

RE * NullablePrefixRemover::transformAssertion(Assertion * a) {
    return a;
}

RE * removeNullablePrefix(RE * r) {
    return NullablePrefixRemover().transformRE(r);
}

class NullableSuffixRemover: public RE_Transformer {
protected:
    RE * transformSeq(Seq * seq) override;
    RE * transformRep(Rep * rep) override;
    RE * transformAssertion(Assertion * a) override;
public:
    NullableSuffixRemover() : RE_Transformer("NullableSuffixRemoval") {}
};

RE * NullableSuffixRemover::transformSeq(Seq * seq) {
    std::vector<RE*> list;
    // if the sequence is empty, return it unmodified.
    if (isNullable(seq)) return seq;
    // Process the last element.
    auto ri = seq->rbegin();
    auto r = transform(*ri);
    while (isNullable(r)) {
        // Skip empty elements.
        ri++;
        r = transform(*ri);
    }
    // Special case: nothing skipped and first element unchanged.
    if ((ri == seq->rbegin()) && (r == *ri)) return seq;
    std::copy(seq->begin(), (ri + 1).base(), std::back_inserter(list));
    list.push_back(r);
    return makeSeq(list.begin(), list.end());
}

RE * NullableSuffixRemover::transformRep(Rep * rep) {
    auto lb = rep->getLB();
    auto r = rep->getRE();
    if ((lb == 0) || isNullable(r)) {
        return makeSeq();
    }
    auto s = transform(r);
    if ((s == r) && (lb == rep->getUB())) return rep; // special case.  No transformation required.
    if (lb == 1) return s;
    if (lb == 2) return makeSeq({r, s});
    return makeSeq({makeRep(r, lb - 1, lb - 1), s});
}

RE * NullableSuffixRemover::transformAssertion(Assertion * a) {
    return a;
}
RE * removeNullableSuffix(RE * r) {
    return NullableSuffixRemover().transformRE(r);
}

}
