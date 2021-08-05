/*
 *  Copyright (c) 2019 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#include <re/transforms/re_simplifier.h>

#include <re/adt/adt.h>
#include <re/analysis/re_inspector.h>
#include <re/transforms/re_memoizing_transformer.h>

#include <util/small_flat_set.hpp>

using namespace llvm;

namespace re {

using Set = SmallFlatSet<RE *, 16>;
using List = std::vector<RE *>;

struct RE_Simplifier final : public RE_MemoizingTransformer {

    RE * transformAlt(Alt * alt) override {
        Set set;
        set.reserve(alt->size());
        for (RE * item : *alt) {
            item = transform(item);
            if (LLVM_UNLIKELY(isa<Alt>(item))) {
                const Alt & alt = *cast<Alt>(item);
                set.reserve(set.capacity() + alt.size());
                set.insert(alt.begin(), alt.end());
            }  else {
                set.insert(item);
            }
        }
        return makeAlt(set.begin(), set.end());
    }

    RE * transformSeq(Seq * seq) override {
        List list;
        list.reserve(seq->size());
        for (RE * item : *seq) {
            item = transform(item);
            if (LLVM_UNLIKELY(isa<Seq>(item) && cast<Seq>(item)->empty())) {
                continue;
            }
            list.push_back(item);
        }
        return makeSeq(list.begin(), list.end());
    }

    RE * transformName(Name * nm) override {
        nm->setDefinition(transform(nm->getDefinition()));
        return nm;
    }

    RE_Simplifier() : RE_MemoizingTransformer("Simplifier", NameTransformationMode::TransformDefinition) { }

};

RE * simplifyRE(RE * re) {
    return RE_Simplifier().transformRE(re);
}

using ReferenceSet = boost::container::flat_set<std::string>;

struct ReferenceCollector  final : public RE_Inspector {
    ReferenceCollector(ReferenceSet & references)
    : RE_Inspector(NameProcessingMode::None, InspectionMode::IgnoreNonUnique), mReferences(references) {}

    void inspectReference(Reference * r) override {
        auto name = r -> getName();
        mReferences.insert(r->getName());
    }

private:
    ReferenceSet & mReferences;
};

struct UnneededCaptureRemoval  : public RE_Transformer {
    UnneededCaptureRemoval(ReferenceSet & references)
    : RE_Transformer("UnneededCaptureRemoval"), mReferences(references) {}

    RE * transformCapture(Capture * c) override {
        auto name = c->getName();
        auto x = c->getCapturedRE();
        auto t = transform(x);
        if (mReferences.count(c->getName()) == 0) {
            return t;
        } else if (t != x) {
            return makeCapture(name, t);
        }
        return c;
    }

private:
    ReferenceSet & mReferences;
};

RE * removeUnneededCaptures(RE * r) {
    ReferenceSet refs;
    ReferenceCollector(refs).inspectRE(r);
    return UnneededCaptureRemoval(refs).transformRE(r);
}
}
