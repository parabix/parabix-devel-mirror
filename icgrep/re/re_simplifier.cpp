#include "re_simplifier.h"
#include <re/re_name.h>
#include <re/re_alt.h>
#include <re/re_seq.h>
#include <re/re_rep.h>
#include <re/re_range.h>
#include <re/re_diff.h>
#include <re/re_intersect.h>
#include <re/re_assertion.h>
#include <re/re_toolchain.h>
#include <re/re_toolchain.h>
#include <boost/container/flat_set.hpp>

using namespace llvm;

namespace re {

using Set = boost::container::flat_set<RE *>;
using List = std::vector<RE *>;

struct RE_Simplifier final : public RE_Transformer {

    RE * transformAlt(Alt * alt) override {
        Set set;
        set.reserve(alt->size());
        for (RE * item : *alt) {
            item = transform(item);
            if (LLVM_UNLIKELY(isa<Alt>(item))) {
                for (RE * innerAlt : *cast<Alt>(item)) {
                    set.insert(innerAlt);
                }
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

    RE_Simplifier() : RE_Transformer("Simplifier", NameTransformationMode::TransformDefinition) { }

};

RE * simplifyRE(RE * re) {
    return RE_Simplifier().transformRE(re);
}

using ReferenceSet = boost::container::flat_set<std::string>;

struct ReferenceCollector  final : public RE_Inspector {
    ReferenceCollector(ReferenceSet & references)
    : RE_Inspector(InspectionMode::IgnoreNonUnique), mReferences(references) {}
    
    void inspectName(Name * n) final {
        if (n->getType() == Name::Type::Reference) {
            mReferences.insert(n->getName());
        }
    }
    
private:
    ReferenceSet & mReferences;
};

struct UnneededCaptureRemoval final : public RE_Transformer {
    UnneededCaptureRemoval(ReferenceSet & references)
    : RE_Transformer("UnneededCaptureRemoval"), mReferences(references) {}
    
    RE * transformName(Name * n) final {
        if (n->getType() == Name::Type::Capture) {
            if (mReferences.count(n->getName()) == 0) {
                return n->getDefinition();
            }
        }
        return n;
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
