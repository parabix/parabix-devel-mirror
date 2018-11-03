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

struct PassContainer final : public RE_Transformer {

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

    PassContainer() : RE_Transformer("Simplifier", NameTransformationMode::TransformDefinition) { }

};

RE * RE_Simplifier::simplify(RE * re) {
    PassContainer pc;
    return pc.transformRE(re);
}

}
