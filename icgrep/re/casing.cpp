#include <re/casing.h>
#include <re/re_cc.h>
#include <UCD/unicode_set.h>
#include <UCD/CaseFolding.h>
#include <re/re_alt.h>             // for Alt, makeAlt
#include <re/re_any.h>             // for makeAny, Any
#include <re/re_assertion.h>       // for Assertion, Assertion::Sense, Asser...
#include <re/re_start.h>
#include <re/re_end.h>
#include <re/re_diff.h>            // for Diff, makeDiff
#include <re/re_group.h>
#include <re/re_intersect.h>       // for Intersect
#include <re/re_name.h>            // for Name
#include <re/re_rep.h>             // for Rep, makeRep
#include <re/re_seq.h>             // for Seq, makeSeq
#include <vector>                  // for vector, allocator
#include <llvm/Support/Casting.h>  // for dyn_cast, isa
#include <llvm/Support/ErrorHandling.h>
#include <re/re_utility.h>

using namespace llvm;

namespace re {

class ResolveCaseInsensitiveMode final : public RE_Transformer {
public:
    RE * transformCC(CC * cc) override;
    RE * transformName(Name * name) override;
    RE * transformGroup(Group * g) override;

    ResolveCaseInsensitiveMode(const bool globallyCaseInsensitive) : inCaseInsensitiveMode(globallyCaseInsensitive) { }

private:
    bool inCaseInsensitiveMode;
};

RE * ResolveCaseInsensitiveMode::transformCC(CC * cc) {
    if (inCaseInsensitiveMode) {
        return makeCC(caseInsensitize(*cc));
    }
    return cc;
}

RE * ResolveCaseInsensitiveMode::transformName(Name * name) {
    if (!inCaseInsensitiveMode || (name->getDefinition() == nullptr)) {
        return name;
    }
    Name * n = nullptr;
    if (name->hasNamespace()) {
        n = makeName(name->getNamespace(), name->getName() + "/i", name->getType());
    } else {
        n = makeName(name->getName() + "/i", name->getType());
    }
    n->setDefinition(transform(name->getDefinition()));
    return n;
}

RE * ResolveCaseInsensitiveMode::transformGroup(Group * g) {
    if (g->getMode() == Group::Mode::CaseInsensitiveMode) {
        const auto wasInCaseInsensitiveMode = inCaseInsensitiveMode;
        inCaseInsensitiveMode = g->getSense() == Group::Sense::On;
        RE * const re = transform(g->getRE());
        inCaseInsensitiveMode = wasInCaseInsensitiveMode;
        return re;
    } else {
        return RE_Transformer::transformGroup(g);
    }
}


RE * resolveCaseInsensitiveMode(RE * re, const bool globallyCaseInsensitive) {
    ResolveCaseInsensitiveMode R(globallyCaseInsensitive);
    return R.transform(re);
}

}
