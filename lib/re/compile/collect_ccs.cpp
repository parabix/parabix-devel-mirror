#include <re/compile/collect_ccs.h>

#include <re/adt/re_name.h>
#include <re/adt/re_any.h>
#include <re/adt/re_alt.h>
#include <re/adt/re_cc.h>
#include <re/adt/re_seq.h>
#include <re/adt/re_rep.h>
#include <re/adt/re_range.h>
#include <re/adt/re_diff.h>
#include <re/adt/re_intersect.h>
#include <re/adt/re_assertion.h>
#include <re/cc/alphabet.h>
#include <re/cc/alphabet.h>
#include <re/compile/re_toolchain.h>

#include <boost/container/flat_set.hpp>
#include <llvm/Support/raw_ostream.h>

using namespace llvm;

namespace re {
    
struct SetCollector final : public RE_Inspector {

    SetCollector(const cc::Alphabet * alphabet, const std::set<Name *> & ignoredExternals, std::vector<CC *> & ccs)
    : RE_Inspector(InspectionMode::IgnoreNonUnique)
    , alphabet(alphabet)
    , ignoredExternals(ignoredExternals)
    , ccs(ccs) {

    }

    void inspectName(Name * n) final {
        if (ignoredExternals.count(n) == 0) {
            RE_Inspector::inspectName(n);
        }
    }

    void inspectCC(CC * cc) final {
        if (LLVM_LIKELY(cc->getAlphabet() == alphabet)) {
            ccs.push_back(cc);
        }
    }

private:
    const cc::Alphabet * const alphabet;
    const std::set<Name *> & ignoredExternals;
    std::vector<CC *> & ccs;
};


std::vector<CC *> collectCCs(RE * const re, const cc::Alphabet & a, std::set<Name *> external) {
    std::vector<CC *> ccs;
    SetCollector collector(&a, external, ccs);
    collector.inspectRE(re);
    return ccs;
}



}
