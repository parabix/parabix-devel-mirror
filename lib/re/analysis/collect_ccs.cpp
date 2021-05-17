#include <re/analysis/collect_ccs.h>

#include <re/adt/adt.h>
#include <re/alphabet/alphabet.h>
#include <re/analysis/re_inspector.h>

#include <boost/container/flat_set.hpp>
#include <llvm/Support/raw_ostream.h>

using namespace llvm;

namespace re {
    
struct SetCollector final : public RE_Inspector {

    SetCollector(const cc::Alphabet * alphabet, re::NameProcessingMode m, std::vector<CC *> & ccs)
    : RE_Inspector(m, InspectionMode::IgnoreNonUnique)
    , alphabet(alphabet)
    , ccs(ccs) {

    }

    void inspectPropertyExpression(PropertyExpression * pe) final {
        RE * resolved = pe->getResolvedRE();
        if (resolved) inspectRE(resolved);
    }

    void inspectCC(CC * cc) final {
        if (LLVM_LIKELY(cc->getAlphabet() == alphabet)) {
            ccs.push_back(cc);
        }
    }

private:
    const cc::Alphabet * const alphabet;
    std::vector<CC *> & ccs;
};


std::vector<CC *> collectCCs(RE * const re, const cc::Alphabet & a, re::NameProcessingMode m) {
    std::vector<CC *> ccs;
    SetCollector collector(&a, m, ccs);
    collector.inspectRE(re);
    return ccs;
}



}
