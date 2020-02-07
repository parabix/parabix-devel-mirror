#include <re/transforms/re_multiplex.h>

#include <functional>
#include <re/adt/adt.h>
#include <re/alphabet/alphabet.h>
#include <re/cc/multiplex_CCs.h>
#include <re/analysis/re_analysis.h>
#include <re/transforms/re_transformer.h>

using namespace llvm;

namespace re {


struct CC_multiplexer final : public RE_Transformer {
public:
    CC_multiplexer(const cc::MultiplexedAlphabet * mpx, std::set<Name *> externalNames) :
        RE_Transformer("Multiplex_" + mpx->getName()),
        mMultiplexedAlphabet(mpx), mExternalNames(externalNames) {}
    RE * transformCC(CC * cc) override {
        if (cc->getAlphabet() == mMultiplexedAlphabet->getSourceAlphabet()) {
            return mMultiplexedAlphabet->transformCC(cc);
        }
        return cc;
    };
    RE * transformName(Name * name) override {
        if (mExternalNames.count(name) > 0) return name;
        if (LLVM_LIKELY(name->getDefinition() != nullptr)) {
            RE * xfrm = transform(name->getDefinition());
            return xfrm;
        }
        return name;
    }
private:
    const cc::MultiplexedAlphabet * const mMultiplexedAlphabet;
    std::set<Name *> mExternalNames;
};

RE * transformCCs(const cc::MultiplexedAlphabet * const mpx, RE * re, std::set<Name *> externalNames) {
    return CC_multiplexer(mpx, externalNames).transformRE(re);
}

}
