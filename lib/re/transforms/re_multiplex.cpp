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
    CC_multiplexer(const cc::MultiplexedAlphabet * mpx, NameTransformationMode mode) :
        RE_Transformer("Multiplex_" + mpx->getName(), mode),
        mMultiplexedAlphabet(mpx) {}
    RE * transformCC(CC * cc) override {
        if (cc->getAlphabet() == mMultiplexedAlphabet->getSourceAlphabet()) {
            return mMultiplexedAlphabet->transformCC(cc);
        }
        return cc;
    };
    RE * transformPropertyExpression(PropertyExpression * pe) override {
        if (LLVM_LIKELY(pe->getResolvedRE() != nullptr)) {
            RE * xfrm = transform(pe->getResolvedRE());
            return xfrm;
        }
        return pe;
    }
private:
    const cc::MultiplexedAlphabet * const mMultiplexedAlphabet;
};

RE * transformCCs(const cc::MultiplexedAlphabet * const mpx, RE * re, NameTransformationMode mode) {
    return CC_multiplexer(mpx, mode).transformRE(re);
}

}
