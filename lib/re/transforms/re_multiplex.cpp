#include <re/transforms/re_multiplex.h>

#include <functional>
#include <re/adt/adt.h>
#include <re/alphabet/alphabet.h>
#include <re/cc/multiplex_CCs.h>
#include <re/analysis/re_analysis.h>
#include <re/transforms/re_transformer.h>

using namespace llvm;

namespace re {


class CC_multiplexer final : public RE_Transformer {
public:
    CC_multiplexer(const cc::MultiplexedAlphabet * mpx) :
        RE_Transformer("Multiplex_" + mpx->getName()), mMultiplexedAlphabet(mpx) {}
    RE * transformCC(CC *) override;
    RE * transformName(Name *) override;
private:
    const cc::MultiplexedAlphabet * const mMultiplexedAlphabet;
};

RE * CC_multiplexer::transformCC(CC * cc) {
    if (cc->getAlphabet() == mMultiplexedAlphabet->getSourceAlphabet()) {
        return mMultiplexedAlphabet->transformCC(cc);
    }
    return cc;
}

RE * CC_multiplexer::transformName(Name * name) {
    if (LLVM_LIKELY(name->getDefinition() != nullptr)) {
        RE * xfrm = transform(name->getDefinition());
        if (name->getType() == Name::Type::ZeroWidth)
            return makeZeroWidth(name->getName(), xfrm);
        else
            return makeName(name->getName(), xfrm);
    }
    return name;
}

RE * transformCCs(const cc::MultiplexedAlphabet * const mpx, RE * re) {
    return CC_multiplexer(mpx).transformRE(re);
}

}
