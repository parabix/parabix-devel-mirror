#ifndef RE_MULTIPLEX_H
#define RE_MULTIPLEX_H

#include <UCD/ucd_compiler.hpp>
#include <cc/multiplex_CCs.h>
#include <re/re_utility.h>

namespace re {

    class RE;
    class Name;
    class CC;

    RE * multiplex(RE * const re,
                   const std::vector<const CC *> & UnicodeSets,
                   const std::vector<std::vector<unsigned>> & exclusiveSetIDs);

    RE * transformCCs(cc::MultiplexedAlphabet * mpx, RE * r);

    
    class CC_multiplexer : public RE_Transformer {
    public:
        CC_multiplexer(cc::MultiplexedAlphabet * mpx) : RE_Transformer(), mMultiplexedAlphabet(mpx) {}
        RE * transformCC(CC *) override;
        RE * transformName(Name *) override;
    private:
        cc::MultiplexedAlphabet * mMultiplexedAlphabet;
    };

}
#endif
