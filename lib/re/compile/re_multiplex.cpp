#include <re/compile/re_multiplex.h>

#include <functional>
#include <iostream>
#include <sstream>
#include <boost/container/flat_set.hpp>
#include <llvm/Support/raw_ostream.h>
#include <re/adt/adt.h>
#include <re/adt/re_utility.h>
#include <re/adt/printer_re.h>
#include <re/alphabet/alphabet.h>
#include <re/cc/multiplex_CCs.h>
#include <re/compile/re_analysis.h>
#include <re/compile/re_transformer.h>
#include <unicode/compile/ucd_compiler.hpp>
#include <unicode/compile/resolve_properties.h>

using namespace boost::container;
using namespace llvm;

namespace re {
  

class CC_multiplexer : public RE_Transformer {
public:
    CC_multiplexer(cc::MultiplexedAlphabet * mpx) :
        RE_Transformer("Multiplex_" + mpx->getName()), mMultiplexedAlphabet(mpx) {}
    RE * transformCC(CC *) override;
    RE * transformName(Name *) override;
private:
    cc::MultiplexedAlphabet * mMultiplexedAlphabet;
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
        else if (name->getType() == Name::Type::Capture)
            return makeCapture(name->getName(), xfrm);
        else
            return makeName(name->getName(), xfrm);
    }
    return name;
}

RE * transformCCs(cc::MultiplexedAlphabet * mpx, RE * re) {
    return CC_multiplexer(mpx).transformRE(re);
} 

}
