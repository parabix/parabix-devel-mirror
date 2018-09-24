#include "re_multiplex.h"
#include <re/re_name.h>
#include <re/re_alt.h>
#include <re/re_cc.h>
#include <re/re_seq.h>
#include <re/re_rep.h>
#include <re/re_diff.h>
#include <re/re_intersect.h>
#include <re/re_assertion.h>
#include <re/re_group.h>
#include <re/re_analysis.h>
#include <re/re_memoizer.hpp>
#include <re/printer_re.h>
#include <UCD/ucd_compiler.hpp>
#include <UCD/resolve_properties.h>
#include <boost/container/flat_set.hpp>
#include <cc/alphabet.h>
#include <cc/multiplex_CCs.h>
#include <sstream>
#include <iostream>
#include <functional>
#include <llvm/Support/raw_ostream.h>

using namespace boost::container;
using namespace llvm;

namespace re {
  

class CC_multiplexer : public RE_Transformer {
public:
    CC_multiplexer(cc::MultiplexedAlphabet * mpx) : RE_Transformer(), mMultiplexedAlphabet(mpx) {}
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
    return CC_multiplexer(mpx).transform(re);
} 

}
