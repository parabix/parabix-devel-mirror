#include <re/re_re.h>
#include "re_name_resolve.h"
#include <re/re_name.h>
#include <re/re_alt.h>
#include <re/re_cc.h>
#include <re/re_seq.h>
#include <re/re_rep.h>
#include <re/re_range.h>
#include <re/re_diff.h>
#include <re/re_intersect.h>
#include <re/re_assertion.h>
#include <re/re_analysis.h>
#include <re/re_group.h>
#include <re/re_start.h>
#include <re/re_end.h>
#include <re/re_any.h>
#include <re/re_toolchain.h>
#include <re/re_memoizer.hpp>
#include <UCD/resolve_properties.h>
#include <cc/alphabet.h>
#include <boost/container/flat_set.hpp>
#include <llvm/Support/ErrorHandling.h>


using namespace boost::container;
using namespace llvm;

namespace re {
  
class UnicodeNameResolver : public RE_Transformer {
public:
    UnicodeNameResolver() : RE_Transformer("UnicodeNames") {}
    RE * transformName(Name * name) override;
private:
    Memoizer mMemoizer;
};
    
RE * UnicodeNameResolver::transformName(Name * name) {
    auto f = mMemoizer.find(name);
    if (f == mMemoizer.end()) {
        if (LLVM_LIKELY(name->getDefinition() != nullptr)) {
            name->setDefinition(transform(name->getDefinition()));
        } else if (LLVM_LIKELY(name->getType() == Name::Type::UnicodeProperty || name->getType() == Name::Type::ZeroWidth)) {
            if (UCD::resolvePropertyDefinition(name)) {
                name->setDefinition(transform(name->getDefinition()));
            } else {
                name->setDefinition(makeCC(UCD::resolveUnicodeSet(name), &cc::Unicode));
            }
        } else {
            UndefinedNameError(name);
        }
        return mMemoizer.memoize(name);
    } else {
        return *f;
    }
}

RE * resolveUnicodeNames(RE * re) {
    return UnicodeNameResolver().transformRE(re);
}

 
class AnchorResolution : public RE_Transformer {
public:
    AnchorResolution(RE * anchorRE);
    RE * transformStart(Start * s) override;
    RE * transformEnd(End * s) override;

private:
    RE * mAnchorRE;
    bool mIsNegated;
};
 
AnchorResolution::AnchorResolution(RE * breakRE)
: RE_Transformer() {
    if (const CC * cc = dyn_cast<CC>(breakRE)) {
        mIsNegated = true;
        if (cc->getAlphabet() == &cc::Unicode) {
            mAnchorRE = makeDiff(makeCC(0, 0x10FFFF), breakRE);
        } else if (cc->getAlphabet() == &cc::Byte) {
            mAnchorRE = makeDiff(makeByte(0, 0xFF), breakRE);
        } else {
            llvm::report_fatal_error("resolveAnchors: unexpected alphabet " + cc->getAlphabet()->getName());
        }
    } else {
        mIsNegated = false;
        mAnchorRE = breakRE;
    }
}

RE * AnchorResolution::transformStart(Start * s) {
    if (mIsNegated) return makeNegativeLookBehindAssertion(mAnchorRE);
    return makeAlt({makeSOT(), makeLookBehindAssertion(mAnchorRE)});
}

RE * AnchorResolution::transformEnd(End * e) {
    if (mIsNegated) return makeNegativeLookAheadAssertion(mAnchorRE);
    return makeAlt({makeEOT(), makeLookAheadAssertion(mAnchorRE)});
}

RE * resolveAnchors(RE * r, RE * breakRE) {
    return AnchorResolution(breakRE).transformRE(r);
}
                                                        
}
