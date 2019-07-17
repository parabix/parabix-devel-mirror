#include <re/unicode/re_name_resolve.h>

#include <boost/container/flat_set.hpp>
#include <llvm/Support/ErrorHandling.h>
#include <re/adt/adt.h>
#include <re/alphabet/alphabet.h>
#include <re/compile/re_analysis.h>
#include <re/compile/re_transformer.h>
#include <re/unicode/resolve_properties.h>


using namespace boost::container;
using namespace llvm;

namespace re {
  
class UnicodeNameResolver final : public RE_Transformer {
public:
    UnicodeNameResolver() : RE_Transformer("UnicodeNames") {}
    RE * transformName(Name * name) override;
    RE * transformRange(Range * rg) override;
};
    
RE * UnicodeNameResolver::transformName(Name * name) {
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
    return name;
}

RE * getDefinitionOf(RE * const re) {
    if (Name * const name = dyn_cast<Name>(re)) {
        RE * const def = name->getDefinition();
        if (LLVM_UNLIKELY(def == nullptr)) {
            llvm::report_fatal_error(name->getFullName() + " could not be resolved.");
        }
        return def;
    }
    return re;
}

RE * UnicodeNameResolver::transformRange(Range * rg) {
    RE * const x = getDefinitionOf(transform(rg->getLo()));
    RE * const y = getDefinitionOf(transform(rg->getHi()));
    return makeRange(x, y);
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
: RE_Transformer("Anchor Resolution") {
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
    RE * sot = makeNegativeLookBehindAssertion(makeByte(0x00,0xFF));
    return makeAlt({sot, makeLookBehindAssertion(mAnchorRE)});
}

RE * AnchorResolution::transformEnd(End * e) {
    if (mIsNegated) return makeNegativeLookAheadAssertion(mAnchorRE);
    RE * eot = makeNegativeLookAheadAssertion(makeByte(0x00,0xFF));
    return makeAlt({eot, makeLookAheadAssertion(mAnchorRE)});
}

RE * resolveAnchors(RE * r, RE * breakRE) {
    return AnchorResolution(breakRE).transformRE(r);
}
                                                        
}
