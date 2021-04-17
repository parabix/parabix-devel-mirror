#include <re/unicode/re_name_resolve.h>

#include <boost/container/flat_set.hpp>
#include <llvm/Support/ErrorHandling.h>
#include <re/adt/adt.h>
#include <re/alphabet/alphabet.h>
#include <re/analysis/re_analysis.h>
#include <re/transforms/re_transformer.h>
#include <re/unicode/resolve_properties.h>


using namespace boost::container;
using namespace llvm;

namespace re {

    
class EscapeNameResolver final : public RE_Transformer {
public:
    EscapeNameResolver(NameStandard c) : RE_Transformer("EscapeNames"), mStandard(c) {}
    RE * transformName(Name * name) override {
        std::string theName = name->getFullName();
        if (theName == "\\d") {
            if (mStandard == NameStandard::Posix) return makeRange(makeCC(0x30), makeCC(0x39));
            return makePropertyExpression(PropertyExpression::Kind::Codepoint, "Nd");
        }
        if (theName == "\\s") {
            return makePropertyExpression(PropertyExpression::Kind::Codepoint, "whitespace");
        }
        if (theName == "\\w") {
            return makePropertyExpression(PropertyExpression::Kind::Codepoint, "word");
        }
        if (theName == "\\b") {
            return makePropertyExpression(PropertyExpression::Kind::Boundary, "word");
        }
        return name;
    }
private:
    NameStandard mStandard;
};
    
RE * resolveEscapeNames(RE * re, NameStandard c) {
    return EscapeNameResolver(c).transformRE(re);
}
    

class AnchorResolution : public RE_Transformer {
public:
    AnchorResolution(RE * anchorRE) : RE_Transformer("Anchor Resolution"), mAnchorRE(anchorRE) {}
    RE * transformStart(Start * s) override {
        return makeAlt({s, makeLookBehindAssertion(mAnchorRE)});
    }
    RE * transformEnd(End * s) override{
        return makeAlt({s, makeLookAheadAssertion(mAnchorRE)});
    }
private:
    RE * mAnchorRE;
};
 
RE * resolveAnchors(RE * r, RE * breakRE) {
    return AnchorResolution(breakRE).transformRE(r);
}
                                                        
}
