/*
 *  Copyright (c) 2017 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#include <re/transforms/exclude_CC.h>

#include <llvm/Support/Casting.h>
#include <llvm/Support/ErrorHandling.h>
#include <re/adt/adt.h>
#include <re/transforms/re_transformer.h>

using namespace llvm;

namespace re {
 
class CC_Remover : public RE_Transformer {
public:
    CC_Remover(CC * toExclude) : RE_Transformer("Exclude"), mExcludedCC(toExclude) {}
    RE * transformCC (CC * cc) override;
    RE * transformName (Name * name) override;
private:
    CC * mExcludedCC;
};
    
RE * CC_Remover::transformCC(CC * cc) {
    if (intersects(mExcludedCC, cc)) return subtractCC(cc, mExcludedCC);
    else return cc;
}

RE * CC_Remover::transformName(Name * n) {
    switch (n->getType()) {
        case Name::Type::ZeroWidth:
            return n;
        default:
            RE * defn = n->getDefinition();
            if (const CC * cc0 = dyn_cast<CC>(defn)) {
                if (!intersects(mExcludedCC, cc0)) return n;
            }
            std::string cc_name = n->getName() + "--" + mExcludedCC->canonicalName();
            return makeName(cc_name, Name::Type::Unicode, transform(defn));
            /*
             return transform(defn);
             */
    }
}
    
RE * exclude_CC(RE * re, CC * cc) {
    return CC_Remover(cc).transformRE(re);
}
}

