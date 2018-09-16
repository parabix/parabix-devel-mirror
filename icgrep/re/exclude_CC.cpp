/*
 *  Copyright (c) 2017 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#include <re/exclude_CC.h>
#include <re/re_cc.h>
#include <re/re_name.h>
#include <re/re_start.h>
#include <re/re_end.h>
#include <re/re_any.h>
#include <re/re_seq.h>
#include <re/re_alt.h>
#include <re/re_rep.h>
#include <re/re_group.h>
#include <re/re_range.h>
#include <re/re_diff.h>
#include <re/re_intersect.h>
#include <re/re_assertion.h>
#include <re/re_utility.h>
#include <llvm/Support/Casting.h>
#include <llvm/Support/ErrorHandling.h>

using namespace llvm;

namespace re {
 
class CC_Remover : public RE_Transformer {
public:
    CC_Remover(CC * toExclude) : RE_Transformer(), mExcludedCC(toExclude) {}
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
        case Name::Type::Reference:
        case Name::Type::ZeroWidth:
            return n;
        case Name::Type::Capture:
            return makeCapture(n->getName(), transform(n->getDefinition()));
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
    return CC_Remover(cc).transform(re);
}
}

