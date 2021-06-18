/*
 *  Copyright (c) 2018 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#ifndef RE_CONTEXTUAL_SIMPLIFICATION_H
#define RE_CONTEXTUAL_SIMPLIFICATION_H

#include <re/transforms/re_transformer.h>

namespace re {

RE * simplifyAssertions(RE * r);

class RE_ContextSimplifier : public RE_Transformer {
public:
    inline RE_ContextSimplifier() : RE_Transformer("ContextSimplification") {}
    RE * transformSeq(Seq * s) override;
};

}

#endif
