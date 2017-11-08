/*
 *  Copyright (c) 2017 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#ifndef ANY_H
#define ANY_H

#include "re_re.h"
#include "re_cc.h"
#include <UCD/unicode_set.h>
#include <llvm/Support/Casting.h>

namespace re {

class Any : public RE {
public:
    static inline bool classof(const RE * re) {
        return (re->getClassTypeId() == ClassTypeId::CC) && llvm::cast<CC>(re)->full();
    }
    static inline bool classof(const void *) {
        return false;
    }
protected:
    Any() : RE(ClassTypeId::Any) {}
    virtual ~Any() {}
};

inline CC * makeAny() {
    return makeCC(0, UCD::UNICODE_MAX);
}

}

#endif // ANY_H
