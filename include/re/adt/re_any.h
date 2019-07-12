/*
 *  Copyright (c) 2017 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#ifndef ANY_H
#define ANY_H

#include <llvm/Support/Casting.h>
#include <re/adt/re_re.h>
#include <re/adt/re_cc.h>
#include <re/adt/re_name.h>
#include <ucd/core/unicode_set.h>

namespace re {

class Any {
public:
    static inline bool classof(const RE * re) {
        if (llvm::isa<Name>(re)) {
            re = llvm::cast<Name>(re)->getDefinition();
            if (re == nullptr) return false;
        }
        return llvm::isa<CC>(re) && llvm::cast<CC>(re)->full();
    }
    static inline bool classof(const void *) {
        return false;
    }
private:
    Any() {}
};

inline RE * makeAny() {
    Name * dot = makeName(".", Name::Type::UnicodeProperty);
    dot->setDefinition(makeCC(0, 0x10FFFF));
    return dot;
}

}

#endif // ANY_H
