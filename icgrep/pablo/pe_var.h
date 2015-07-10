/*
 *  Copyright (c) 2014 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#ifndef PE_VAR_H
#define PE_VAR_H

#include <pablo/pabloAST.h>
#include <pablo/ps_assign.h>
#include <pablo/pe_next.h>
#include <pablo/pe_string.h>
#include <stdexcept>

namespace llvm {
    class Value;
    class BasicBlock;
}

namespace pablo {

class Var : public PabloAST {
    friend class PabloBlock;
    friend class And;
    friend class Or;
    friend class Not;
    friend class Sel;
    friend class Xor;
    friend class Zeroes;
    friend class Ones;
public:
    static inline bool classof(const PabloAST * e) {
        return e->getClassTypeId() == ClassTypeId::Var;
    }
    static inline bool classof(const void *) {
        return false;
    }
    virtual ~Var(){
    }
    inline const String * getName() const {
        return mName;
    }
protected:
    Var(PabloAST * var)
    : PabloAST(ClassTypeId::Var)
    , mName(cast<String>(var)) {

    }
private:
    const String *     mName;
};

}



#endif // PE_VAR_H


