/*
 *  Copyright (c) 2014 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#ifndef PE_VAR_H
#define PE_VAR_H

#include <pablo/pabloAST.h>
#include <pablo/ps_assign.h>
#include <pablo/pe_string.h>

namespace pablo {

class Var : public PabloAST {
    friend class PabloBlock;
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
    Var(const PabloAST * var)
    : PabloAST(ClassTypeId::Var)
    , mName(cast<String>(var)) {

    }
private:
    const String * const mName;
};

}



#endif // PE_VAR_H


