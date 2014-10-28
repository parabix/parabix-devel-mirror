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
    inline const PabloAST * getVar() const {
        return mVar;
    }
    inline bool isInternal() const {
        return mVar != mName;
    }
    inline bool isExternal() const {
        return mVar == mName;
    }
protected:
    void* operator new (std::size_t size) noexcept {
        return mAllocator.allocate(size);
    }
    Var(const PabloAST * var)
    : PabloAST(ClassTypeId::Var)
    , mVar(var)
    , mName(getNameOf(var))
    {

    }
private:
    static inline const String * getNameOf(const PabloAST * var) {
        if (isa<String>(var)) {
            return cast<String>(var);
        }
        if (isa<Assign>(var)) {
            return cast<Assign>(var)->getName();
        }
        if (isa<Next>(var)) {
            return cast<Next>(var)->getName();
        }
        throw std::runtime_error("Pablo Var only accepts String, Assign and Next nodes.");
    }
private:
    const PabloAST * const  mVar;
    const String * const    mName;
};

}



#endif // PE_VAR_H


