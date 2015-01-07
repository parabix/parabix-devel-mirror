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
//    virtual PabloAST * getOperand(const unsigned index) const {
//        assert (index == 0);
//        return mVar;
//    }
//    virtual unsigned getNumOperands() const {
//        return 1;
//    }
//    virtual void setOperand(const unsigned index, PabloAST * value) {
//        assert (index == 0);
//        mVar = value;
//        mName = getNameOf(value);
//    }
    inline const String * getName() const {
        return mName;
    }
//    inline PabloAST * getVar() {
//        return mVar;
//    }
//    inline const PabloAST * getVar() const {
//        return mVar;
//    }
//    inline bool isInternal() const {
//        return mVar != mName;
//    }
//    inline bool isExternal() const {
//        return mVar == mName;
//    }
protected:
    Var(PabloAST * var, PabloBlock *)
    : PabloAST(ClassTypeId::Var)
    //, mVar(var)
    , mName(cast<String>(var)) // getNameOf(var))
    {
        var->addUser(this);
    }
private:
//    static inline const String * getNameOf(const PabloAST * var) {
//        if (isa<String>(var)) {
//            return cast<String>(var);
//        }
//        if (isa<Assign>(var)) {
//            return cast<Assign>(var)->getName();
//        }
//        if (isa<Next>(var)) {
//            return cast<Next>(var)->getName();
//        }
//        throw std::runtime_error("Pablo Var only accepts String, Assign and Next nodes.");
//    }
private:
    //PabloAST *         mVar;
    const String *     mName;
};

}



#endif // PE_VAR_H


