#ifndef BRANCH_H
#define BRANCH_H

#include <pablo/pabloAST.h>
#include <llvm/ADT/SmallVector.h>

namespace pablo { class Var; }
namespace pablo { class PabloBlock; }

namespace pablo {

class Branch : public Statement {
    friend class PabloBlock;
    friend class Statement;
    friend class Simplifier;
public:
    using EscapedVars = llvm::SmallVector<Var *, 16>;
    static bool classof(const PabloAST * e) {
        switch (e->getClassTypeId()) {
            case ClassTypeId::If:
            case ClassTypeId::While:
                return true;
            default:
                return false;
        }
    }
    static bool classof(const void *) {
        return false;
    }
    PabloAST * getCondition() const {
        return getOperand(0);
    }
    void setCondition(PabloAST * const condition) {
        return setOperand(0, condition);
    }
    PabloBlock * getBody() const {
        return mBody;
    }
    bool isRegular() const {
        return mRegular;
    }
    void setRegular(const bool value) {
        mRegular = value;
    }
    PabloBlock * setBody(PabloBlock * const body);
    EscapedVars getEscaped() const;
protected:
    Branch(const ClassTypeId typeId, PabloAST * condition, PabloBlock * body, Allocator & allocator);
protected:
    PabloBlock *    mBody;
    bool            mRegular;
};

class If final : public Branch {
    friend class PabloBlock;
    friend class Statement;
    friend class Simplifier;
public:
    static inline bool classof(const PabloAST * e) {
        return e->getClassTypeId() == ClassTypeId::If;
    }
protected:
    If(PabloAST * condition, PabloBlock * body, Allocator & allocator)
    : Branch(ClassTypeId::If, condition, body, allocator) {

    }
};

class While final : public Branch {
    friend class PabloBlock;
    friend class Statement;
    friend class Simplifier;
public:
    static inline bool classof(const PabloAST * e) {
        return e->getClassTypeId() == ClassTypeId::While;
    }
protected:
    While(PabloAST * condition, PabloBlock * body, Allocator & allocator)
    : Branch(ClassTypeId::While, condition, body, allocator) {

    }
};


}

#endif // BRANCH_H
