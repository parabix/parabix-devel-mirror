#ifndef BRANCH_H
#define BRANCH_H

#include <pablo/pabloAST.h>
namespace pablo { class Var; }
namespace pablo { class PabloBlock; }

namespace pablo {

class Branch : public Statement {
    friend class PabloBlock;
    friend class Statement;
    friend class Simplifier;
public:
    using EscapedVars = std::vector<Var *>;
    static inline bool classof(const PabloAST * e) {
        switch (e->getClassTypeId()) {
            case ClassTypeId::If:
            case ClassTypeId::While:
                return true;
            default:
                return false;
        }
    }
    static inline bool classof(const void *) {
        return false;
    }
    inline PabloAST * getCondition() const {
        return getOperand(0);
    }
    inline void setCondition(PabloAST * const condition) {
        return setOperand(0, condition);
    }
    inline PabloBlock * getBody() {
        return mBody;
    }
    inline  PabloBlock * getBody() const {
        return mBody;
    }
    PabloBlock * setBody(PabloBlock * const body);
    EscapedVars getEscaped() const;
protected:
    void addEscaped(PabloAST * const node);
    void removeEscaped(PabloAST * const node);
    Branch(const ClassTypeId typeId, PabloAST * condition, PabloBlock * body, Allocator & allocator);
protected:
    PabloBlock *    mBody;
    unsigned        mEscapedCount;
    unsigned        mEscapedCapacity;
    PabloAST **     mEscaped;
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
