#ifndef ARITHMETIC_H
#define ARITHMETIC_H

#include <pablo/pabloAST.h>

namespace pablo {

class Operator : public PabloAST {
    friend class PabloBlock;
public:
    static inline bool classof(const PabloAST * e) {
        switch (e->getClassTypeId()) {
            case ClassTypeId::Add:
            case ClassTypeId::Subtract:
            case ClassTypeId::LessThan:
            case ClassTypeId::LessThanEquals:
            case ClassTypeId::Equals:
            case ClassTypeId::GreaterThanEquals:
            case ClassTypeId::GreaterThan:
            case ClassTypeId::NotEquals:
                return true;
            default:
                return false;
        }
    }
    static inline bool classof(const void *) {
        return false;
    }
    PabloAST * getLH() const {
        return mOperand[0];
    }
    PabloAST * getRH() const {
        return mOperand[1];
    }
    void replaceUsesOfWith(PabloAST * const from, PabloAST * const to, const bool recursive = false) final;
    virtual ~Operator() { }
protected:
    Operator(const ClassTypeId typeId, llvm::Type * const type, PabloAST * const expr1, PabloAST * const expr2, Allocator & allocator)
    : PabloAST(typeId, type, allocator)
    , mOperand({expr1, expr2}) {
        mOperand[0]->addUser(this);
        mOperand[1]->addUser(this);
    }
private:
    std::array<PabloAST *, 2> mOperand;
};

#define CREATE_OPERATOR_TYPE(Name) \
class Name final : public Operator { \
    friend class PabloBlock; \
public: \
    static inline bool classof(const PabloAST * e) { \
        return e->getClassTypeId() == ClassTypeId::Name; \
    } \
protected: \
    Name(llvm::Type * const type, PabloAST * const expr1, PabloAST * const expr2, Allocator & allocator) \
    : Operator(ClassTypeId::Name, type, expr1, expr2, allocator) { \
    } \
};

CREATE_OPERATOR_TYPE(Add)
CREATE_OPERATOR_TYPE(Subtract)
CREATE_OPERATOR_TYPE(LessThan)
CREATE_OPERATOR_TYPE(LessThanEquals)
CREATE_OPERATOR_TYPE(Equals)
CREATE_OPERATOR_TYPE(GreaterThanEquals)
CREATE_OPERATOR_TYPE(GreaterThan)
CREATE_OPERATOR_TYPE(NotEquals)

#undef CREATE_OPERATOR_TYPE

}

#endif // ARITHMETIC_H
