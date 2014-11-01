#ifndef PE_NEXT_H
#define PE_NEXT_H

#include <pablo/pabloAST.h>
#include <pablo/ps_assign.h>
#include <array>

namespace pablo {

class Assign;

class Next : public Statement {
    friend class PabloBlock;
public:
    static inline bool classof(const PabloAST * e) {
        return e->getClassTypeId() == ClassTypeId::Next;
    }
    static inline bool classof(const void *) {
        return false;
    }
    virtual PabloAST * getOperand(const unsigned index) const {
        assert (index < 2);
        return mExprs[index];
    }
    virtual unsigned getNumOperands() const {
        return 2;
    }
    virtual void setOperand(const unsigned index, PabloAST * value) {
        assert (index < 2);
        assert (index == 0 || isa<Assign>(value));
        mExprs[index] = value;
    }
    inline const Assign * getInitial() const {
        return cast<const Assign>(mExprs[0]);
    }
    inline const String * getName() const {
        return getInitial()->getName();
    }
    inline PabloAST * getExpr() const {
        return mExprs[1];
    }
protected:
    Next(PabloAST * initial, PabloAST * expr, StatementList * parent)
    : Statement(ClassTypeId::Next, parent)
    , mExprs({cast<Assign>(initial), expr})
    {

    }
private:
    std::array<PabloAST*, 2>  mExprs;
};

}


#endif // PE_NEXT_H
