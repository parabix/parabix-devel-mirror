#ifndef PE_NEXT_H
#define PE_NEXT_H

#include <pablo/pabloAST.h>
#include <pablo/ps_assign.h>

namespace pablo {

class Assign;

class Next : public PabloAST {
    friend class PabloBlock;
public:
    static inline bool classof(const PabloAST * e) {
        return e->getClassTypeId() == ClassTypeId::Next;
    }
    static inline bool classof(const void *) {
        return false;
    }
    inline const Assign * getInitial() const {
        return mInitial;
    }
    inline const std::string & getName() const {
        return mInitial->getName();
    }
    inline PabloAST * getExpr() const {
        return mExpr;
    }
protected:
    Next(const PabloAST * initial, PabloAST * expr)
    : PabloAST(ClassTypeId::Next)
    , mInitial(cast<Assign>(initial))
    , mExpr(expr)
    {

    }
private:
    const Assign * const      mInitial;
    PabloAST * const          mExpr;
};

}


#endif // PE_NEXT_H
