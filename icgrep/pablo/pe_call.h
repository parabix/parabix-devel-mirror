#ifndef PE_CALL_H
#define PE_CALL_H

#include <pablo/pabloAST.h>
#include <pablo/function.h>

namespace pablo {

class Call : public Statement {
    friend class PabloBlock;
public:
    static inline bool classof(const PabloAST * e) {
        return e->getClassTypeId() == ClassTypeId::Call;
    }
    static inline bool classof(const void *) {
        return false;
    }
    virtual ~Call() { }

    inline const Prototype * getPrototype() const {
        return cast<Prototype>(getOperand(0));
    }
    inline const String * getCallee() const {
        return getPrototype()->getName();
    }
    inline void setLocalCarryIndex(const unsigned idx) {
        mLocalCarryIndex = idx;
    }
    inline unsigned getLocalCarryIndex() const {
        return mLocalCarryIndex;
    }
protected:
    Call(PabloAST * prototype)
    : Statement(ClassTypeId::Call, {prototype}, cast<Prototype>(prototype)->getName())
    , mLocalCarryIndex(0) {

    }
private:
    unsigned mLocalCarryIndex;
};
}

#endif // PE_CALL_H


