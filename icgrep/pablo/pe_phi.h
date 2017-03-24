#ifndef PE_PHI_H
#define PE_PHI_H

#include <pablo/pabloAST.h>

namespace pablo {

class Phi : public PabloAST {
    friend class PabloBlock;
public:
    static inline bool classof(const PabloAST * e) {
        return e->getClassTypeId() == ClassTypeId::Phi;
    }
    static inline bool classof(const void *) {
        return false;
    }
    virtual ~Phi(){
    }
    void addIncomingValue(PabloAST * const value) {
        assert (getType() == value->getType());
        for (unsigned i = 0; i < getNumIncomingValues(); ++i) {
            if (LLVM_UNLIKELY(getIncomingValue(i) == value)) {
                return;
            }
        }
        assert (mNumIncomingValues < 2);
        mIncomingValue[mNumIncomingValues++] = value;
    }
    PabloAST * getIncomingValue(const unsigned i) const {
        assert (i < mNumIncomingValues);
        return mIncomingValue[i];
    }
    unsigned getNumIncomingValues() const {
        return mNumIncomingValues;
    }
protected:
    Phi(llvm::Type * type, Allocator & allocator)
    : PabloAST(ClassTypeId::Phi, type, allocator)
    , mNumIncomingValues(0) {

    }
private:
    unsigned   mNumIncomingValues;
    PabloAST * mIncomingValue[2];
};

}

#endif
