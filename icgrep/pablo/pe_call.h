#ifndef PE_CALL_H
#define PE_CALL_H

#include <pablo/pabloAST.h>
#include <pablo/pe_string.h>
#include <iostream>

namespace pablo {

class Call : public PabloAST {
    friend class PabloBlock;
public:
    static inline bool classof(const PabloAST * e) {
        return e->getClassTypeId() == ClassTypeId::Call;
    }
    static inline bool classof(const void *) {
        return false;
    }
    virtual ~Call() {
    }
    virtual PabloAST * getOperand(const unsigned index) const {
        assert (index == 0);
        return mCallee;
    }
    virtual unsigned getNumOperands() const {
        return 1;
    }
    virtual void setOperand(const unsigned index, PabloAST * value) {
        assert (index == 0);
        assert (isa<String>(value));
        mCallee = cast<String>(value);
    }
    inline const String * getCallee() const {
        return mCallee;
    }
protected:
    void* operator new (std::size_t size) noexcept {
        return mAllocator.allocate(size);
    }
    Call(PabloAST * callee)
    : PabloAST(ClassTypeId::Call)
    , mCallee(cast<String>(callee)) {

    }
private:
    String * mCallee;
};
}

#endif // PE_CALL_H


