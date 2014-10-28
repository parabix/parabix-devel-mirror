/*
 *  Copyright (c) 2014 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#ifndef PS_SETMARKER_H
#define PS_SETMARKER_H

#include <pablo/pe_string.h>

namespace pablo {

class Assign : public PabloAST {
    friend class PabloBlock;
    friend class Next;
public:
    static inline bool classof(const PabloAST * e) {
        return e->getClassTypeId() == ClassTypeId::Assign;
    }
    static inline bool classof(const void *) {
        return false;
    }
    virtual ~Assign() {
    }
    inline const String * getName() const {
        return mName;
    }
    inline PabloAST * getExpr() const {
        return mExpr;
    }
    inline bool isOutputAssignment() const {
        return mOutputIndex >= 0;
    }
    inline int getOutputIndex() const {
        return mOutputIndex;
    }
protected:
    void* operator new (std::size_t size) noexcept {
        return mAllocator.allocate(size);
    }
    Assign(PabloAST * name, PabloAST * expr, const int outputIndex)
    : PabloAST(ClassTypeId::Assign)
    , mName(cast<String>(name))
    , mExpr(expr)
    , mOutputIndex(outputIndex)
    {

    }
private:
    String * const          mName;
    PabloAST * const        mExpr;
    const int               mOutputIndex;
};

}

#endif // PS_SETMARKER_H

