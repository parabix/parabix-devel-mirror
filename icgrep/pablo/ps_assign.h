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
    inline bool isOutput() const {
        return mIsOutput;
    }
protected:
    Assign(PabloAST * name, PabloAST * expr, const bool isOutput)
    : PabloAST(ClassTypeId::Assign)
    , mName(cast<String>(name))
    , mExpr(expr)
    , mIsOutput(isOutput)
    {

    }
private:
    String * const          mName;
    PabloAST * const        mExpr;
    const bool              mIsOutput;
};

}

#endif // PS_SETMARKER_H

