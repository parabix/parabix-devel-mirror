/*
 *  Copyright (c) 2014 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#ifndef PS_SETMARKER_H
#define PS_SETMARKER_H

#include <pablo/pe_string.h>

namespace pablo {

class Assign : public Statement {
    friend class PabloBlock;
    friend class Next;
    friend class Var;
public:
    static inline bool classof(const PabloAST * e) {
        return e->getClassTypeId() == ClassTypeId::Assign;
    }
    static inline bool classof(const void *) {
        return false;
    }
    virtual ~Assign() {
    }
    inline PabloAST * getExpr() const {
        return getOperand(0);
    }
    inline bool isOutputAssignment() const {
        return mOutputIndex >= 0;
    }
    inline int getOutputIndex() const {
        return mOutputIndex;
    }
    bool superfluous() const;
protected:
    explicit Assign(PabloAST * expr, int outputIndex, String * name, PabloBlock * parent)
    : Statement(ClassTypeId::Assign, {expr}, name, parent)
    , mOutputIndex(outputIndex)
    {

    }
private:
    const int           mOutputIndex;
};

}

#endif // PS_SETMARKER_H

