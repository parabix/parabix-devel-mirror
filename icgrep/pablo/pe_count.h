/*
 *  Copyright (c) 2015 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#ifndef PE_COUNT_H
#define PE_COUNT_H

#include <pablo/pabloAST.h>

namespace pablo {

class Count : public Statement {
    friend class PabloBlock;
public:
    static inline bool classof(const PabloAST * e) {
        return e->getClassTypeId() == ClassTypeId::Count;
    }
    static inline bool classof(const void *) {
        return false;
    }
    virtual ~Count() {
    }
    inline PabloAST * getExpr() const {
        return getOperand(0);
    }
    inline void setGlobalCountIndex(const unsigned idx) {
        globalCountIndex = idx;
    }
    inline unsigned getGlobalCountIndex() const {
        return globalCountIndex;
    }
protected:
    Count(PabloAST * expr, String * name)
    : Statement(ClassTypeId::Count, {expr}, name)
    {

    }
private:
    unsigned globalCountIndex;
};

}

#endif // PE_COUNT_H



