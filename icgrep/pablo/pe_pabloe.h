/*
 *  Copyright (c) 2014 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#ifndef PE_PABLOE_H
#define PE_PABLOE_H

#include <llvm/Support/Casting.h>
#include <llvm/IR/Value.h>
#include <list>

using namespace llvm;

namespace pablo {

class PabloE {
public:
    enum class ClassTypeId : unsigned {
        Advance
        , All
        , And
        , Call
        , CharClass
        , MatchStar
        , Not
        , Or
        , ScanThru
        , Sel
        , Var
        , Xor
        , Assign
        , If
        , While
        , String
    };
    inline ClassTypeId getClassTypeId() const {
        return mClassTypeId;
    }
    virtual ~PabloE() = 0;
protected:
    inline PabloE(const ClassTypeId id)
    : mClassTypeId(id) {

    }
private:
    const ClassTypeId   mClassTypeId;
};

bool equals(const PabloE * expr1, const PabloE *expr2);

typedef std::list<PabloE *> ExpressionList;

}

#endif // PE_PABLOE_H



