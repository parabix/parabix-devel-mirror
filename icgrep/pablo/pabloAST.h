/*
 *  Copyright (c) 2014 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#ifndef PE_PabloAST_H
#define PE_PabloAST_H

#include <llvm/Support/Casting.h>
#include <llvm/IR/Value.h>
#include <list>

using namespace llvm;

namespace pablo {

class PabloAST {
public:
    enum class ClassTypeId : unsigned {
        Advance
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
        , Zeroes
        , Ones
    };
    inline ClassTypeId getClassTypeId() const {
        return mClassTypeId;
    }
    virtual ~PabloAST() = 0;
protected:
    inline PabloAST(const ClassTypeId id)
    : mClassTypeId(id) {

    }
private:
    const ClassTypeId   mClassTypeId;
};

bool equals(const PabloAST * expr1, const PabloAST *expr2);

typedef std::list<PabloAST *> ExpressionList;

}

#endif // PE_PabloAST_H



