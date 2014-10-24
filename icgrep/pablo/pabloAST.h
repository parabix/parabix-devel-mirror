/*
 *  Copyright (c) 2014 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#ifndef PE_PabloAST_H
#define PE_PabloAST_H

#include <llvm/Support/Casting.h>
#include <llvm/IR/Value.h>
#include <vector>

using namespace llvm;

namespace pablo {

class PabloAST {
public:
    enum class ClassTypeId : unsigned {
        Advance
        , And
        , Assign
        , Call
        , CharClass
        , If
        , MatchStar
        , Next
        , Not
        , Ones
        , Or
        , ScanThru
        , Sel
        , String
        , Var
        , While
        , Xor
        , Zeroes
    };
    inline ClassTypeId getClassTypeId() const {
        return mClassTypeId;
    }
protected:
    inline PabloAST(const ClassTypeId id)
    : mClassTypeId(id) {

    }
private:
    const ClassTypeId   mClassTypeId;
};

bool equals(const PabloAST * expr1, const PabloAST *expr2);

typedef std::vector<PabloAST *> StatementList;

}

#endif // PE_PabloAST_H



