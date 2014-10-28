/*
 *  Copyright (c) 2014 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#ifndef PE_PabloAST_H
#define PE_PabloAST_H

#include <llvm/Support/Casting.h>
#include <slab_allocator.h>
#include <vector>

using namespace llvm;

namespace pablo {

class PabloBlock;

class PabloAST {
public:
    typedef SlabAllocator<1024> Allocator;
    enum class ClassTypeId : unsigned {
        Advance
        , And
        , Assign
        , Call
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
    inline static void release_memory() {
        mAllocator.release_memory();
    }
protected:
    inline PabloAST(const ClassTypeId id)
    : mClassTypeId(id)
    {

    }
    static Allocator mAllocator;
private:
    const ClassTypeId   mClassTypeId;
};

bool equals(const PabloAST * expr1, const PabloAST *expr2);

typedef std::vector<PabloAST *> StatementList;

}

#endif // PE_PabloAST_H



