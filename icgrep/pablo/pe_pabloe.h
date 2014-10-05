/*
 *  Copyright (c) 2014 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#ifndef PE_PABLOE_H
#define PE_PABLOE_H

#include <llvm/Support/Casting.h>

using namespace llvm;

namespace pablo {

class PabloE
{
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
    const ClassTypeId mClassTypeId;
};

bool equals(const PabloE * expr1, const PabloE *expr2);

//template <typename To, typename From>
//inline static bool isa(const From * object) {
//    return To::classof(object);
//}

//template <typename To, typename From>
//inline static To * cast(From * object) {
//    return reinterpret_cast<To *>(object);
//}

//template <typename To, typename From>
//inline static To * dyn_cast(From * object) {
//    if (isa<To, From>(object)) {
//        return cast<To, From>(object);
//    }
//    return nullptr;
//}

}

#endif // PE_PABLOE_H



