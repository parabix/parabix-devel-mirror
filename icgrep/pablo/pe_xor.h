/*
 *  Copyright (c) 2014 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#ifndef XOR_H
#define XOR_H

#include <pablo/pabloAST.h>

namespace pablo {

class Xor : public Variadic {
    friend class PabloBlock;
public:
    static inline bool classof(const PabloAST * e) {
        return e->getClassTypeId() == ClassTypeId::Xor;
    }
    static inline bool classof(const void *) {
        return false;
    }
    virtual ~Xor() { }
protected:
    Xor(const PabloType * const type, PabloAST * expr1, PabloAST * expr2, String * name)
    : Variadic(ClassTypeId::Xor, type, {expr1, expr2}, name)
    {

    }
    Xor(const PabloType * const type, const unsigned reserved, String * name)
    : Variadic(ClassTypeId::Xor, type, reserved, name)
    {

    }
    template<typename iterator>
    Xor(const PabloType * const type, iterator begin, iterator end, String * name)
    : Variadic(ClassTypeId::Xor, type, begin, end, name) {

    }
};

}

#endif // XOR_H



