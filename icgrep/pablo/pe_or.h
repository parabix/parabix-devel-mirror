/*
 *  Copyright (c) 2014 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#ifndef PE_OR_H
#define PE_OR_H

#include <pablo/pabloAST.h>

namespace pablo {

class Or : public Variadic {
    friend class PabloBlock;
public:
    static inline bool classof(const PabloAST * e) {
        return e->getClassTypeId() == ClassTypeId::Or;
    }
    static inline bool classof(const void *) {
        return false;
    }
    virtual ~Or() { }
protected:
    Or(PabloAST * expr1, PabloAST * expr2, String * name)
    : Variadic(ClassTypeId::Or, {expr1, expr2}, name)
    {

    }
    Or(const unsigned reserved, String * name)
    : Variadic(ClassTypeId::Or, reserved, name)
    {

    }
    template<typename iterator>
    Or(iterator begin, iterator end, String * name)
    : Variadic(ClassTypeId::Or, begin, end, name) {

    }
};

}

#endif // PE_OR_H



