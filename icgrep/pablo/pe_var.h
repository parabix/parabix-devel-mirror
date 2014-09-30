/*
 *  Copyright (c) 2014 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#ifndef PE_VAR_H
#define PE_VAR_H

#include "pe_pabloe.h"
#include <string>

namespace pablo {

class Var : public PabloE {
    friend Var * make_var(const std::string);
public:
    static inline bool classof(const PabloE * e) {
        return e->getClassTypeId() == ClassTypeId::Var;
    }
    static inline bool classof(const void *) {
        return false;
    }
    virtual ~Var(){

    }
    inline const std::string & getVar() const {
        return mVar;
    }
protected:
    Var(const std::string var)
    : PabloE(ClassTypeId::Var)
    , mVar(var)
    {

    }
private:
    const std::string mVar;
};

inline Var * make_var(const std::string var) {
    return new Var(var);
}

}



#endif // PE_VAR_H


