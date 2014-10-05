/*
 *  Copyright (c) 2014 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#ifndef PE_VAR_H
#define PE_VAR_H

#include <pablo/pe_pabloe.h>
#include <pablo/ps_assign.h>
#include <string>

namespace pablo {

class Var : public PabloE {
    friend Var * makeVar(const std::string);
public:
    static inline bool classof(const PabloE * e) {
        return e->getClassTypeId() == ClassTypeId::Var;
    }
    static inline bool classof(const void *) {
        return false;
    }
    virtual ~Var(){

    }
    inline const std::string & getName() const {
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

inline Var * makeVar(const std::string var) {
    return new Var(var);
}

inline Var * makeVar(const Assign * assign) {
    return makeVar(assign->getName());
}

}



#endif // PE_VAR_H


