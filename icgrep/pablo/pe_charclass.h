/*
 *  Copyright (c) 2014 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#ifndef PE_CHARCLASS_H
#define PE_CHARCLASS_H

#include "pe_pabloe.h"
#include <string>

namespace pablo {

class CharClass : public PabloE {
    friend CharClass * makeCharClass(const std::string cc);
public:
    static inline bool classof(const PabloE * e) {
        return e->getClassTypeId() == ClassTypeId::CharClass;
    }
    static inline bool classof(const void *) {
        return false;
    }
    virtual ~CharClass(){

    }
    inline const std::string & getCharClass() const {
        return mCharClass;
    }
protected:
    CharClass(const std::string charClass)
    : PabloE(ClassTypeId::CharClass)
    , mCharClass(charClass)
    {

    }
private:
    const std::string mCharClass;
};

inline CharClass * makeCharClass(const std::string cc) {
    return new CharClass(cc);
}

}

#endif // PE_CHARCLASS_H


