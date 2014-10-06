/*
 *  Copyright (c) 2014 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#ifndef PE_CHARCLASS_H
#define PE_CHARCLASS_H

#include <pablo/pe_pabloe.h>
#include <pablo/pe_string.h>

namespace pablo {

class CharClass : public PabloE {
    friend CharClass * makeCharClass(const String *);
    friend struct CodeGenState;
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
        return *mCharClass;
    }
protected:
    CharClass(const PabloE * cc)
    : PabloE(ClassTypeId::CharClass)
    , mCharClass(cast<String>(cc))
    {

    }
private:
    const String * const mCharClass;
};

inline CharClass * makeCharClass(const String * cc) {
    return new CharClass(cc);
}

}

#endif // PE_CHARCLASS_H


