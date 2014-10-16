/*
 *  Copyright (c) 2014 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#ifndef PE_CHARCLASS_H
#define PE_CHARCLASS_H

#include <pablo/pabloAST.h>
#include <pablo/pe_string.h>

namespace pablo {

class CharClass : public PabloAST {
    friend class PabloBlock;
public:
    static inline bool classof(const PabloAST * e) {
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
    CharClass(const PabloAST * cc)
    : PabloAST(ClassTypeId::CharClass)
    , mCharClass(cast<String>(cc))
    {

    }
private:
    const String * const mCharClass;
};

}

#endif // PE_CHARCLASS_H


