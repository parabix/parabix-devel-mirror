/*
 *  Copyright (c) 2017 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#ifndef ANY_H
#define ANY_H

#include <re/adt/re_re.h>

namespace re {

class Any : public RE {
public:
    static Any * Create() {return new Any();}
    RE_SUBTYPE(Any)
private:
    Any() : RE(ClassTypeId::Any) {}
};

inline Any * makeAny() {return Any::Create();}

}

#endif // ANY_H
