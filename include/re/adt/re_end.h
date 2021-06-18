/*
 *  Copyright (c) 2018 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#ifndef END_H
#define END_H

#include <re/adt/re_re.h>

namespace re {

class End : public RE {
public:
    static End * Create() {return new End();}
    RE_SUBTYPE(End)
private:
    End() : RE(ClassTypeId::End) {}
};

inline End * makeEnd() {return End::Create();}

}

#endif // END_H
