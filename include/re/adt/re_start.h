/*
 *  Copyright (c) 2014 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#ifndef START_H
#define START_H

#include <re/re_re.h>

namespace re {

class Start : public RE {
public:
    static Start * Create() {return new Start();}
    RE_SUBTYPE(Start)
private:
    Start() : RE(ClassTypeId::Start) {}
};

inline Start * makeStart() {return Start::Create();}

}

#endif // START_H

