/*
 *  Copyright (c) 2018 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#include <re/adt/re_assertion.h>

#include <re/adt/re_start.h>
#include <re/adt/re_end.h>
#include <re/adt/nullable.h>

using namespace llvm;

namespace re {
    
RE * makeSOT () {
    //return makeNegativeLookBehindAssertion(makeByte(0x00,0xFF));
    return makeStart();
}

RE * makeEOT () {
    //return makeNegativeLookAheadAssertion(makeByte(0x00,0xFF));
    return makeEnd();
}

}
