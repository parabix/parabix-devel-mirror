/*
 *  Copyright (c) 2018 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#ifndef RE_EMPTY_SET_H
#define RE_EMPTY_SET_H

namespace re {

class RE;

bool isEmptySet(RE * re);

RE * makeEmptySet();

}

#endif
