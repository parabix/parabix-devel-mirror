/*
 *  Copyright (c) 2018 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#ifndef TO_UTF8_H
#define TO_UTF8_H

namespace re {
class RE;

RE * toUTF8(RE * r, bool convertName = false);

}

#endif // TO_UTF8_H
