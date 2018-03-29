/*
 *  Copyright (c) 2018 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#ifndef REPLACE_CC_H
#define REPLACE_CC_H


namespace re {

    class CC;
    class RE;

    RE * replaceCC(RE * re, CC * toReplace, RE * replacement);
}
#endif
