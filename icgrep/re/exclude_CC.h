/*
 *  Copyright (c) 2017 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#ifndef EXCLUDE_CC_H
#define EXCLUDE_CC_H

namespace re {
    class RE;
    class CC;
    
    /* Transform a regular expression r so that matched strings do not include
       matches to any character within the given character class cc.
       (However, do not transform assertions, so that lookahead or lookbehind
        may still require matches to cc.  */
    RE * exclude_CC(RE * r, CC * cc);
}

#endif // EXCLUDE_CC_H
