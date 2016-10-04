/*
 *  Copyright (c) 2016 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#ifndef ICGREP_RE_PARSER_HELPER_H
#define ICGREP_RE_PARSER_HELPER_H

#define bit3C(x) (1ULL << ((x) - 0x3C))


// It would probably be best to enforce that {}, [], () must always
// be balanced.   But legacy syntax allows } and ] to occur as literals
// in certain contexts (no opening { or [, or immediately after [ or [^ ).
// Perhaps this define should become a parameter.
#define LEGACY_UNESCAPED_RBRAK_RBRACE_ALLOWED true
#define LEGACY_UNESCAPED_HYPHEN_ALLOWED true


#endif //ICGREP_RE_PARSER_HELPER_H
