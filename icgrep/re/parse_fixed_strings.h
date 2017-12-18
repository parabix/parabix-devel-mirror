/*
 *  Copyright (c) 2017 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#ifndef PARSE_FIXED_STRINGS_H
#define PARSE_FIXED_STRINGS_H
#include <re/re_parser.h>

namespace re {
    class FixedStringParser : public RE_Parser {
    public:
        FixedStringParser(const std::string & regular_expression) : RE_Parser(regular_expression) {
            mReSyntax = RE_Syntax::FixedStrings;
        }
        RE * parse_alt () override;
        RE * parse_seq () override;
    };
}

#endif //PARSE_FIXED_STRINGS_H
