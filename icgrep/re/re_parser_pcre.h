/*
 *  Copyright (c) 2016 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#ifndef ICGREP_RE_PARSER_PCRE_H
#define ICGREP_RE_PARSER_PCRE_H

#include <re/re_parser.h>

namespace re {
    class RE_Parser_PCRE : public RE_Parser {
    public:
        RE_Parser_PCRE(const std::string & regular_expression) : RE_Parser(regular_expression) {
            mReSyntax = RE_Syntax ::PCRE;
        }
    protected:
        virtual bool isSetEscapeChar(char c) override;
    };
}

#endif //ICGREP_RE_PARSER_PCRE_H
