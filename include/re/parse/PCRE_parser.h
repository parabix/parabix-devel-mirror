/*
 *  Copyright (c) 2018 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#ifndef PCRE_PARSER_H
#define PCRE_PARSER_H

#include <re/parse/parser.h>

namespace re {
    class PCRE_Parser : public RE_Parser {
    public:
        PCRE_Parser(const std::string & regular_expression) : RE_Parser(regular_expression) {
            mReSyntax = RE_Syntax::PCRE;
        }
    protected:
        virtual bool isSetEscapeChar(char c) override;
    };
}

#endif
