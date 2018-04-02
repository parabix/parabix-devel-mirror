/*
 *  Copyright (c) 2018 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#ifndef ERE_PARSER_H
#define ERE_PARSER_H

#include <re/parsers/parser.h>

namespace re {
    class ERE_Parser : public RE_Parser  {
    public:
        ERE_Parser(const std::string & regular_expression) : RE_Parser(regular_expression) {
            mReSyntax = RE_Syntax::ERE;
        }

    protected:
       virtual RE * parse_next_item() override;

       virtual RE * parse_group() override;

       virtual RE * parse_escaped() override;
       
       RE * parse_bracket_expr();

       
    };
}

#endif
