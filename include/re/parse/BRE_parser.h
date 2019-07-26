/*
 *  Copyright (c) 2018 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#ifndef BRE_PARSER_H
#define BRE_PARSER_H

#include <re/parse/parser.h>
#include <re/parse/ERE_parser.h>

namespace re {
    class BRE_Parser : public ERE_Parser  {
    public:
        BRE_Parser(const std::string & regular_expression) : ERE_Parser(regular_expression) {
            mReSyntax = RE_Syntax::BRE;
        }

    protected:
        RE * parse_alt() override;
        RE * parse_seq() override;
        RE * parse_next_item() override ;
        RE * extend_item(RE * re) override;
        RE * parse_group() override;
        std::pair<int, int> parse_range_bound() override;
    };
}

#endif
