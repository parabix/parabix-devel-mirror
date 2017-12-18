/*
 *  Copyright (c) 2016 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#ifndef ICGREP_RE_PARSER_BRE_H
#define ICGREP_RE_PARSER_BRE_H

#include <re/re_parser.h>
#include <re/re_parser_ere.h>

namespace re {
    class RE_Parser_BRE : public RE_Parser_ERE  {
    public:
        RE_Parser_BRE(const std::string & regular_expression) : RE_Parser_ERE(regular_expression) {
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


#endif //ICGREP_RE_PARSER_BRE_H
