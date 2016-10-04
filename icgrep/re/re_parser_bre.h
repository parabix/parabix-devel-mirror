/*
 *  Copyright (c) 2016 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#ifndef ICGREP_RE_PARSER_BRE_H
#define ICGREP_RE_PARSER_BRE_H

#include <re/re_parser.h>

namespace re {
    class RE_Parser_BRE : public RE_Parser  {
    public:
        RE_Parser_BRE(const std::string & regular_expression) : RE_Parser(regular_expression)  {

        }

    protected:
        virtual bool isSetEscapeChar(char c) override;
        virtual bool isUnsupportChartsetOperator(char c) override;
        virtual RE * parse_alt() override;
        virtual RE * parse_next_item() override ;
        virtual RE * parse_escaped() override;
        virtual RE * extend_item(RE * re) override;
        virtual RE * parse_group() override;
        virtual std::pair<int, int> parse_range_bound() override;
        virtual codepoint_t parse_escaped_codepoint() override;
        virtual RE * parsePropertyExpression() override;
        virtual RE * parseEscapedSet() override;

    private:
        bool isEscapedCharAhead(char c);
        bool isCharAhead(char c);
    };
}


#endif //ICGREP_RE_PARSER_BRE_H
