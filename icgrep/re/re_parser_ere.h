/*
 *  Copyright (c) 2016 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#ifndef ICGREP_RE_PARSER_ERE_H
#define ICGREP_RE_PARSER_ERE_H

#include <re/re_parser.h>
namespace re {
    class RE_Parser_ERE : public RE_Parser  {
    public:
        RE_Parser_ERE(const std::string & regular_expression) : RE_Parser(regular_expression)  {

        }

    protected:
        virtual bool isSetEscapeChar(char c) override;
        virtual bool isUnsupportChartsetOperator(char c) override;
    };
}


#endif //ICGREP_RE_PARSER_ERE_H
