/*
 *  Copyright (c) 2016 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#ifndef ICGREP_RE_PARSER_PROSITE_H
#define ICGREP_RE_PARSER_PROSITE_H

#include <re/re_parser.h>

namespace re {
    class RE_Parser_PROSITE : public RE_Parser  {
    public:
        RE_Parser_PROSITE(const std::string & regular_expression) : RE_Parser(regular_expression) {
            mReSyntax = RE_Syntax::PROSITE;
        }

    protected:
        virtual RE * parse_RE() override;
        virtual RE * parse_seq() override;
        virtual RE * extend_item(RE * re) override;
        virtual RE * parse_next_item() override;
        virtual std::pair<int, int> parse_range_bound() override;
        
    private:
        RE * parse_prosite_alt();
        RE * parse_prosite_not();
    };
}


#endif //ICGREP_RE_PARSER_PROSITE_H
