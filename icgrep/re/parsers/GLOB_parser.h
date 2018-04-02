/*
 *  Copyright (c) 2018 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#ifndef GLOB_PARSER_H
#define GLOB_PARSER_H

#include <re/parsers/parser.h>
#include <re/parsers/ERE_parser.h>

//  GLOB parsing for Filename expansion in accord with Posix rules
//  IEEE-1003.1 XCU section  2.13 Pattern Matching Notation

namespace re {
    class FileGLOB_Parser : public RE_Parser  {
    public:
        FileGLOB_Parser(const std::string & glob) : RE_Parser(glob), mExcludeDotContext(true) {
            mReSyntax = RE_Syntax::FileGLOB;
        }

    protected:
        RE * parse_alt() override;
        RE * parse_seq() override;
        RE * parse_next_item() override;
        RE * parse_bracket_expr();
        RE * range_extend(RE * e1);
    private:
        bool mExcludeDotContext;
    };
}


#endif
