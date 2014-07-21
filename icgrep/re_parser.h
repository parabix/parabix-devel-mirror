/*
 *  Copyright (c) 2014 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#ifndef RE_PARSER_H
#define RE_PARSER_H

#include "re_re.h"
#include "parseresult.h"
#include "parsesuccess.h"
#include "parsefailure.h"

#include "cc_compiler.h"

#include <fstream>
#include <iostream>
#include <string>
#include <sstream>
#include <vector>
#include <ctype.h>


struct parse_int_retVal{
    int i;
    std::string remaining;
};

struct parse_result_retVal{
    ParseResult* result;
    std::string remaining;
};

struct parse_re_list_retVal{
    std::list<RE*> re_list;
    std::string remaining;
};

class RE_Parser
{
public:
    RE_Parser();
    //The module exports the parse result.
    ParseResult* parse_re(std::string intput_string);
private:
    parse_result_retVal parse_re_helper(std::string s);
    parse_re_list_retVal parse_re_alt_form_list(std::string s);
    parse_result_retVal parse_re_form(std::string s);
    parse_re_list_retVal parse_re_item_list(std::string s);
    parse_result_retVal parse_re_item(std::string s);
    parse_result_retVal parse_re_unit(std::string s);
    parse_result_retVal extend_item(RE* re, std::string s);
    parse_result_retVal parse_cc(std::string s);
    parse_result_retVal parse_cc_body(std::string s);
    parse_result_retVal parse_cc_body0(std::string s, CC* cc_sofar);
    parse_result_retVal parse_cc_body1(int chr, std::string s, CC* cc_sofar);

    parse_int_retVal parse_hex(std::string s);
    parse_int_retVal parse_hex_body(int i, std::string s);
    int parse_hex_body1(int i, std::string hex_str);

    parse_int_retVal parse_int(std::string s);
    parse_int_retVal parse_int1(int i, std::string s);
    parse_result_retVal negate_cc_result(parse_result_retVal cc_result);
};

#endif // RE_PARSER_H
