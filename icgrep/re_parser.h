/*
 *  Copyright (c) 2014 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#ifndef RE_PARSER_H
#define RE_PARSER_H

#include "re_re.h"
#include "re_alt.h"
#include "re_cc.h"
#include "re_name.h"
#include "re_end.h"
#include "re_rep.h"
#include "re_seq.h"
#include "re_start.h"
#include "parseresult.h"
#include "parsesuccess.h"
#include "parsefailure.h"

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

struct parse_re_vector_retVal{
    std::vector<RE*> re_vector;
    std::string remaining;
};

class RE_Parser
{
public:
    //RE_Parser();
    //The module exports the parse result.
    static ParseResult* parse_re(std::string intput_string);
private:
    static parse_result_retVal parse_re_helper(std::string s);
    static parse_re_list_retVal parse_re_alt_form_list(std::string s);
    static parse_result_retVal parse_re_form(std::string s);
    static parse_re_list_retVal parse_re_item_list(std::string s);
    static parse_result_retVal parse_re_item(std::string s);
    static parse_result_retVal parse_re_unit(std::string s);
    static parse_result_retVal extend_item(RE* re, std::string s);
    static parse_result_retVal parse_cc(std::string s);
    static parse_result_retVal parse_cc_body(std::string s);
    static parse_result_retVal parse_cc_body0(std::string s, CC* cc_sofar);
    static parse_result_retVal parse_cc_body1(int chr, std::string s, CC* cc_sofar);
    static parse_result_retVal parse_utf8_bytes(int suffix_count, std::string s);
    static parse_result_retVal parse_utf8_suffix_byte(int suffix_byte_num, std::string s, Seq* seq_sofar);

    static parse_result_retVal parse_unicode_category(std::string s, bool negated);
    static parse_result_retVal parse_unicode_category1(std::string character, std::string s, Name* name_sofar);
    static bool isValidUnicodeCategoryName(Name* name);

    static parse_int_retVal parse_hex(std::string s);
    static parse_int_retVal parse_hex_body(int i, std::string s);
    static int parse_hex_body1(int i, std::string hex_str);

    static parse_int_retVal parse_int(std::string s);
    static parse_int_retVal parse_int1(int i, std::string s);
    static parse_result_retVal negate_cc_result(parse_result_retVal cc_result);
};

#endif // RE_PARSER_H
