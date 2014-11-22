/*
 *  Copyright (c) 2014 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#ifndef RE_PARSER_H
#define RE_PARSER_H

#include "re_re.h"
#include "re_any.h"
#include "re_name.h"

#include <string>
#include <list>
#include <memory>

namespace re {
	
enum CharsetOperatorKind
	{intersectOp, setDiffOp, ampChar, hyphenChar, rangeHyphen, posixPropertyOpener, setOpener, setCloser, backSlash, emptyOperator};


class RE_Parser
{
public:

    static RE * parse(const std::string &input_string);

private:

    typedef std::string::const_iterator cursor_t;

    RE_Parser(const std::string & regular_expression);

    RE * parse_alt(const bool subexpression);

    RE * parse_seq();

    RE * parse_next_token();

    Any * parse_any_character();

    RE * extend_item(RE * re);

    void parse_range_bound(int & lower_bound, int & upper_bound);

    RE * parse_literal();

    RE * parse_escaped();

    RE * parse_escaped_set();

    unsigned parse_utf8_codepoint();

    Name * parse_property_expression();
	
	CharsetOperatorKind getCharsetOperator();

    RE * parse_charset();

    unsigned parse_codepoint();

    unsigned parse_escaped_codepoint();

    unsigned parse_hex_codepoint(int mindigits, int maxdigits);

    unsigned parse_octal_codepoint(int mindigits, int maxdigits);

    unsigned parse_int();

    inline void throw_incomplete_expression_error_if_end_of_stream() const;

private:

    cursor_t                    _cursor;
    const cursor_t              _end;
};

}

#endif // RE_PARSER_H
