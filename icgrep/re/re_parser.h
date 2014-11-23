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

typedef unsigned codepoint_t;

enum ModeFlagType 
    {CASE_INSENSITIVE_MODE_FLAG = 1,
     MULTILINE_MODE_FLAG = 2,
     DOTALL_MODE_FLAG = 4,
     IGNORE_SPACE_MODE_FLAG = 8,
     UNIX_LINES_MODE_FLAG = 16};
    
class RE_Parser
{
public:

    static RE * parse(const std::string &input_string);

private:

    typedef std::string::const_iterator cursor_t;

    RE_Parser(const std::string & regular_expression);

    RE * parse_RE();
    
    RE * parse_alt();
    
    RE * parse_seq();

    RE * parse_next_item();
    
    RE * parse_group();
    
    RE * extend_item(RE * re);

    void parse_range_bound(int & lower_bound, int & upper_bound);

    unsigned parse_int();
    
    RE * parse_escaped();

    RE * parse_escaped_set();

    codepoint_t parse_utf8_codepoint();

    Name * parse_property_expression();
	
	CharsetOperatorKind getCharsetOperator();

    RE * parse_charset();

    codepoint_t parse_codepoint();

    codepoint_t parse_escaped_codepoint();

    codepoint_t parse_hex_codepoint(int mindigits, int maxdigits);

    codepoint_t parse_octal_codepoint(int mindigits, int maxdigits);

    inline void throw_incomplete_expression_error_if_end_of_stream() const;

private:

    cursor_t                    _cursor;
    const cursor_t              _end;
};

}

#endif // RE_PARSER_H
