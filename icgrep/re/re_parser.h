/*
 *  Copyright (c) 2014 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#ifndef RE_PARSER_H
#define RE_PARSER_H

#include "re_re.h"
#include "re_cc.h"
#include "re_name.h"

#include <string>
#include <list>
#include <memory>

namespace re {

class RE_Parser
{
public:

    static RE * parse(const std::string &intput_string, const bool allow_escapes_within_charset = false);

private:

    typedef std::string::const_iterator cursor_t;

    RE_Parser(const std::string & regular_expression, const bool allow_escapes_within_charset);

    RE * parse_alt(const bool subexpression);

    RE * parse_seq();

    RE * parse_next_token();

    CC * parse_any_character();

    RE * extend_item(RE * re);

    RE * parse_range_bound(RE * re);

    RE * parse_literal();

    RE * parse_escaped_metacharacter();

    unsigned parse_utf8_codepoint();

    Name * parse_unicode_category(const bool negated);

    RE * parse_charset();

    bool parse_charset_literal(unsigned & literal);

    unsigned parse_hex();

    unsigned parse_int();

    static void negate_cc(std::unique_ptr<CC> & cc);

    inline void throw_incomplete_expression_error_if_end_of_stream() const;

private:

    cursor_t                    _cursor;
    const cursor_t              _end;
    const bool                  _allow_escapes_within_charset;
};

}

#endif // RE_PARSER_H
