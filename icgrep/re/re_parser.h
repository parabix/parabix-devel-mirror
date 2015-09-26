/*
 *  Copyright (c) 2014 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#ifndef RE_PARSER_H
#define RE_PARSER_H

#include <re/re_re.h>
#include <re/re_any.h>
#include <re/re_name.h>
#include <UCD/resolve_properties.h>
#include <string>
#include <list>
#include <memory>
#include <map>


namespace re {

enum CharsetOperatorKind
	{intersectOp, setDiffOp, ampChar, hyphenChar, rangeHyphen, posixPropertyOpener, setOpener, setCloser, backSlash, emptyOperator};

enum ModeFlagType 
    {CASE_INSENSITIVE_MODE_FLAG = 1,
     MULTILINE_MODE_FLAG = 2,      // not currently implemented
     DOTALL_MODE_FLAG = 4,         // not currently implemented
     IGNORE_SPACE_MODE_FLAG = 8,   // not currently implemented
     UNIX_LINES_MODE_FLAG = 16};   // not currently implemented
    
const int MAX_REPETITION_LOWER_BOUND = 1024;
const int MAX_REPETITION_UPPER_BOUND = 2048;

typedef unsigned ModeFlagSet;
    
class RE_Parser
{
public:

    friend Name * UCD::resolveProperty(const std::string, RE_Parser *);
    friend Name * UCD::resolveProperty(const std::string, const std::string, RE_Parser *);

    static RE * parse(const std::string &input_string, ModeFlagSet initialFlags);

private:

    using NameMap = std::map<std::pair<std::string, std::string>, re::Name *>;

    typedef std::string::const_iterator cursor_t;

    RE_Parser(const std::string & regular_expression);
    
    RE_Parser(const std::string & regular_expression, ModeFlagSet initialFlags);

    RE * parse_RE();
    
    RE * parse_alt();
    
    RE * parse_seq();

    RE * parse_next_item();
    
    RE * parse_group();
    
    RE * extend_item(RE * re);

    void parse_range_bound(int & lo_codepoint, int & hi_codepoint);

    unsigned parse_int();
    
    RE * parse_escaped();

    RE * parseEscapedSet();

    codepoint_t parse_utf8_codepoint();

    Name * parsePropertyExpression();
    
    CC * parseNamePatternExpression();
    
    RE * makeComplement(RE * s);
    RE * makeWordBoundary();
    RE * makeWordNonBoundary();
    Name * makeDigitSet();
    Name * makeAlphaNumeric();
    Name * makeWhitespaceSet();
    Name * makeWordSet();

    Name * createName(const std::string value);
    Name * createName(const std::string prop, const std::string value);

	CharsetOperatorKind getCharsetOperator();

    RE * parse_charset();

    codepoint_t parse_codepoint();

    codepoint_t parse_escaped_codepoint();

    codepoint_t parse_hex_codepoint(int mindigits, int maxdigits);

    codepoint_t parse_octal_codepoint(int mindigits, int maxdigits);

    inline void throw_incomplete_expression_error_if_end_of_stream() const;
    
    // CC insertion dependent on case-insensitive flag.
    CC * build_CC(codepoint_t cp);
    
    void CC_add_codepoint(CC * cc, codepoint_t cp);
    
    void CC_add_range(CC * cc, codepoint_t lo, codepoint_t hi);

    static std::string canonicalize(const cursor_t begin, const cursor_t end);

private:

    cursor_t                    _cursor;
    const cursor_t              _end;
    ModeFlagSet                 fModeFlagSet;
    bool                        fNested;
    NameMap                     mNameMap;
};

}

#endif // RE_PARSER_H
