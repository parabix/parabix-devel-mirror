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
#include <re/re_memoizer.hpp>
#include <re/parsefailure.h>

namespace re {

enum CharsetOperatorKind
    {intersectOp, setDiffOp, ampChar, hyphenChar, rangeHyphen, posixPropertyOpener, setOpener, setCloser, backSlash, emptyOperator};

enum ModeFlagType : unsigned {
    NONE = 0,
    CASE_INSENSITIVE_MODE_FLAG = 1,
    MULTILINE_MODE_FLAG = 2,      // not currently implemented
    DOTALL_MODE_FLAG = 4,         // not currently implemented
    IGNORE_SPACE_MODE_FLAG = 8,   // not currently implemented
    UNIX_LINES_MODE_FLAG = 16,    // not currently implemented
    GRAPHEME_CLUSTER_MODE = 32
};

const int MAX_REPETITION_LOWER_BOUND = 1024;
const int MAX_REPETITION_UPPER_BOUND = 2048;

typedef unsigned ModeFlagSet;

class RE_Parser
{
public:

    static RE * parse(const std::string &input_string, ModeFlagSet initialFlags);

private:

    using NameMap = std::map<std::pair<std::string, std::string>, re::Name *>;

    using cursor_t = std::string::const_iterator;

    using char_t = const std::string::value_type;

    struct Cursor {

        inline Cursor & operator++() {
            if (LLVM_UNLIKELY(mCursor == mEnd)) {
                throw IncompleteRegularExpression();
            }
            ++mCursor;
            return *this;
        }

        inline Cursor operator++(int) {
            if (LLVM_UNLIKELY(mCursor == mEnd)) {
                throw IncompleteRegularExpression();
            }
            Cursor tmp(*this);
            ++mCursor;
            return tmp;
        }

        inline const char_t operator*() const {
            if (LLVM_UNLIKELY(mCursor == mEnd)) {
                return 0;
            }
            return *mCursor;
        }

        inline bool noMore() const {
            return mCursor == mEnd;
        }

        inline bool more() const {
            return mCursor != mEnd;
        }

        inline cursor_t::difference_type remaining() const {
            return mEnd - mCursor;
        }

        inline cursor_t pos() const {
            return mCursor;
        }

        Cursor(const std::string & expression) : mCursor(expression.cbegin()), mEnd(expression.cend()) {}
        Cursor(const Cursor & cursor) : mCursor(cursor.mCursor), mEnd(cursor.mEnd) {}
        inline Cursor & operator=(const Cursor & cursor) {
            mCursor = cursor.mCursor;
            mEnd = cursor.mEnd;
            return *this;
        }
    private:
        cursor_t    mCursor;
        cursor_t    mEnd;
    };

    RE_Parser(const std::string & regular_expression);

    RE_Parser(const std::string & regular_expression, ModeFlagSet initialFlags);

    RE * parse_RE();

    RE * parse_alt();

    RE * parse_seq();

    RE * parse_next_item();

    RE * parse_group();

    RE * extend_item(RE * re);

    RE * parseGraphemeBoundary(RE * re);

    std::pair<int, int> parse_range_bound();

    unsigned parse_int();

    RE * parse_escaped();

    RE * parseEscapedSet();

    codepoint_t parse_utf8_codepoint();

    RE * parsePropertyExpression();

    Name * parseNamePatternExpression();

    RE * makeComplement(RE * s);
    RE * makeWordBoundary();
    RE * makeWordNonBoundary();
    Name * makeDigitSet();
    Name * makeAlphaNumeric();
    Name * makeWhitespaceSet();
    Name * makeWordSet();

    Name * createName(std::string && value);
    Name * createName(std::string && prop, std::string && value);

    CharsetOperatorKind getCharsetOperator();

    RE * parse_charset();

    codepoint_t parse_codepoint();

    codepoint_t parse_escaped_codepoint();

    codepoint_t parse_hex_codepoint(int mindigits, int maxdigits);

    codepoint_t parse_octal_codepoint(int mindigits, int maxdigits);

    // CC insertion dependent on case-insensitive flag.
    Name * createCC(const codepoint_t cp);
    void insert(CC * cc, const codepoint_t cp);
    void insert_range(CC * cc, const codepoint_t lo, const codepoint_t hi);

    static std::string canonicalize(const cursor_t begin, const cursor_t end);

private:

    Cursor                      mCursor;
    ModeFlagSet                 fModeFlagSet;
    bool                        fNested;
    NameMap                     mNameMap;
    Memoizer                    mMemoizer;
};

}

#endif // RE_PARSER_H
