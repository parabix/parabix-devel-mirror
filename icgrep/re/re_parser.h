/*
 *  Copyright (c) 2014-6 International Characters.
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
#include <llvm/Support/ErrorHandling.h>

namespace re {

enum RE_Syntax {FixedStrings, BRE, ERE, PCRE, PROSITE};
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

    static RE * parse(const std::string &input_string, ModeFlagSet initialFlags, RE_Syntax syntax = RE_Syntax::PCRE);

    
    static LLVM_ATTRIBUTE_NORETURN void ParseFailure(std::string errmsg) {
        llvm::report_fatal_error(errmsg);
    }
    
protected:
    using NameMap = std::map<std::pair<std::string, std::string>, re::Name *>;

    using cursor_t = std::string::const_iterator;

    using char_t = const std::string::value_type;

    struct Cursor {

        inline Cursor & operator++() {
            if (LLVM_UNLIKELY(mCursor == mEnd)) {
                ParseFailure("Incomplete regular expression!");
            }
            ++mCursor;
            return *this;
        }

        inline Cursor operator++(int) {
            if (LLVM_UNLIKELY(mCursor == mEnd)) {
                ParseFailure("Incomplete regular expression!");
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

    virtual RE * parse_RE();

    virtual RE * parse_alt();

    virtual RE * parse_alt_with_intersect(RE* reToBeIntersected);

    virtual RE * parse_seq();

    RE * parse_seq_with_intersect(RE* reToBeIntersected);

    virtual RE * parse_next_item();

    virtual RE * parse_group();

    virtual bool isSetEscapeChar(char c);

    virtual RE * extend_item(RE * re);

    RE * parseGraphemeBoundary(RE * re);

    virtual std::pair<int, int> parse_range_bound();

    unsigned parse_int();

    virtual RE * parse_escaped();

    virtual RE * parseEscapedSet();

    codepoint_t parse_utf8_codepoint();

    virtual RE * parsePropertyExpression();
    RE * parseRegexPropertyValue(const std::string& propName, const std::string& regexValue);

    Name * parseNamePatternExpression();

    RE * makeComplement(RE * s);
    RE * makeWordBoundary();
    RE * makeWordNonBoundary();
    RE * makeReBoundary(RE * wordC);
    RE * makeReNonBoundary(RE * wordC);
    RE * makeWordBegin();
    RE * makeWordEnd();
    Name * makeDigitSet();
    Name * makeAlphaNumeric();
    Name * makeWhitespaceSet();
    Name * makeWordSet();
   
    Name * createName(std::string && value);
    Name * createName(std::string && prop, std::string && value);

    virtual bool isUnsupportChartsetOperator(char c);
    CharsetOperatorKind getCharsetOperator();

    RE * parse_charset();

    codepoint_t parse_codepoint();

    virtual codepoint_t parse_escaped_codepoint();

    codepoint_t parse_hex_codepoint(int mindigits, int maxdigits);

    codepoint_t parse_octal_codepoint(int mindigits, int maxdigits);

    // CC insertion dependent on case-insensitive flag.
    Name * createCC(const codepoint_t cp);
    void insert(CC * cc, const codepoint_t cp);
    void insert_range(CC * cc, const codepoint_t lo, const codepoint_t hi);

    static std::string canonicalize(const cursor_t begin, const cursor_t end);
    bool isCharAhead(char c);

protected:

    ModeFlagSet                 fModeFlagSet;
    bool                        fNested;
    bool                        fGraphemeBoundaryPending;
    bool                        fSupportNonCaptureGroup;
    Cursor                      mCursor;
    unsigned                    mCaptureGroupCount;
    NameMap                     mNameMap;
    Memoizer                    mMemoizer;
    RE_Syntax                   mReSyntax;
};

}

#endif // RE_PARSER_H
