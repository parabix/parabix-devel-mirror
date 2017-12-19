/*
 *  Copyright (c) 2014-6 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#ifndef RE_PARSER_H
#define RE_PARSER_H

#include <map>
#include <re/re_memoizer.hpp>
#include "re/re_cc.h"

namespace re { class Name; }

namespace re {

enum RE_Syntax {FixedStrings, BRE, ERE, PCRE, PROSITE};

enum ModeFlagType : unsigned {
    DEFAULT_MODE = 0,
    CASE_INSENSITIVE_MODE_FLAG = 1,
    MULTILINE_MODE_FLAG = 2,
    DOTALL_MODE_FLAG = 4,         // not currently implemented
    IGNORE_SPACE_MODE_FLAG = 8,
    UNIX_LINES_MODE_FLAG = 16,
    GRAPHEME_CLUSTER_MODE = 32
};

using ModeFlagSet = unsigned;

class RE_Parser {
public:

    static LLVM_ATTRIBUTE_NORETURN void ParseFailure(std::string errmsg);

    static RE * parse(const std::string &input_string, ModeFlagSet initialFlags, RE_Syntax syntax = RE_Syntax::PCRE, bool ByteMode = false);

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

    
    
    inline bool at(char c) {
        return (mCursor.more()) && (*mCursor == c);
    }
    
    inline bool accept(char c) {
        if (at(c)) {
            mCursor++;
            return true;
        }
        return false;
    }
    
    inline void require(char c) {
        if (!accept(c)) {
            if (mCursor.noMore()) ParseFailure("Expecting " + std::string(1, c) + " but end of input encountered");
            ParseFailure("Expecting " + std::string(1, c) + " but " + std::string(1, *mCursor) + " encountered");
        }
    }
    
    inline bool atany(std::string s) {
        if (mCursor.noMore()) return false;
        for (unsigned i = 0; i < s.length(); i++) {
            if (s[i] == *mCursor) return true;
        }
        return false;
    }
    
    inline bool at(std::string s) {
        Cursor tmp = mCursor;
        for (unsigned i = 0; i < s.length(); i++) {
            if (tmp.noMore() || (s[i] != *tmp)) return false;
            tmp++;
        }
        return true;
    }
    
    inline bool accept(std::string s) {
        Cursor tmp = mCursor;
        for (unsigned i = 0; i < s.length(); i++) {
            if (tmp.noMore() || (s[i] != *tmp)) return false;
            tmp++;
        }
        mCursor = tmp;
        return true;
    }
    
    inline void require(std::string s) {
        if (!accept(s)) {
            if (mCursor.noMore()) ParseFailure("Expecting " + s + " but end of input encountered");
            size_t rem = mCursor.remaining();
            ParseFailure("Expecting " + s + " but " + std::string(mCursor.pos(), mCursor.pos() + std::min(rem, s.length())) + " encountered");
        }
    }
    
    RE_Parser(const std::string & regular_expression);

    RE_Parser(const std::string & regular_expression, ModeFlagSet initialFlags);

    virtual RE * parse_RE();

    virtual RE * parse_alt();
    
    virtual RE * parse_seq();

    virtual RE * parse_next_item();

    virtual RE * parse_group();
    
    RE * parse_mode_group(bool & closing_paren_parsed);

    RE * parse_capture_body();
    
    RE * parse_back_reference();
    
    virtual bool isSetEscapeChar(char c);

    virtual RE * extend_item(RE * re);

    RE * parseGraphemeBoundary(RE * re);

    virtual std::pair<int, int> parse_range_bound();

    unsigned parse_int();

    virtual RE * parse_escaped();

    virtual RE * parseEscapedSet();

    codepoint_t parse_literal_codepoint();
    
    codepoint_t parse_utf8_codepoint();

    virtual RE * parsePropertyExpression();

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
   
    Name * createName(std::string value);
    Name * createName(std::string prop, std::string value);

    RE * parse_extended_bracket_expression();
    RE * parse_bracketed_items();
    RE * range_extend(RE * e1);
    
    RE * parse_equivalence_class();
    RE * parse_collation_element();
    RE * parse_Posix_class();
    RE * parse_escaped_char_item();
    
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
    bool                        fByteMode;
    ModeFlagSet                 fModeFlagSet;
    bool                        fNested;
    unsigned                    mGroupsOpen;
    Cursor                      mCursor;
    unsigned                    mCaptureGroupCount;
    NameMap                     mNameMap;
    Memoizer                    mMemoizer;
    RE_Syntax                   mReSyntax;
};

}

#endif // RE_PARSER_H
