/*
 *  Copyright (c) 2016 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#include <re/re_parser_bre.h>
#include <re/re_parser_helper.h>
#include <re/re_alt.h>
#include <re/re_any.h>
#include <re/re_seq.h>
#include <re/re_start.h>
#include <re/re_end.h>
#include <re/re_assertion.h>
#include <re/re_rep.h>


namespace re{

    // \d and \D Removed
    const uint64_t setEscapeCharacters = bit3C('b') | bit3C('p') | bit3C('q') | bit3C('w') | bit3C('s') | bit3C('<') | bit3C('>') |
                                         bit3C('B') | bit3C('P') | bit3C('Q') | bit3C('W') | bit3C('S') | bit3C('N') | bit3C('X');

    bool RE_Parser_BRE::isSetEscapeChar(char c) {
        return c >= 0x3C && c <= 0x7B && ((setEscapeCharacters >> (c - 0x3C)) & 1) == 1;
    }

    inline bool RE_Parser_BRE::isUnsupportChartsetOperator(char c) {
        switch (c) {
            case '\\':
                return true;
            default:
                return false;
        }
    }

    RE * RE_Parser_BRE::parse_alt_with_intersect(RE *reToBeIntersected) {
        std::vector<RE *> alt;
        for (;;) {
            alt.push_back(parse_seq_with_intersect(reToBeIntersected));

            if (!isEscapedCharAhead('|')) {
                break;
            }
            ++mCursor; // advance past the alternation character '\'
            ++mCursor; // advance past the alternation character '|'
        }
        if (alt.empty()) {
            ParseFailure("No regular expression found!");
        }
        return makeAlt(alt.begin(), alt.end());
    }

    RE * RE_Parser_BRE::parse_next_item() {
        RE * re = nullptr;
        if (mCursor.more()) {
            switch (*mCursor) {
                case '^':
                    ++mCursor;
                    return makeStart();
                case '$':
                    ++mCursor;
                    return makeEnd();
                case '*':
                    ParseFailure("Need something to repeat before *, \\+, \\? or \\{.");
                case ']':
                    if (LEGACY_UNESCAPED_RBRAK_RBRACE_ALLOWED) {
                        return createCC(parse_utf8_codepoint());
                    }
                    ParseFailure("Use  \\] for literal ].");
                case '[':
                    mCursor++;
                    re = parse_charset();
                    if ((fModeFlagSet & ModeFlagType::GRAPHEME_CLUSTER_MODE) != 0) {
                        re = makeSeq({re, makeZeroWidth("GCB")});
                    }
                    return re;
                case '.': // the 'any' metacharacter
                    mCursor++;
                    return makeAny();
                case '\\':  // escape processing
                    return parse_escaped();
                default:
                    re = createCC(parse_utf8_codepoint());
                    if ((fModeFlagSet & ModeFlagType::GRAPHEME_CLUSTER_MODE) != 0) {
                        fGraphemeBoundaryPending = true;
                    }
                    return re;
            }
        }
        return nullptr;
    }

    inline RE * RE_Parser_BRE::parse_escaped() {
        if (mCursor.remaining() < 2) {
            ParseFailure("Redundant \\ in the end");
        }
        auto nextCursor = mCursor.pos() + 1;
        switch (*nextCursor) {
            case '(':
                ++mCursor;
                ++mCursor;
                return parse_group();
            case '|': case ')':
                return nullptr;
            case '}':
                if (fNested) {
                    return nullptr;  //  a recursive invocation for a regexp in \N{...}
                } else if (LEGACY_UNESCAPED_RBRAK_RBRACE_ALLOWED) {
                    ++mCursor;
                    return createCC(parse_utf8_codepoint());
                }
                ParseFailure("Use \\} for literal }.");
            case '+': case '?': case '{':
                ParseFailure("Need something to repeat before *, \\+, \\? or \\{.");
        }

        ++mCursor;
        if (isSetEscapeChar(*mCursor)) {
            return parseEscapedSet();
        }
        else if (isdigit(*mCursor)) {
            mCursor++;
            std::string backref = std::string(mCursor.pos()-2, mCursor.pos());
            auto key = std::make_pair("", backref);
            auto f = mNameMap.find(key);
            if (f != mNameMap.end()) {
                return makeReference(backref, f->second);
            }
            else {
                ParseFailure("Back reference " + backref + " without prior capture group.");
            }
        }
        else {
            return createCC(parse_escaped_codepoint());
        }
    }

    RE * RE_Parser_BRE::extend_item(RE * re) {
        if (LLVM_LIKELY(mCursor.more())) {
            if (*mCursor == '*') {
                return RE_Parser::extend_item(re);
            } else if (*mCursor == '\\') {
                if (mCursor.remaining() < 2) {
                    ParseFailure("Redundant \\ in the end");
                }
                if (isCharAhead('?') || isCharAhead('+')) {
                    ++mCursor;
                    return RE_Parser::extend_item(re);
                } else if (isCharAhead('{')) {
                    ++mCursor;
                    auto ret = RE_Parser::extend_item(re);
                    ++mCursor;  //Skip '}'
                    return ret;
                }
            }
        }
        return re;
    }

    // Parse some kind of parenthesized group.  Input precondition: mCursor
    // after the (
    RE * RE_Parser_BRE::parse_group() {
        RE * group_expr = nullptr;
        // Capturing paren group.
        RE * captured = parse_alt();
        mCaptureGroupCount++;
        std::string captureName = "\\" + std::to_string(mCaptureGroupCount);
        Name * const capture  = mMemoizer.memoize(makeCapture(captureName, captured));
        auto key = std::make_pair("", captureName);
        mNameMap.insert(std::make_pair(std::move(key), capture));
        group_expr = capture;

        if (!isEscapedCharAhead(')')) {
            ParseFailure("Closing parenthesis required.");
        }
        ++mCursor;
        ++mCursor;
        return group_expr;
    }

    inline bool RE_Parser_BRE::isEscapedCharAhead(char c) {
        if (*mCursor != '\\') {
            return false;
        }
        if (mCursor.remaining() < 2) {
            ParseFailure("Redundant \\ in the end");
        }
        return isCharAhead(c);
    }

    inline std::pair<int, int> RE_Parser_BRE::parse_range_bound() {
        int lower_bound = 0, upper_bound = 0;
        if (*++mCursor != ',') {
            lower_bound = parse_int();
        }

        if (isEscapedCharAhead('}')) {
            upper_bound = lower_bound;
        } else if (*mCursor != ',') {
            ParseFailure("Bad lower bound!");
        } else if (++mCursor, isEscapedCharAhead('}')) {
            upper_bound = Rep::UNBOUNDED_REP;
        } else {
            upper_bound = parse_int();
            if (!isEscapedCharAhead('}')) {
                ParseFailure("Bad upper bound!");
            }
        }
        return std::make_pair(lower_bound, upper_bound);
    }

    // A backslash escape was found, and various special cases (back reference,
    // quoting with \Q, \E, sets (\p, \P, \d, \D, \w, \W, \s, \S, \b, \B), grapheme
    // cluster \X have been ruled out.
    // It may be one of several possibilities or an error sequence.
    // 1. Special control codes (\a, \e, \f, \n, \r, \t, \v)
    // 2. General control codes c[@-_a-z?]
    // 3. Restricted octal notation 0 - 0777
    // 4. General octal notation o\{[0-7]+\}
    // 5. General hex notation x\{[0-9A-Fa-f]+\}
    // 6. An error for any unrecognized alphabetic escape
    // 7. An escaped ASCII symbol, standing for itself

    codepoint_t RE_Parser_BRE::parse_escaped_codepoint() {
        codepoint_t cp_value;
        switch (*mCursor) {
            case 'o':
                ++mCursor;
                if (isEscapedCharAhead('{')) {
                    ++mCursor;
                    ++mCursor;
                    cp_value = parse_octal_codepoint(1, 7);
                    if (!isEscapedCharAhead('}')) ParseFailure("Malformed octal escape sequence");
                    ++mCursor;
                    ++mCursor;
                    return cp_value;
                }
                else {
                    ParseFailure("Malformed octal escape sequence");
                }
            case 'x':
                ++mCursor;
                if (isEscapedCharAhead('{')) {
                    ++mCursor;
                    ++mCursor;
                    cp_value = parse_hex_codepoint(1, 6);
                    if (!isEscapedCharAhead('}')) ParseFailure("Malformed hex escape sequence");
                    ++mCursor;
                    ++mCursor;
                    return cp_value;
                }
                else {
                    return parse_hex_codepoint(1,2);  // ICU compatibility
                }
            case 'u':
                ++mCursor;
                if (isEscapedCharAhead('{')) {
                    ++mCursor;
                    ++mCursor;
                    cp_value = parse_hex_codepoint(1, 6);
                    if (!isEscapedCharAhead('}')) ParseFailure("Malformed hex escape sequence");
                    ++mCursor;
                    ++mCursor;
                    return cp_value;
                }
                else {
                    return parse_hex_codepoint(4,4);  // ICU compatibility
                }
            default:
                return RE_Parser::parse_escaped_codepoint();
        }
    }

    RE * RE_Parser_BRE::parsePropertyExpression() {
        const auto start = mCursor.pos();
        while (mCursor.more()) {
            bool done = false;
            if (isEscapedCharAhead('}')) {
                done = true;
            } else {
                switch (*mCursor) {
                    case ':': case '=':
                        done = true;
                }
            }
            if (done) {
                break;
            }
            ++mCursor;
        }
        if (*mCursor == '=') {
            // We have a property-name = value expression
            const auto prop_end = mCursor.pos();
            mCursor++;
            auto val_start = mCursor.pos();
            if (*val_start != '\\' || !isCharAhead('/')) {
                // property-value is normal string
                while (mCursor.more()) {
                    if (isEscapedCharAhead('}') || *mCursor == ':') {
                        break;
                    }
                    ++mCursor;
                }
                return createName(canonicalize(start, prop_end), canonicalize(val_start, mCursor.pos()));
            } else {
                // property-value is another regex
                ++mCursor;
                auto previous = val_start;
                auto current = (++mCursor).pos();
                val_start = current;

                while (true) {
                    if (*current == '/' && *previous == '\\') {
                        break;
                    }

                    if (!mCursor.more()) {
                        ParseFailure("Malformed property expression");
                    }

                    previous = current;
                    current = (++mCursor).pos();
                }
                ++mCursor;
                return parseRegexPropertyValue(canonicalize(start, prop_end), canonicalize(val_start, previous));
            }
        }
        return createName(canonicalize(start, mCursor.pos()));
    }

    RE * RE_Parser_BRE::parseEscapedSet() {
        bool complemented = false;
        RE * re = nullptr;
        switch (*mCursor) {
            case 'b':
                ++mCursor;
                if (!isEscapedCharAhead('{')) {
                    return complemented ? makeWordNonBoundary() : makeWordBoundary();
                } else {
                    ++mCursor;
                    switch (*++mCursor) {
                        case 'g':
                            re = complemented ? makeZeroWidth("NonGCB") : makeZeroWidth("GCB");
                            break;
                        case 'w': ParseFailure("\\b{w} not yet supported.");
                        case 'l': ParseFailure("\\b{l} not yet supported.");
                        case 's': ParseFailure("\\b{s} not yet supported.");
                        default: ParseFailure("Unrecognized boundary assertion");
                    }
                    ++mCursor;
                    if (!isEscapedCharAhead('}')) {
                        ParseFailure("Malformed boundary assertion");
                    }
                    ++mCursor;
                    ++mCursor;
                    return re;
                }
            case 'q':
                ++mCursor;
                if (!isEscapedCharAhead('{')) {
                    ParseFailure("Malformed grapheme-boundary property expression");
                }
                ++mCursor;
                ++mCursor;
                ParseFailure("Literal grapheme cluster expressions not yet supported.");
                if (!isEscapedCharAhead('}')) {
                    ParseFailure("Malformed grapheme-boundary property expression");
                }
                ++mCursor;
                ++mCursor;
                return complemented ? makeComplement(re) : re;
            case 'p':
                ++mCursor;
                if (!isEscapedCharAhead('{')) {
                    ParseFailure("Malformed property expression");
                }
                ++mCursor;
                ++mCursor;
                re = parsePropertyExpression();
                if (!isEscapedCharAhead('}')) {
                    ParseFailure("Malformed property expression");
                }
                ++mCursor;
                ++mCursor;
                return complemented ? makeComplement(re) : re;
            case 'N':
                ++mCursor;
                if (!isEscapedCharAhead('{')) {
                    ParseFailure("Malformed \\N expression");
                }
                ++mCursor;
                ++mCursor;
                re = parseNamePatternExpression();
                if (!isEscapedCharAhead('}')) {
                    ParseFailure("Malformed \\N expression");
                }
                ++mCursor;
                ++mCursor;
                assert (re);
                return re;
            default:
                return RE_Parser::parseEscapedSet();
        }
    }
}
