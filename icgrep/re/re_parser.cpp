/*
 *  Copyright (c) 2017 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#include <re/re_parser.h>
#include <re/re_parser_helper.h>
#include <re/re_parser_pcre.h>
#include <re/re_parser_ere.h>
#include <re/re_parser_bre.h>
#include <re/re_parser_prosite.h>
#include <re/parse_fixed_strings.h>
#include <re/re_name.h>
#include <re/re_alt.h>
#include <re/re_any.h>
#include <re/re_end.h>
#include <re/re_rep.h>
#include <re/re_seq.h>
#include <re/re_start.h>
#include <re/re_range.h>
#include <re/re_diff.h>
#include <re/re_intersect.h>
#include <re/re_group.h>
#include <re/re_assertion.h>
#include <re/printer_re.h>
#include <sstream>
#include <string>
#include <algorithm>
#include <iostream>
#include <llvm/Support/Casting.h>
#include <llvm/Support/ErrorHandling.h>
#include <llvm/Support/raw_ostream.h>
#include <llvm/ADT/STLExtras.h> // for make_unique

using namespace llvm;

namespace re {


RE * RE_Parser::parse(const std::string & regular_expression, ModeFlagSet initialFlags, RE_Syntax syntax, bool ByteMode) {

    std::unique_ptr<RE_Parser> parser = nullptr;
    switch (syntax) {
        case RE_Syntax::PCRE:
            parser = make_unique<RE_Parser_PCRE>(regular_expression);
            break;
        case RE_Syntax::ERE:
            parser = make_unique<RE_Parser_ERE>(regular_expression);
            break;
        case RE_Syntax ::BRE:
            parser = make_unique<RE_Parser_BRE>(regular_expression);
            break;
        case RE_Syntax ::PROSITE:
            parser = make_unique<RE_Parser_PROSITE>(regular_expression);
            break;
        default:
            parser = make_unique<FixedStringParser>(regular_expression);
            break;
    }
    parser->fByteMode = ByteMode;
    parser->fModeFlagSet = initialFlags;
    parser->mGroupsOpen = 0;
    parser->fNested = false;
    parser->mCaptureGroupCount = 0;
    RE * re = parser->parse_RE();
    if (re == nullptr) {
        ParseFailure("An unexpected parsing error occurred!");
    }
    return re;
}

RE_Parser::RE_Parser(const std::string & regular_expression)
: fByteMode(false)
, fModeFlagSet(MULTILINE_MODE_FLAG)
, fNested(false)
, mGroupsOpen(0)
, mCursor(regular_expression)
, mCaptureGroupCount(0)
, mReSyntax(RE_Syntax::PCRE)
{

}

RE * makeAtomicGroup(RE * r) {
    RE_Parser::ParseFailure("Atomic grouping not supported.");
}

RE * makeBranchResetGroup(RE * r) {
    // Branch reset groups only affect submatch numbering, but
    // this has no effect in icgrep.
    return r;
}

RE * RE_Parser::parse_RE() {
    return parse_alt();
}

RE * RE_Parser::parse_alt() {
    std::vector<RE *> alt;
    do {
        alt.push_back(parse_seq());
    }
    while (accept('|'));
    return makeAlt(alt.begin(), alt.end());
}
    
RE * RE_Parser::parse_seq() {
    std::vector<RE *> seq;
    if (!mCursor.more() || (*mCursor == '|') || ((mGroupsOpen > 0) && (*mCursor == ')'))) return makeSeq();
    for (;;) {
        RE * re = parse_next_item();
        if (re == nullptr) {
            break;
        }
        re = extend_item(re);
        seq.push_back(re);
    }
    return makeSeq(seq.begin(), seq.end());
}

RE * createStart(ModeFlagSet flags) {
    if ((flags && ModeFlagType::MULTILINE_MODE_FLAG) == 0) return makeZeroWidth("^s");  //single-line mode
    if ((flags & ModeFlagType::UNIX_LINES_MODE_FLAG) != 0) {
        return makeNegativeLookBehindAssertion(makeByte(makeCC(makeCC(0, '\n'-1), makeCC('\n'+1, 0xFF))));
    }
    return makeStart();
}
RE * createEnd(ModeFlagSet flags) {
    if ((flags && ModeFlagType::MULTILINE_MODE_FLAG) == 0) return makeZeroWidth("$s");  //single-line mode
    if ((flags & ModeFlagType::UNIX_LINES_MODE_FLAG) != 0) {
        return makeNegativeLookAheadAssertion(makeByte(makeCC(makeCC(0, '\n'-1), makeCC('\n'+1, 0xFF))));
    }
    return makeEnd();
}
RE * createAny(ModeFlagSet flags) {
    return makeAny();
}
    
    
RE * RE_Parser::parse_next_item() {
    if (mCursor.noMore() || atany("*?+{|")) return nullptr;
    else if (((mGroupsOpen > 0) && at(')')) || (fNested && at('}'))) return nullptr;
    else if (accept('^')) return createStart(fModeFlagSet);
    else if (accept('$')) return createEnd(fModeFlagSet);
    else if (accept('.')) return createAny(fModeFlagSet);
    else if (accept('(')) return parse_group();
    else if (accept('[')) return parse_extended_bracket_expression();
    else if (accept('\\')) return parse_escaped();
    else return createCC(parse_literal_codepoint());
}
    
    
RE * RE_Parser::parse_mode_group(bool & closing_paren_parsed) {
    const ModeFlagSet savedModeFlagSet = fModeFlagSet;
    while (mCursor.more() && !atany(":)")) {
        bool negateMode = accept('-');
        ModeFlagType modeBit;
        switch (*mCursor) {
            case 'i': modeBit = CASE_INSENSITIVE_MODE_FLAG; break;
            case 'g': modeBit = GRAPHEME_CLUSTER_MODE; break;
            case 'm': modeBit = MULTILINE_MODE_FLAG; break;
                //case 's': modeBit = DOTALL_MODE_FLAG; break;
            case 'x': modeBit = IGNORE_SPACE_MODE_FLAG; break;
            case 'd': modeBit = UNIX_LINES_MODE_FLAG; break;
            default: ParseFailure("Unsupported mode flag.");
        }
        ++mCursor;
        if (negateMode) {
            fModeFlagSet &= ~modeBit;
            negateMode = false;  // for next flag
        } else {
            fModeFlagSet |= modeBit;
        }
    }
    if (accept(':')) {
        RE * group_expr = parse_alt();
        auto changed = fModeFlagSet ^ savedModeFlagSet;
        if ((changed & CASE_INSENSITIVE_MODE_FLAG) != 0) {
            group_expr = makeGroup(Group::Mode::CaseInsensitiveMode, group_expr,
                                   (fModeFlagSet & CASE_INSENSITIVE_MODE_FLAG) == 0 ? Group::Sense::Off : Group::Sense::On);
        }
        if ((changed & GRAPHEME_CLUSTER_MODE) != 0) {
            group_expr = makeGroup(Group::Mode::GraphemeMode, group_expr,
                                   (fModeFlagSet & GRAPHEME_CLUSTER_MODE) == 0 ? Group::Sense::Off : Group::Sense::On);
        }
        fModeFlagSet = savedModeFlagSet;
        closing_paren_parsed = false;
        return group_expr;
    } else {  // if *_cursor == ')'
        require(')');
        closing_paren_parsed = true;
        auto changed = fModeFlagSet ^ savedModeFlagSet;
        if ((changed & (CASE_INSENSITIVE_MODE_FLAG|GRAPHEME_CLUSTER_MODE)) != 0) {
            RE * group_expr = parse_seq();
            if ((changed & CASE_INSENSITIVE_MODE_FLAG) != 0) {
                group_expr = makeGroup(Group::Mode::CaseInsensitiveMode, group_expr,
                                       (fModeFlagSet & CASE_INSENSITIVE_MODE_FLAG) == 0 ? Group::Sense::Off : Group::Sense::On);
            }
            if ((changed & GRAPHEME_CLUSTER_MODE) != 0) {
                group_expr = makeGroup(Group::Mode::GraphemeMode, group_expr,
                                       (fModeFlagSet & GRAPHEME_CLUSTER_MODE) == 0 ? Group::Sense::Off : Group::Sense::On);
            }
            return group_expr;
        }
        else return makeSeq();
    }

}

// Parse some kind of parenthesized group.  Input precondition: mCursor
// after the (
RE * RE_Parser::parse_group() {
    mGroupsOpen++;
    RE * group_expr = nullptr;
    if (accept('?')) {
        if (accept('#')) {
            while (mCursor.more() && !at(')')) ++mCursor;
            group_expr = makeSeq();
        } else if (accept(':')) { // Non-capturing paren
            group_expr = parse_alt();
        } else if (accept('=')) { // positive look ahead
            group_expr = makeLookAheadAssertion(parse_alt());
        } else if (accept('!')) { // negative look ahead
            group_expr = makeNegativeLookAheadAssertion(parse_alt());
        } else if (accept("<=")) { // positive look ahead
            group_expr = makeLookBehindAssertion(parse_alt());
        } else if (accept("<!")) { // negative look ahead
            group_expr = makeNegativeLookBehindAssertion(parse_alt());
        } else if (accept('>')) { // negative look ahead
            group_expr = makeAtomicGroup(parse_alt());
        } else if (accept('|')) { // negative look ahead
            group_expr = makeBranchResetGroup(parse_alt());
        } else if (atany("-dimsxg")) { // mode switches
            bool closing_paren_parsed;
            group_expr = parse_mode_group(closing_paren_parsed);
            if (closing_paren_parsed) {
                mGroupsOpen--;
                return group_expr;
            }
        } else {
            ParseFailure("Illegal (? syntax.");
        }
    } else { // Capturing paren group.
        group_expr = parse_capture_body();
    }
    require(')');
    mGroupsOpen--;
    return group_expr;
}
    
RE * RE_Parser::parse_capture_body() {
    RE * captured = parse_alt();
    mCaptureGroupCount++;
    std::string captureName = "\\" + std::to_string(mCaptureGroupCount);
    Name * const capture  = mMemoizer.memoize(makeCapture(captureName, captured));
    auto key = std::make_pair("", captureName);
    mNameMap.insert(std::make_pair(std::move(key), capture));
    return capture;
}
    
    RE * RE_Parser::parse_back_reference() {
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

#define ENABLE_EXTENDED_QUANTIFIERS true
    
// Extend a RE item with one or more quantifiers
RE * RE_Parser::extend_item(RE * re) {
    int lb, ub;
    if (accept('*')) {lb = 0; ub = Rep::UNBOUNDED_REP;}
    else if (accept('?')) {lb = 0; ub = 1;}
    else if (accept('+')) {lb = 1; ub = Rep::UNBOUNDED_REP;}
    else if (accept('{')) std::tie(lb, ub) = parse_range_bound();
    else {
        // No quantifier found.
        return re;
    }
    if (ENABLE_EXTENDED_QUANTIFIERS && accept('?')) {
        // Non-greedy qualifier: no difference for Parabix RE matching
        re = makeRep(re, lb, ub);
    } else if (ENABLE_EXTENDED_QUANTIFIERS && accept('+')) {
        // Possessive qualifier
        if (ub == Rep::UNBOUNDED_REP) {
            re = makeSeq({makeRep(re, lb, ub), makeNegativeLookAheadAssertion(re)});
        } else if (lb == ub) {
            re = makeRep(re, ub, ub);
        } else /* if (lb < ub) */{
            re = makeAlt({makeSeq({makeRep(re, lb, ub-1), makeNegativeLookAheadAssertion(re)}), makeRep(re, ub, ub)});
        }
    } else {
        re = makeRep(re, lb, ub);
    }
    // The quantified expression may be extended with a further quantifier, e,g., [a-z]{6,7}{2,3}
    return extend_item(re);
}

std::pair<int, int> RE_Parser::parse_range_bound() {
    int lb, ub;
    if (accept(',')) {
        lb = 0;
        ub = parse_int();
    } else {
        lb = parse_int();
        if (accept('}')) return std::make_pair(lb, lb);
        else require(',');
        if (accept('}')) return std::make_pair(lb, Rep::UNBOUNDED_REP);
        ub = parse_int();
        if (ub < lb) ParseFailure("Upper bound less than lower bound");
    }
    require('}');
    return std::make_pair(lb, ub);
}

unsigned RE_Parser::parse_int() {
    unsigned value = 0;
    if (!isdigit(*mCursor)) ParseFailure("Expecting integer");
    while (isdigit(*mCursor)) {
        value *= 10;
        value += static_cast<int>(*mCursor++) - 48;
    }
    return value;
}


const uint64_t setEscapeCharacters = bit3C('b') | bit3C('p') | bit3C('q') | bit3C('d') | bit3C('w') | bit3C('s') | bit3C('<') | bit3C('>') |
                                     bit3C('B') | bit3C('P') | bit3C('Q') | bit3C('D') | bit3C('W') | bit3C('S') | bit3C('N') | bit3C('X');

bool RE_Parser::isSetEscapeChar(char c) {
    return c >= 0x3C && c <= 0x7B && ((setEscapeCharacters >> (c - 0x3C)) & 1) == 1;
}
                                 

RE * RE_Parser::parse_escaped() {
    
    if (isSetEscapeChar(*mCursor)) {
        return parseEscapedSet();
    }
    else if (atany("xo0")) {
        codepoint_t cp = parse_escaped_codepoint();
        if ((cp >= 0x80) && (cp <= 0xFF)) {
            return makeByte(makeCC(cp));
        }
        else return createCC(cp);
    }
    else if (isdigit(*mCursor)) {
        return parse_back_reference();
    }
    else {
        return createCC(parse_escaped_codepoint());
    }
}
    
RE * RE_Parser::parseEscapedSet() {
    bool complemented = false;
    RE * re = nullptr;
    switch (*mCursor) {
        case 'B': complemented = true;
        case 'b':
            if (*++mCursor != '{') {
                return complemented ? makeWordNonBoundary() : makeWordBoundary();
            } else {
                ++mCursor;
                if (isCharAhead('}')) {
                    switch (*mCursor) {
                        case 'g':
                            re = complemented ? makeZeroWidth("\\B{g}") : makeZeroWidth("\\b{g}");
                            ++mCursor;
                            ++mCursor;
                            break;
                        case 'w': ParseFailure("\\b{w} not yet supported.");
                        case 'l': ParseFailure("\\b{l} not yet supported.");
                        case 's': ParseFailure("\\b{s} not yet supported.");
//                        default: ParseFailure("Unrecognized boundary assertion");
                    }
                }
                if (!re) {
                    auto propExpr = parsePropertyExpression();
                    if (*mCursor++ != '}') {
                        ParseFailure("Malformed boundary assertion");
                    }
                    re = complemented ? makeReNonBoundary(propExpr) : makeReBoundary(propExpr);
                }
                return re;
            }
        case 'd':
            ++mCursor;
            return makeDigitSet();
        case 'D':
            ++mCursor;
            return makeComplement(makeDigitSet());
        case 's':
            ++mCursor;
            return makeWhitespaceSet();
        case 'S':
            ++mCursor;
            return makeComplement(makeWhitespaceSet());
        case 'w':
            ++mCursor;
            return makeWordSet();
        case 'W':
            ++mCursor;
            return makeComplement(makeWordSet());
        case 'Q':
            complemented = true;
        case 'q':
            if (*++mCursor != '{') {
                ParseFailure("Malformed grapheme cluster expression");
            }
            ++mCursor;
            ParseFailure("Literal grapheme cluster expressions not yet supported.");
            if (*mCursor != '}') {
                ParseFailure("Malformed grapheme cluster expression");
            }
            ++mCursor;
            return complemented ? makeComplement(re) : re;
        case 'P':
            complemented = true;
        case 'p':
            if (*++mCursor != '{') {
                ParseFailure("Malformed property expression");
            }
            ++mCursor;
            re = parsePropertyExpression();
            if (*mCursor != '}') {
                ParseFailure("Malformed property expression");
            }
            ++mCursor;
            return complemented ? makeComplement(re) : re;
        case 'X':
            // \X is equivalent to ".+?\b{g}"; proceed the minimal number of characters (but at least one)
            // to get to the next extended grapheme cluster boundary.
            ++mCursor;
            return makeSeq({makeAny(), makeRep(makeSeq({makeZeroWidth("\\B{g}"), makeAny()}), 0, Rep::UNBOUNDED_REP), makeZeroWidth("\\b{g}")});
        case 'N':
            ++mCursor;
            re = parseNamePatternExpression();
            assert (re);
            return re;
        case '<':
            ++mCursor;
            return makeWordBegin();
        case '>':
            ++mCursor;
            return makeWordEnd();
        default:
            ParseFailure("Internal error");
    }
}
    
void InvalidUTF8Encoding() {
    RE_Parser::ParseFailure("Invalid UTF-8 encoding!");
}

codepoint_t RE_Parser::parse_literal_codepoint() {
    if (fByteMode) {
       return static_cast<uint8_t>(*mCursor++);
    }
    else return parse_utf8_codepoint();
}

codepoint_t RE_Parser::parse_utf8_codepoint() {
    // Must cast to unsigned char to avoid sign extension.
    unsigned char pfx = static_cast<unsigned char>(*mCursor++);
    codepoint_t cp = pfx;
    if (pfx < 0x80) return cp;
    unsigned suffix_bytes = 0;
    if (pfx < 0xE0) {
        if (pfx < 0xC2) {  // bare suffix or illegal prefix 0xC0 or 0xC2
            InvalidUTF8Encoding();
        }
        suffix_bytes = 1;
        cp &= 0x1F;
    } else if (pfx < 0xF0) { // [0xE0, 0xEF]
        cp &= 0x0F;
        suffix_bytes = 2;
    } else { // [0xF0, 0xFF]
        cp &= 0x0F;
        suffix_bytes = 3;
    }
    while (suffix_bytes--) {
        if (mCursor.noMore()) {
            InvalidUTF8Encoding();
        }
        char_t sfx = *mCursor++;
        if ((sfx & 0xC0) != 0x80) {
            InvalidUTF8Encoding();
        }
        cp = (cp << 6) | (sfx & 0x3F);
    }
    // It is an error if a 3-byte sequence is used to encode a codepoint < 0x800
    // or a 4-byte sequence is used to encode a codepoint < 0x10000.
    if ((pfx == 0xE0 && cp < 0x800) || (pfx == 0xF0 && cp < 0x10000)) {
        InvalidUTF8Encoding();
    }
    // It is an error if a 4-byte sequence is used to encode a codepoint
    // above the Unicode maximum.
    if (cp > UCD::UNICODE_MAX) {
        InvalidUTF8Encoding();
    }
    return cp;
}

std::string RE_Parser::canonicalize(const cursor_t begin, const cursor_t end) {
    std::locale loc;
    std::stringstream s;   
    for (auto i = begin; i != end; ++i) {
        switch (*i) {
            case '_': case ' ': case '-':
                break;
            default:
                s << std::tolower(*i, loc);
        }
    }
    return s.str();
}

bool RE_Parser::isCharAhead(char c) {
    if (mCursor.remaining() < 2) {
        return false;
    }
    auto nextCursor = mCursor.pos() + 1;
    return *nextCursor == c;
}

RE * RE_Parser::parsePropertyExpression() {
    const auto start = mCursor.pos();
    while (mCursor.more()) {
        bool done = false;
        switch (*mCursor) {
            case '}': case ':': case '=':
                done = true;
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
        if (*val_start == '/') {
            // property-value is another regex
            auto previous = val_start;
            auto current = (++mCursor).pos();
            val_start = current;
            
            while (true) {
                if (*current == '/' && *previous != '\\') {
                    break;
                }
                
                if (!mCursor.more()) {
                    ParseFailure("Malformed property expression");
                }
                
                previous = current;
                current = (++mCursor).pos();
            }
            ++mCursor;
            //return parseRegexPropertyValue(canonicalize(start, prop_end), std::string(val_start, current));
            return createName(canonicalize(start, prop_end), std::string(val_start-1, current));
        }
        if (*val_start == '@') {
            // property-value is @property@ or @identity@
            auto previous = val_start;
            auto current = (++mCursor).pos();
            val_start = current;
            
            while (true) {
                if (*current == '@' && *previous != '\\') {
                    break;
                }
                
                if (!mCursor.more()) {
                    ParseFailure("Malformed property expression");
                }
                
                previous = current;
                current = (++mCursor).pos();
            }
            ++mCursor;
            //return parseRegexPropertyValue(canonicalize(start, prop_end), std::string(val_start, current));
            return createName(canonicalize(start, prop_end), std::string(val_start-1, current));
        }
        else {
            // property-value is normal string
            while (mCursor.more()) {
                bool done = false;
                switch (*mCursor) {
                    case '}': case ':':
                        done = true;
                }
                if (done) {
                    break;
                }
                ++mCursor;
            }
            return createName(canonicalize(start, prop_end), std::string(val_start, mCursor.pos()));
        }
    }
    return createName(canonicalize(start, mCursor.pos()));
}

Name * RE_Parser::parseNamePatternExpression(){
    require('{');
    const auto start = mCursor.pos();
    while (mCursor.more()) {
        if (*mCursor == '\\') {
            ++mCursor;
            if (!mCursor.more()) {
                break;
            }
        }
        else if (*mCursor == '}') {
            break;
        }
        ++mCursor;
    }
    std::string nameRegexp = "/(?i)" + std::string(start, mCursor.pos());
    require('}');
    return createName("na", nameRegexp);
}


// Parse a bracketted expression with possible && (intersection) or
// -- (set difference) operators.
// Initially, the opening bracket has been consumed.
RE * RE_Parser::parse_extended_bracket_expression () {
    bool negated = accept('^');
    RE * t1 = parse_bracketed_items();
    bool have_new_expr = true;
    while (have_new_expr) {
        if (accept("&&")) {
            RE * t2 = parse_bracketed_items();
            t1 = makeIntersect(t1, t2);
        } else if (accept("--")) {
            RE * t2 = parse_bracketed_items();
            t1 = makeDiff(t1, t2);
        }
        else have_new_expr = false;
    }
    require(']');
    if (negated) return makeComplement(t1);
    else return t1;
}

// Parsing items within a bracket expression.
// Items represent individual characters or sets of characters.
// Ranges may be formed by individual character items separated by '-'.
RE * RE_Parser::parse_bracketed_items () {
    std::vector<RE *> items;
    do {
        if (accept('[')) {
            if (accept('=')) items.push_back(parse_equivalence_class());
            else if (accept('.')) items.push_back(range_extend(parse_collation_element()));
            else if (accept(':')) items.push_back(parse_Posix_class());
            else items.push_back(parse_extended_bracket_expression());
        } else if (accept('\\')) {
            if (at('N') || !isSetEscapeChar(*mCursor)) items.push_back(range_extend(parse_escaped_char_item()));
            else items.push_back(parseEscapedSet());
        } else {
            items.push_back(range_extend(makeCC(parse_literal_codepoint())));
        }
    } while (mCursor.more() && !at(']') && !at("&&") && (!at("--") || at("--]")));
    return makeAlt(items.begin(), items.end());
}

//  Given an individual character expression, check for and parse
//  a range extension if one exists, or return the individual expression.
RE * RE_Parser::range_extend(RE * char_expr1) {
    // A range extension is signalled by a hyphen '-', except for special cases:
    // (a) if the following character is "]" the hyphen is a literal set character.
    // (b) if the following character is "-" the hyphen is part of a set subtract
    // operator, unless it the set is immediately closed by "--]".
    if (!at('-') || at("-]") || (at("--") && !at("--]"))) return char_expr1;
    accept('-');
    RE * char_expr2 = nullptr;
    if (accept('\\')) char_expr2 = parse_escaped_char_item();
    else if (accept('[')) {
        if (accept('.')) char_expr2 = parse_collation_element();
        else ParseFailure("Error in range expression");
    } else {
        char_expr2 = makeCC(parse_literal_codepoint());
    }
    return makeRange(char_expr1, char_expr2);
}

RE * RE_Parser::parse_equivalence_class() {
    const auto start = mCursor.pos() - 1;
    while (mCursor.more() && !at('=')) {
        ++mCursor;
    }
    std::string name = std::string(start, mCursor.pos());
    require("=]");
    return createName(name);
}
RE * RE_Parser::parse_collation_element() {
    const auto start = mCursor.pos() - 1;
    while (mCursor.more() && !at('.')) {
        ++mCursor;
    }
    std::string name = std::string(start, mCursor.pos());
    require(".]");
    return createName(name);
}

RE * RE_Parser::parse_Posix_class() {
    bool negated = accept('^');
    RE * posixSet = parsePropertyExpression();
    require(":]");
    if (negated) return makeComplement(posixSet);
    else return posixSet;
}

RE * RE_Parser::parse_escaped_char_item() {
    if (accept('N')) return parseNamePatternExpression();
    else return makeCC(parse_escaped_codepoint());
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

codepoint_t RE_Parser::parse_escaped_codepoint() {
    codepoint_t cp_value;
    if (accept('a')) return 0x07; // BEL
    else if (accept('e')) return 0x1B; // ESC
    else if (accept('f')) return 0x0C; // FF
    else if (accept('n')) return 0x0A; // LF
    else if (accept('r')) return 0x0D; // CR
    else if (accept('t')) return 0x09; // HT
    else if (accept('v')) return 0x0B; // VT
    else if (accept('c')) {// Control-escape based on next char
            // \c@, \cA, ... \c_, or \ca, ..., \cz
            if (((*mCursor >= '@') && (*mCursor <= '_')) || ((*mCursor >= 'a') && (*mCursor <= 'z'))) {
                cp_value = static_cast<codepoint_t>(*mCursor & 0x1F);
                mCursor++;
                return cp_value;
            }
            else if (accept('?')) return 0x7F;  // \c? ==> DEL
            else ParseFailure("Illegal \\c escape sequence");
    } else if (accept('0')) {
        return parse_octal_codepoint(0,3);
    } else if (accept('o')) {
        if (!accept('{')) ParseFailure("Malformed octal escape sequence");
        cp_value = parse_octal_codepoint(1, 7);
        require('}');
        return cp_value;
    } else if (accept('x')) {
        if (!accept('{')) return parse_hex_codepoint(1,2);  // ICU compatibility
        cp_value = parse_hex_codepoint(1, 6);
        require('}');
        return cp_value;
    } else if (accept('u')) {
        if (!accept('{')) return parse_hex_codepoint(4,4);  // ICU compatibility
        cp_value = parse_hex_codepoint(1, 6);
        require('}');
        return cp_value;
    } else if (accept('U')) {
        return parse_hex_codepoint(8,8);  // ICU compatibility
    } else {
        if (((*mCursor >= 'A') && (*mCursor <= 'Z')) || ((*mCursor >= 'a') && (*mCursor <= 'z'))){
            //Escape unknown letter will be parse as normal letter
            return parse_literal_codepoint();
            //ParseFailure("Undefined or unsupported escape sequence");
        }
        else if ((*mCursor < 0x20) || (*mCursor >= 0x7F))
            ParseFailure("Illegal escape sequence");
        else return static_cast<codepoint_t>(*mCursor++);
    }
}

codepoint_t RE_Parser::parse_octal_codepoint(int mindigits, int maxdigits) {
    codepoint_t value = 0;
    int count = 0;
    while (mCursor.more() && count < maxdigits) {
        const char t = *mCursor;
        if (t < '0' || t > '7') {
            break;
        }
        value = value * 8 | (t - '0');
        ++mCursor;
        ++count;
    }
    if (count < mindigits) ParseFailure("Octal sequence has too few digits");
    if (value > UCD::UNICODE_MAX) ParseFailure("Octal value too large");
    return value;
}

codepoint_t RE_Parser::parse_hex_codepoint(int mindigits, int maxdigits) {
    codepoint_t value = 0;
    int count = 0;
    while (mCursor.more() && isxdigit(*mCursor) && count < maxdigits) {
        const char t = *mCursor;
        if (isdigit(t)) {
            value = (value * 16) | (t - '0');
        }
        else {
            value = ((value * 16) | ((t | 32) - 'a')) + 10;
        }
        ++mCursor;
        ++count;
    }
    if (count < mindigits) ParseFailure("Hexadecimal sequence has too few digits");
    if (value > UCD::UNICODE_MAX) ParseFailure("Hexadecimal value too large");
    return value;
}

Name * RE_Parser::createCC(const codepoint_t cp) {
    CC * cc = makeCC(cp);
    return mMemoizer.memoize(cc);
}

void RE_Parser::insert(CC * cc, const codepoint_t cp) {
    cc->insert(cp);
}

void RE_Parser::insert_range(CC * cc, const codepoint_t lo, const codepoint_t hi) {
    cc->insert_range(lo, hi);
}

RE * RE_Parser::makeComplement(RE * s) {
  return makeDiff(makeAny(), s);
}

           

                           
RE * RE_Parser::makeWordBoundary() {
    Name * wordC = makeWordSet();
    return makeReBoundary(wordC);
}

RE * RE_Parser::makeWordNonBoundary() {
    Name * wordC = makeWordSet();
    return makeReNonBoundary(wordC);
}

inline RE * RE_Parser::makeReBoundary(RE * re) {
    return makeBoundaryAssertion(re);
}
inline RE * RE_Parser::makeReNonBoundary(RE * re) {
    return makeNegativeBoundaryAssertion(re);
}

RE * RE_Parser::makeWordBegin() {
    Name * wordC = makeWordSet();
    return makeNegativeLookBehindAssertion(wordC);
}

RE * RE_Parser::makeWordEnd() {
    Name * wordC = makeWordSet();
    return makeNegativeLookAheadAssertion(wordC);
}

Name * RE_Parser::makeDigitSet() {
    return mMemoizer.memoize(createName("nd"));
}

Name * RE_Parser::makeAlphaNumeric() {
    return mMemoizer.memoize(createName("alnum"));
}

Name * RE_Parser::makeWhitespaceSet() {
    return mMemoizer.memoize(createName("whitespace"));
}

Name * RE_Parser::makeWordSet() {
    return mMemoizer.memoize(createName("word"));
}

Name * RE_Parser::createName(std::string value) {
    auto key = std::make_pair("", value);
    auto f = mNameMap.find(key);
    if (f != mNameMap.end()) {
        return f->second;
    }
    Name * const property = mMemoizer.memoize(makeName(value, Name::Type::UnicodeProperty));
    mNameMap.insert(std::make_pair(std::move(key), property));
    return property;
    }

Name * RE_Parser::createName(std::string prop, std::string value) {
    auto key = std::make_pair(prop, value);
    auto f = mNameMap.find(key);
    if (f != mNameMap.end()) {
        return f->second;
    }
    Name * const property = mMemoizer.memoize(makeName(prop, value, Name::Type::UnicodeProperty));
    mNameMap.insert(std::make_pair(std::move(key), property));
    return property;
}

LLVM_ATTRIBUTE_NORETURN void RE_Parser::ParseFailure(std::string errmsg) {
    llvm::report_fatal_error(errmsg);
}

}
