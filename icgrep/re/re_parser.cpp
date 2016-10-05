/*
 *  Copyright (c) 2016 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#include <re/re_parser.h>
#include <re/re_parser_helper.h>
#include <re/re_parser_pcre.h>
#include <re/re_parser_ere.h>
#include <re/re_parser_bre.h>
#include <re/re_name.h>
#include <re/re_alt.h>
#include <re/re_end.h>
#include <re/re_rep.h>
#include <re/re_seq.h>
#include <re/re_start.h>
#include <re/re_diff.h>
#include <re/re_intersect.h>
#include <re/re_assertion.h>
#include <re/printer_re.h>
#include <UCD/resolve_properties.h>
#include <UCD/CaseFolding_txt.h>
#include <grep_engine.h>
#include <sstream>
#include <string>
#include <algorithm>

namespace re {
    

RE * RE_Parser::parse(const std::string & regular_expression, ModeFlagSet initialFlags, RE_Syntax syntax) {
    std::unique_ptr<RE_Parser> parser = nullptr;

    switch (syntax) {
        case RE_Syntax::PCRE:
            parser = llvm::make_unique<RE_Parser_PCRE>(regular_expression);
            break;
        case RE_Syntax::ERE:
            parser = llvm::make_unique<RE_Parser_ERE>(regular_expression);
            break;
        case RE_Syntax ::BRE:
            parser = llvm::make_unique<RE_Parser_BRE>(regular_expression);
            break;
        default:
            //TODO handle FixString
            ParseFailure("Unsupport RE syntax!");
            break;
    }


    parser->fModeFlagSet = initialFlags;
    parser->fNested = false;
    parser->fGraphemeBoundaryPending = false;
    parser->mCaptureGroupCount = 0;
    RE * re = parser->parse_RE();
    if (re == nullptr) {
        ParseFailure("An unexpected parsing error occurred!");
    }
    return re;
}

inline RE_Parser::RE_Parser(const std::string & regular_expression)
    : fModeFlagSet(0)
    , fNested(false)
    , fGraphemeBoundaryPending(false)
    , fSupportNonCaptureGroup(false)
    , mCursor(regular_expression)
    , mCaptureGroupCount(0)
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
    for (;;) {
        alt.push_back(parse_seq());
        if (*mCursor != '|') {
            break;
        }
        ++mCursor; // advance past the alternation character '|'
    }
    if (alt.empty()) {
        ParseFailure("No regular expression found!");
    }
    return makeAlt(alt.begin(), alt.end());
}

inline RE * RE_Parser::parse_seq() {
    std::vector<RE *> seq;
    for (;;) {
        RE * re = parse_next_item();
        if (re == nullptr) {
            if (fGraphemeBoundaryPending == true) {
                seq.push_back(makeZeroWidth("GCB"));
                fGraphemeBoundaryPending = false;
            }
            break;
        }
        re = extend_item(re);
        seq.push_back(re);
    }
    return makeSeq(seq.begin(), seq.end());
}

RE * RE_Parser::parse_next_item() {
    RE * re = nullptr;
    if (mCursor.more()) {        
        switch (*mCursor) {
            case '(':
                ++mCursor;
                return parse_group();
            case '^':
                ++mCursor;
                return makeStart();
            case '$':
                ++mCursor;
                return makeEnd();
            case '|': case ')':
                break;
            case '*': case '+': case '?': case '{':
                ParseFailure("Need something to repeat before *, +, ? or {.");
            case ']':
                if (LEGACY_UNESCAPED_RBRAK_RBRACE_ALLOWED) {
                    return createCC(parse_utf8_codepoint());
                }
                ParseFailure("Use  \\] for literal ].");
            case '}':
                if (fNested) {
                    break;  //  a recursive invocation for a regexp in \N{...}
                } else if (LEGACY_UNESCAPED_RBRAK_RBRACE_ALLOWED) {
                    return createCC(parse_utf8_codepoint());
                }
                ParseFailure("Use \\} for literal }.");
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
                ++mCursor;
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


// Parse some kind of parenthesized group.  Input precondition: mCursor
// after the (
RE * RE_Parser::parse_group() {
    const ModeFlagSet modeFlagSet = fModeFlagSet;
    RE * group_expr = nullptr;
    if (*mCursor == '?' && fSupportNonCaptureGroup) {
        switch (*++mCursor) {
            case '#':  // comment
                while (*++mCursor != ')');
                ++mCursor;
                return parse_next_item();
            case ':':  // Non-capturing paren
                ++mCursor;
                group_expr = parse_alt();
                break;
            case '=':
                ++mCursor;
                group_expr = makeLookAheadAssertion(parse_alt());
                break;
            case '!':
                ++mCursor;
                group_expr = makeNegativeLookAheadAssertion(parse_alt());
                break;
            case '>':
                ++mCursor;
                group_expr = makeAtomicGroup(parse_alt());
                break;
            case '|':
                ++mCursor;
                group_expr = makeBranchResetGroup(parse_alt());
                break;
            case '<':
                ++mCursor;
                if (*mCursor == '=') {
                    ++mCursor;
                    group_expr = makeLookBehindAssertion(parse_alt());
                }
                else if (*mCursor == '!') {
                    ++mCursor;
                    group_expr = makeNegativeLookBehindAssertion(parse_alt());
                } else {
                    ParseFailure("Illegal lookbehind assertion syntax.");
                }
                break;
            case '-': case 'd' : case 'i': case 'm': case 's': case 'x': case 'g':
                while (*mCursor != ')' && *mCursor != ':') {
                    bool negateMode = false;
                    ModeFlagType modeBit;
                    if (*mCursor == '-') {
                        negateMode = true;
                        ++mCursor;
                    }
                    switch (*mCursor) {
                        case 'i': modeBit = CASE_INSENSITIVE_MODE_FLAG; break;
                        case 'g': modeBit = GRAPHEME_CLUSTER_MODE; break;
                        //case 'm': modeBit = MULTILINE_MODE_FLAG; break;
                        //case 's': modeBit = DOTALL_MODE_FLAG; break;
                        //case 'x': modeBit = IGNORE_SPACE_MODE_FLAG; break;
                        //case 'd': modeBit = UNIX_LINES_MODE_FLAG; break;
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
                if (*mCursor == ':') {
                    ++mCursor;
                    group_expr = parse_alt();
                    fModeFlagSet = modeFlagSet;
                    break;
                } else {  // if *_cursor == ')'
                    ++mCursor;
                    return parse_next_item();
                }
            default:
                ParseFailure("Illegal (? syntax.");
        }
    } else { // Capturing paren group.
        RE * captured = parse_alt();
        mCaptureGroupCount++;
        std::string captureName = "\\" + std::to_string(mCaptureGroupCount);
        Name * const capture  = mMemoizer.memoize(makeCapture(captureName, captured));
        auto key = std::make_pair("", captureName);
        mNameMap.insert(std::make_pair(std::move(key), capture));
        group_expr = capture;
    }
    if (*mCursor != ')') {
        ParseFailure("Closing parenthesis required.");
    }
    ++mCursor;
    return group_expr;
}

inline RE * RE_Parser::extend_item(RE * re) {
    if (LLVM_LIKELY(mCursor.more())) {
        int lb = 0, ub = 0;
        bool hasRep = true;
        switch (*mCursor) {
            case '*':
                lb = 0;
                ub = Rep::UNBOUNDED_REP;
                break;
            case '?':
                lb = 0;
                ub = 1;
                break;
            case '+':
                lb = 1;
                ub = Rep::UNBOUNDED_REP;
                break;
            case '{':
                std::tie(lb, ub) = parse_range_bound();
                break;
            default:
                hasRep = false;
        }
        if (hasRep) {
            if (lb > MAX_REPETITION_LOWER_BOUND || ub > MAX_REPETITION_UPPER_BOUND) {
                ParseFailure("Bounded repetition exceeds icgrep implementation limit");
            }
            if ((ub != Rep::UNBOUNDED_REP) && (lb > ub)) {
                ParseFailure("Lower bound cannot exceed upper bound in bounded repetition");
            }
            ++mCursor;
            if (*mCursor == '?') { // Non-greedy qualifier
                // Greedy vs. non-greedy makes no difference for icgrep.
                ++mCursor;
            } else if (*mCursor == '+') {
                ++mCursor;
                ParseFailure("Possessive repetition is not supported in icgrep 1.0");
            }
            re = makeRep(re, lb, ub);
        }
    }
    return re;
}

inline std::pair<int, int> RE_Parser::parse_range_bound() {
    int lower_bound = 0, upper_bound = 0;
    if (*++mCursor != ',') {
        lower_bound = parse_int();
    }
    if (*mCursor == '}') {
        upper_bound = lower_bound;
    } else if (*mCursor != ',') {
        ParseFailure("Bad lower bound!");
    } else if (*++mCursor == '}') {
        upper_bound = Rep::UNBOUNDED_REP;
    } else {
        upper_bound = parse_int();
        if (*mCursor != '}') {
            ParseFailure("Bad upper bound!");
        }
    }
    return std::make_pair(lower_bound, upper_bound);
}

unsigned RE_Parser::parse_int() {
    unsigned value = 0;
    while (isdigit(*mCursor)) {
        value *= 10;
        value += static_cast<int>(*mCursor++) - 48;
    }
    return value;
}


const uint64_t setEscapeCharacters = bit3C('b') | bit3C('p') | bit3C('q') | bit3C('d') | bit3C('w') | bit3C('s') | bit3C('<') | bit3C('>') |
                                     bit3C('B') | bit3C('P') | bit3C('Q') | bit3C('D') | bit3C('W') | bit3C('S') | bit3C('N') | bit3C('X');

inline bool RE_Parser::isSetEscapeChar(char c) {
    return c >= 0x3C && c <= 0x7B && ((setEscapeCharacters >> (c - 0x3C)) & 1) == 1;
}
                                 

inline RE * RE_Parser::parse_escaped() {
    
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
    
RE * RE_Parser::parseEscapedSet() {
    bool complemented = false;
    RE * re = nullptr;
    switch (*mCursor) {
        case 'B': complemented = true;
        case 'b':
            if (*++mCursor != '{') {
                return complemented ? makeWordNonBoundary() : makeWordBoundary();
            } else {
                switch (*++mCursor) {
                    case 'g':
                        re = complemented ? makeZeroWidth("NonGCB") : makeZeroWidth("GCB"); 
                        break;
                    case 'w': ParseFailure("\\b{w} not yet supported.");
                    case 'l': ParseFailure("\\b{l} not yet supported.");
                    case 's': ParseFailure("\\b{s} not yet supported.");
                    default: ParseFailure("Unrecognized boundary assertion");
                }
                if (*++mCursor != '}') {
                    ParseFailure("Malformed boundary assertion");
                }
                ++mCursor;
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
                ParseFailure("Malformed grapheme-boundary property expression");
            }
            ++mCursor;
            ParseFailure("Literal grapheme cluster expressions not yet supported.");
            if (*mCursor != '}') {
                ParseFailure("Malformed grapheme-boundary property expression");
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
            return makeSeq({makeAny(), makeRep(makeSeq({makeZeroWidth("NonGCB"), makeAny()}), 0, Rep::UNBOUNDED_REP), makeZeroWidth("GCB")});
        case 'N':
            if (*++mCursor != '{') {
                ParseFailure("Malformed \\N expression");
            }
            ++mCursor;
            re = parseNamePatternExpression();
            if (*mCursor != '}') {
                ParseFailure("Malformed \\N expression");
            }
            ++mCursor;
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
        const auto prop_end = mCursor.pos();
        mCursor++;
        const auto val_start = mCursor.pos();
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
        // We have a property-name = value expression
        return createName(canonicalize(start, prop_end), canonicalize(val_start, mCursor.pos()));
    }
    return createName(canonicalize(start, mCursor.pos()));
}

Name * RE_Parser::parseNamePatternExpression(){

    ModeFlagSet outerFlags = fModeFlagSet;
    fModeFlagSet = 1;

    bool outerNested = fNested;
    fNested = true;

    RE * nameRE = parse_RE();

    // Reset outer parsing state.
    fModeFlagSet = outerFlags;
    fNested = outerNested;

    // Embed the nameRE in ";.*$nameRE" to skip the codepoint field of Uname.txt
    RE * embedded = makeSeq({mMemoizer.memoize(makeCC(0x3B)), makeRep(makeAny(), 0, Rep::UNBOUNDED_REP), nameRE});
    
    GrepEngine engine;
    engine.grepCodeGen("NamePattern", embedded, false, false, true);
    CC * codepoints = engine.grepCodepoints();
    
    if (codepoints) {
        Name * const result = mMemoizer.memoize(codepoints);
        assert (*cast<CC>(result->getDefinition()) == *codepoints);
        return result;
    }
    return nullptr;
}

inline bool RE_Parser::isUnsupportChartsetOperator(char c) {
    return false;
}

CharsetOperatorKind RE_Parser::getCharsetOperator() {
    if (isUnsupportChartsetOperator(*mCursor)) {
        return emptyOperator;
    }
    switch (*mCursor) {
        case '&':
            ++mCursor;
            if (*mCursor == '&') {
                ++mCursor;
                return intersectOp;
            } else if (*mCursor == '[') {
                // Short-hand for intersectOp when a set follows
                return intersectOp;
            }
            return ampChar;
        case '-':
            ++mCursor;
            if (*mCursor == '-') {
                ++mCursor;
                return setDiffOp;
            } else if (*mCursor == '[') {
                return setDiffOp;
            } else if (*mCursor == ']') {
                return hyphenChar;
            }
            return rangeHyphen;
        case '[':
            ++mCursor;
            if (*mCursor == ':') {
                ++mCursor;
                return posixPropertyOpener;
            }
            return setOpener;
        case ']':
            ++mCursor;
            return setCloser;
        case '\\':
            ++mCursor;
            return backSlash;
        default:
            return emptyOperator;
    }
}

// Precondition: cursor is immediately after opening '[' character
RE * RE_Parser::parse_charset() {
    // Sets are accumulated using two variables:
    // subexprs accumulates set expressions such as \p{Lu}, [\w && \p{Greek}]
    // cc accumulates the literal and calculated codepoints and ranges
    std::vector<RE *> subexprs;
    CC * cc = makeCC();
    // When the last item deal with is a single literal charcacter or calculated codepoint,
    // a following hyphen can indicate a range.   When the last item is a set subexpression,
    // a following hyphen can indicate set subtraction.
    enum {NoItem, CodepointItem, RangeItem, SetItem, BrackettedSetItem} lastItemKind = NoItem;

    codepoint_t lastCodepointItem = 0;
    bool havePendingOperation = false;
    CharsetOperatorKind pendingOperationKind = intersectOp;
    RE * pendingOperand = nullptr;

    // If the first character after the [ is a ^ (caret) then the matching character class is complemented.
    bool negated = false;
    if (*mCursor == '^') {
        negated = true;
        ++mCursor;
    }
    // Legacy rule: an unescaped ] may appear as a literal set character
    // if and only if it appears immediately after the opening [ or [^
    if ( *mCursor == ']' && LEGACY_UNESCAPED_RBRAK_RBRACE_ALLOWED) {
        insert(cc, ']');
        lastItemKind = CodepointItem;
        lastCodepointItem = static_cast<codepoint_t> (']');
        ++mCursor;
    } else if ( *mCursor == '-' && LEGACY_UNESCAPED_HYPHEN_ALLOWED) {
        ++mCursor;
        insert(cc, '-');
        lastItemKind = CodepointItem;
        lastCodepointItem = static_cast<codepoint_t> ('-');
        if (*mCursor == '-') {
            ParseFailure("Set operator has no left operand.");
        }
    }
    while (mCursor.more()) {
        const CharsetOperatorKind op = getCharsetOperator();
        switch (op) {
            case intersectOp:
            case setDiffOp: {
                if (lastItemKind == NoItem) {
                    ParseFailure("Set operator has no left operand.");
                }
                if (!cc->empty()) {
                    subexprs.push_back(mMemoizer.memoize(cc));
                }
                RE * newOperand = makeAlt(subexprs.begin(), subexprs.end());
                subexprs.clear();
                cc = makeCC();
                if (havePendingOperation) {
                    if (pendingOperationKind == intersectOp) {
                        pendingOperand = makeIntersect(pendingOperand, newOperand);
                    }
                    else {
                        pendingOperand = makeDiff(pendingOperand, newOperand);
                    }
                }
                else {
                    pendingOperand = newOperand;
                }
                havePendingOperation = true;
                pendingOperationKind = op;
                lastItemKind = NoItem;
            }
            break;
            case setCloser: {
                if (lastItemKind == NoItem) {
                    ParseFailure("Set operator has no right operand.");
                }
                if (!cc->empty()) {
                    subexprs.push_back(mMemoizer.memoize(cc));
                }
                RE * newOperand = makeAlt(subexprs.begin(), subexprs.end());
                if (havePendingOperation) {
                    if (pendingOperationKind == intersectOp) {
                        newOperand = makeIntersect(pendingOperand, newOperand);
                    }
                    else {
                        newOperand = makeDiff(pendingOperand, newOperand);
                    }
                }
                if (fModeFlagSet & CASE_INSENSITIVE_MODE_FLAG) {
                    if (CC * cc1 = dyn_cast<CC>(newOperand)) {
                        newOperand = caseInsensitize(cc1);
                    }
                }
                return negated ? makeComplement(newOperand) : newOperand;
            }
            case setOpener:
            case posixPropertyOpener: {
                if (lastItemKind != NoItem) {
                    if (!cc->empty()) {
                        subexprs.push_back(mMemoizer.memoize(cc));
                    }
                    RE * newOperand = makeAlt(subexprs.begin(), subexprs.end());
                    subexprs.clear();
                    cc = makeCC();
                    if (havePendingOperation) {
                        if (pendingOperationKind == intersectOp) {
                            pendingOperand = makeIntersect(pendingOperand, newOperand);
                        } else {
                            pendingOperand = makeDiff(pendingOperand, newOperand);
                        }
                    }
                    else {
                        pendingOperand = newOperand;
                    }
                    subexprs.push_back(pendingOperand);
                    havePendingOperation = false;
                }
                if (op == setOpener) {
                    subexprs.push_back(parse_charset());
                    lastItemKind = SetItem;
                }
                else if (op == posixPropertyOpener) {
                    bool negated = false;
                    if (*mCursor == '^') {
                        negated = true;
                        mCursor++;
                    }
                    RE * posixSet = parsePropertyExpression();
                    subexprs.push_back(negated ? makeComplement(posixSet) : posixSet);
                    lastItemKind = BrackettedSetItem;
                    if (*mCursor++ != ':' || *mCursor++ != ']')
                        ParseFailure("Posix set expression improperly terminated.");
                }
            }
            break;
            case rangeHyphen:
                if (lastItemKind != CodepointItem) {
                    ParseFailure("Range operator - has illegal left operand.");
                }
                insert_range(cc, lastCodepointItem, parse_codepoint());
                lastItemKind = RangeItem;
                break;
            case hyphenChar:
                insert(cc, '-');
                lastItemKind = CodepointItem;
                lastCodepointItem = static_cast<codepoint_t> ('-');
                break;
            case ampChar:
                insert(cc, '&');
                lastItemKind = CodepointItem;
                lastCodepointItem = static_cast<codepoint_t> ('&');
                break;
            case backSlash:
                if (isSetEscapeChar(*mCursor)) {
                    subexprs.push_back(parseEscapedSet());
                    lastItemKind = SetItem;
                }
                else {
                    lastCodepointItem = parse_escaped_codepoint();
                    insert(cc, lastCodepointItem);
                    lastItemKind = CodepointItem;
                }
                break;
            case emptyOperator:
                lastCodepointItem = parse_utf8_codepoint();
                insert(cc, lastCodepointItem);
                lastItemKind = CodepointItem;
                break;
        }
    }
    ParseFailure("Set expression not properly terminated.");
}


codepoint_t RE_Parser::parse_codepoint() {
    if (*mCursor == '\\') {
        mCursor++;
        return parse_escaped_codepoint();
    } else {
        return parse_utf8_codepoint();
    }
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
    switch (*mCursor) {
        case 'a': ++mCursor; return 0x07; // BEL
        case 'e': ++mCursor; return 0x1B; // ESC
        case 'f': ++mCursor; return 0x0C; // FF
        case 'n': ++mCursor; return 0x0A; // LF
        case 'r': ++mCursor; return 0x0D; // CR
        case 't': ++mCursor; return 0x09; // HT
        case 'v': ++mCursor; return 0x0B; // VT
        case 'c': // Control-escape based on next char
            ++mCursor;
            // \c@, \cA, ... \c_, or \ca, ..., \cz
            if (((*mCursor >= '@') && (*mCursor <= '_')) || ((*mCursor >= 'a') && (*mCursor <= 'z'))) {
                cp_value = static_cast<codepoint_t>(*mCursor & 0x1F);
                mCursor++;
                return cp_value;
            }
            else if (*mCursor++ == '?') return 0x7F;  // \c? ==> DEL
            else ParseFailure("Illegal \\c escape sequence");
        case '0': // Octal escape:  0 - 0377
            ++mCursor;
            return parse_octal_codepoint(0,3);
        case 'o':
            ++mCursor;
            if (*mCursor == '{') {
                ++mCursor;
                cp_value = parse_octal_codepoint(1, 7);
                if (*mCursor++ != '}') ParseFailure("Malformed octal escape sequence");
                return cp_value;
            }
            else {
                ParseFailure("Malformed octal escape sequence");
            }
        case 'x':
            ++mCursor;
            if (*mCursor == '{') {
              ++mCursor;
              cp_value = parse_hex_codepoint(1, 6);
              if (*mCursor++ != '}') ParseFailure("Malformed hex escape sequence");
              return cp_value;
            }
            else {
                return parse_hex_codepoint(1,2);  // ICU compatibility
            }
        case 'u':
            ++mCursor;
            if (*mCursor == '{') {
                ++mCursor;
                cp_value = parse_hex_codepoint(1, 6);
                if (*mCursor++ != '}') ParseFailure("Malformed hex escape sequence");
                return cp_value;
            }
            else {
                return parse_hex_codepoint(4,4);  // ICU compatibility
            }
        case 'U':
            ++mCursor;
            return parse_hex_codepoint(8,8);  // ICU compatibility
        default:
            // Escaped letters should be reserved for special functions.
            if (((*mCursor >= 'A') && (*mCursor <= 'Z')) || ((*mCursor >= 'a') && (*mCursor <= 'z'))){
                //Escape unknow letter will be parse as normal letter
                return parse_utf8_codepoint();
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
    CC * cc = nullptr;
    if (fModeFlagSet & CASE_INSENSITIVE_MODE_FLAG) {
        cc = makeCC();
        caseInsensitiveInsert(cc, cp);
    } else {
        cc = makeCC(cp);
    }
    return mMemoizer.memoize(cc);
}

inline void RE_Parser::insert(CC * cc, const codepoint_t cp) {
    if (fModeFlagSet & CASE_INSENSITIVE_MODE_FLAG) {
        caseInsensitiveInsert(cc, cp);
    } else {
        cc->insert(cp);
    }
}

inline void RE_Parser::insert_range(CC * cc, const codepoint_t lo, const codepoint_t hi) {
    if (fModeFlagSet & CASE_INSENSITIVE_MODE_FLAG) {
        caseInsensitiveInsertRange(cc, lo, hi);
    } else {
        cc->insert_range(lo, hi);
    }
}

RE * RE_Parser::makeComplement(RE * s) {
  return makeDiff(makeAny(), s);
}

           
                       
                           
RE * RE_Parser::makeWordBoundary() {
    Name * wordC = makeWordSet();
    return makeAlt({makeSeq({makeNegativeLookBehindAssertion(wordC), makeLookAheadAssertion(wordC)}),
                    makeSeq({makeLookBehindAssertion(wordC), makeNegativeLookAheadAssertion(wordC)})});
}

RE * RE_Parser::makeWordNonBoundary() {
    Name * wordC = makeWordSet();
    return makeAlt({makeSeq({makeNegativeLookBehindAssertion(wordC), makeNegativeLookAheadAssertion(wordC)}),
                    makeSeq({makeLookBehindAssertion(wordC), makeLookAheadAssertion(wordC)})});
}

RE * RE_Parser::makeWordBegin() {
    Name * wordC = makeWordSet();
    return makeNegativeLookBehindAssertion(wordC);
}

RE * RE_Parser::makeWordEnd() {
    Name * wordC = makeWordSet();
    return makeNegativeLookAheadAssertion(wordC);
}

inline Name * RE_Parser::makeDigitSet() {
    return mMemoizer.memoize(createName("nd"));
}

inline Name * RE_Parser::makeAlphaNumeric() {
    return mMemoizer.memoize(createName("alnum"));
}

inline Name * RE_Parser::makeWhitespaceSet() {
    return mMemoizer.memoize(createName("whitespace"));
}

inline Name * RE_Parser::makeWordSet() {
    return mMemoizer.memoize(createName("word"));
}

Name * RE_Parser::createName(std::string && value) {
    auto key = std::make_pair("", value);
    auto f = mNameMap.find(key);
    if (f != mNameMap.end()) {
        return f->second;
    }
    Name * const property = mMemoizer.memoize(makeName(value, Name::Type::UnicodeProperty));
    mNameMap.insert(std::make_pair(std::move(key), property));
    return property;
    }

Name * RE_Parser::createName(std::string && prop, std::string && value) {
    auto key = std::make_pair(prop, value);
    auto f = mNameMap.find(key);
    if (f != mNameMap.end()) {
        return f->second;
    }
    Name * const property = mMemoizer.memoize(makeName(prop, value, Name::Type::UnicodeProperty));
    mNameMap.insert(std::make_pair(std::move(key), property));
    return property;
}

}
