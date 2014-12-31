/*
 *  Copyright (c) 2014 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#include <re/re_parser.h>
#include <re/re_name.h>
#include <re/re_alt.h>
#include <re/re_end.h>
#include <re/re_rep.h>
#include <re/re_seq.h>
#include <re/re_start.h>
#include <re/re_diff.h>
#include <re/re_intersect.h>
#include <re/parsefailure.h>
#include <UCD/CaseFolding_txt.h>
#include <algorithm>


// It would probably be best to enforce that {}, [], () must always
// be balanced.   But legacy syntax allows } and ] to occur as literals
// in certain contexts (no opening { or [, or immediately after [ or [^ ).
// Perhaps this define should become a parameter.
#define LEGACY_UNESCAPED_RBRAK_RBRACE_ALLOWED true
#define LEGACY_UNESCAPED_HYPHEN_ALLOWED true


namespace re {

RE * RE_Parser::parse(const std::string & regular_expression) {
    RE_Parser parser(regular_expression);
    RE * re = parser.parse_RE();
    if (re == nullptr) {
        throw ParseFailure("An unexpected parsing error occurred!");
    }
    return re;
}

inline RE_Parser::RE_Parser(const std::string & regular_expression)
: _cursor(regular_expression.begin())
, _end(regular_expression.end())
, fModeFlagSet(0)
{

}

RE * makeLookAheadAssertion(RE * r) {
    throw ParseFailure("Lookahead assertion not supported.");
}

RE * makeNegativeLookAheadAssertion(RE * r) {
    throw ParseFailure("Lookahead assertion not supported.");
}

RE * makeLookBehindAssertion(RE * r) {
    throw ParseFailure("Lookbehind assertion not supported.");
}

RE * makeNegativeLookBehindAssertion(RE * r) {
    throw ParseFailure("Lookbehind assertion not supported.");
}

RE * makeAtomicGroup(RE * r) {
    throw ParseFailure("Atomic grouping not supported.");
}

RE * makeBranchResetGroup(RE * r) {
    // Branch reset groups only affect submatch numbering, but
    // this has no effect in icgrep.
    return r;
}
    
RE * RE_Parser::parse_RE() {
    RE * r = parse_alt();
    if (_cursor != _end) { 
        throw ParseFailure("Unrecognized junk remaining at end of regexp");
    }
    return r;
}

RE * RE_Parser::parse_alt() {
    std::vector<RE *> alt;
    for (;;) {
        alt.push_back(parse_seq());
        if (_cursor == _end || *_cursor != '|') {
            break;
        }
        ++_cursor; // advance past the alternation character '|'
    }
    if (alt.empty())
    {
        throw NoRegularExpressionFound();
    }
    return makeAlt(alt.begin(), alt.end());
}

inline RE * RE_Parser::parse_seq() {
    std::vector<RE *> seq;
    for (;;) {
        RE * re = parse_next_item();
        if (re == nullptr) {
            break;
        }
        seq.push_back(extend_item(re));
    }
    if (seq.empty())
    {
        throw NoRegularExpressionFound();
    }
    return makeSeq(seq.begin(), seq.end());
}

RE * RE_Parser::parse_next_item() {
    RE * re = nullptr;
    if (_cursor != _end) {
        switch (*_cursor) {
            case '(':
                ++_cursor;
                return parse_group();
            case '^':
                ++_cursor;
                return makeStart();
            case '$':
                ++_cursor;
                return makeEnd();
            case '|': case ')':
                return nullptr;  // This is ugly.
            case '*': case '+': case '?': case '{': 
                throw NothingToRepeat();
            case ']': case '}':
                if (LEGACY_UNESCAPED_RBRAK_RBRACE_ALLOWED) {
                    return build_CC(parse_utf8_codepoint());
                }
                else throw ParseFailure("Use  \\] or \\} for literal ] or }.");
            case '[':
                _cursor++;
                return parse_charset();
            case '.': // the 'any' metacharacter
                _cursor++;
                return makeAny();
            case '\\':  // escape processing
                ++_cursor;
                return parse_escaped();
            default:
                return build_CC(parse_utf8_codepoint());
        }
    }
    return re;
}
    
    
// Parse some kind of parenthesized group.  Input precondition: _cursor
// after the (
RE * RE_Parser::parse_group() {
    RE * subexpr;
    RE * group_expr;
    ModeFlagSet savedModeFlags = fModeFlagSet;
    throw_incomplete_expression_error_if_end_of_stream();
    if (*_cursor == '?') {
        ++_cursor;
        throw_incomplete_expression_error_if_end_of_stream();
        switch (*_cursor) {
            case '#':  // comment
                ++_cursor;
                while (_cursor != _end && *_cursor != ')') {
                    ++_cursor;
                }
                throw_incomplete_expression_error_if_end_of_stream();
                ++_cursor;
                return parse_next_item();
            case ':':  // Non-capturing paren
                ++_cursor;
                group_expr = parse_alt();
                break;
            case '=':
                ++_cursor;
                subexpr = parse_alt();
                group_expr = makeLookAheadAssertion(subexpr);
                break;
            case '!':
                ++_cursor;
                subexpr = parse_alt();
                group_expr = makeNegativeLookAheadAssertion(subexpr);
                break;
            case '>':
                ++_cursor;
                subexpr = parse_alt();
                group_expr = makeAtomicGroup(subexpr);
                break;
            case '|':
                ++_cursor;
                subexpr = parse_alt();
                group_expr = makeBranchResetGroup(subexpr);
                break;
            case '<':
                ++_cursor;
                throw_incomplete_expression_error_if_end_of_stream();
                if (*_cursor == '=') {
                    subexpr = parse_alt();
                    group_expr = makeLookBehindAssertion(subexpr);
                }
                else if (*_cursor == '!') {
                    subexpr = parse_alt();
                    group_expr = makeNegativeLookBehindAssertion(subexpr);
                }
                else {
                    throw ParseFailure("Illegal lookbehind assertion syntax.");
                }
                break;
            case '-': case 'd' : case 'i': case 'm': case 's': case 'x': {
                bool negateMode = false;
                ModeFlagType modeBit;
                while (_cursor != _end && *_cursor != ')' && *_cursor != ':') {
                    if (*_cursor == '-') {
                        negateMode = true;
                        _cursor++;
                        throw_incomplete_expression_error_if_end_of_stream();
                    }
                    switch (*_cursor++) {
                        case 'i': modeBit = CASE_INSENSITIVE_MODE_FLAG; break;
                        case 'm': modeBit = MULTILINE_MODE_FLAG; break;
                        case 's': modeBit = DOTALL_MODE_FLAG; break;
                        case 'x': modeBit = IGNORE_SPACE_MODE_FLAG; break;
                        case 'd': modeBit = UNIX_LINES_MODE_FLAG; break;
                        default: throw ParseFailure("Unrecognized mode flag.");
                    }
                    if (negateMode) {
                        fModeFlagSet &= ~modeBit;
                        negateMode = false;  // for next flag
                    }
                    else fModeFlagSet |= modeBit;
                }
                throw_incomplete_expression_error_if_end_of_stream();
                if (*_cursor == ':') {
                    ++_cursor;
                    group_expr = parse_alt();
                }
                else {  // if *_cursor == ')'
                    ++_cursor;
		    // return immediately without restoring mode flags
                    return parse_next_item();
                }
                break;
            }
            default:
                throw ParseFailure("Illegal (? syntax.");
        }
    }
    else {
        // Capturing paren group, but ignore capture in icgrep.
        group_expr = parse_alt();
    }
    // Restore mode flags.
    fModeFlagSet = savedModeFlags;
    if (_cursor == _end || *_cursor++ != ')')
        throw ParseFailure("Closing paren required.");
    return group_expr;
}
    
RE * RE_Parser::extend_item(RE * re) {
    int lower_bound, upper_bound;
    if (_cursor == _end) {
        return re;
    }
    switch (*_cursor) {
        case '*':
            lower_bound = 0;
            upper_bound = Rep::UNBOUNDED_REP;
            break;
        case '?':
            lower_bound = 0;
            upper_bound = 1;
            break;
        case '+':
            lower_bound = 1;
            upper_bound = Rep::UNBOUNDED_REP;
            break;
        case '{':
            parse_range_bound(lower_bound, upper_bound);
            break;
        default:
            return re;
    }
    ++_cursor;
    if (*_cursor == '?') {
        // Non-greedy qualifier
        ++_cursor;
        //return makeNonGreedyRep(re, lower_bound, upper_bound);
        // Greedy vs. non-greedy makes no difference for icgrep.
        return makeRep(re, lower_bound, upper_bound);
    }
    else if (*_cursor == '+') {
        // Possessive qualifier
        ++_cursor;
        //return makePossessiveRep(re, lower_bound, upper_bound);
        throw ParseFailure("Possessive repetition is not supported in icgrep 1.0");
    }
    else {
        // Normal repetition operator.
        return makeRep(re, lower_bound, upper_bound);
    }
}

inline void RE_Parser::parse_range_bound(int & lower_bound, int & upper_bound) {
    ++_cursor;
    throw_incomplete_expression_error_if_end_of_stream();
    if (*_cursor == ',') {
        ++_cursor;
        lower_bound = 0;
    }
    else {
        lower_bound = parse_int();
    }
    throw_incomplete_expression_error_if_end_of_stream();
    if (*_cursor == '}') {
        upper_bound = lower_bound;
    }
    else if (*_cursor != ',') {
        throw BadLowerBound();
    }
    else { // [^,}]
        ++_cursor;
        throw_incomplete_expression_error_if_end_of_stream();
        if (*_cursor == '}') {
            upper_bound = Rep::UNBOUNDED_REP;
        }
        else {
            upper_bound = parse_int();
            if (*_cursor != '}') {
                throw BadUpperBound();
            }
        }
    }
}

unsigned RE_Parser::parse_int() {
    unsigned value = 0;
    for (; _cursor != _end; ++_cursor) {
        if (!isdigit(*_cursor)) {
            break;
        }
        value *= 10;
        value += static_cast<int>(*_cursor) - 48;
    }
    return value;
}


#define bit40(x) (1ULL << ((x) - 0x40))
const uint64_t setEscapeCharacters = bit40('p') | bit40('d') | bit40('w') | bit40('s') | bit40('P') | bit40('D') | bit40('W') | bit40('S');

inline bool isSetEscapeChar(char c) {
    return c >= 0x40 && c <= 0x7F && ((setEscapeCharacters >> (c - 0x40)) & 1) == 1;
}

inline RE * RE_Parser::parse_escaped() {
    throw_incomplete_expression_error_if_end_of_stream();
    if (isSetEscapeChar(*_cursor)) 
      return parse_escaped_set();
    else 
      return build_CC(parse_escaped_codepoint());
}

RE * makeDigitSet() {
  return makeName("Nd", Name::Type::UnicodeProperty);
}

RE * makeWhitespaceSet() {
  throw ParseFailure("\\s not implemented.");
}

RE * makeWordSet() {
  throw ParseFailure("\\w not implemented.");
}

RE * makeComplement(RE * s) {
  return makeDiff(makeAny(), s);
}

RE * RE_Parser::parse_escaped_set() {
    bool complemented = false;
    RE * s;
    switch (*_cursor) {
        case 'd':
            ++_cursor;
            return makeDigitSet();
        case 'D':
            ++_cursor;
            return makeComplement(makeDigitSet());
        case 's':
            ++_cursor;
            return makeWhitespaceSet();
        case 'S':
            ++_cursor;
            return makeComplement(makeWhitespaceSet());
        case 'w':
            ++_cursor;
            return makeWordSet();
        case 'W':
            ++_cursor;
            return makeComplement(makeWordSet());
        case 'P':
            complemented = true;
        case 'p':
            ++_cursor;
            if (_cursor == _end || *_cursor != '{') throw ParseFailure("Malformed property expression");
            ++_cursor;
            s = parse_property_expression();
            if (_cursor == _end || *_cursor != '}') throw ParseFailure("Malformed property expression");
            ++_cursor;
            if (complemented) return makeComplement(s);
            else return s;
        default:
            throw ParseFailure("Internal error");
    }
}

codepoint_t RE_Parser::parse_utf8_codepoint() {
    // Must cast to unsigned char to avoid sign extension.
    unsigned char pfx = static_cast<unsigned char>(*_cursor++);
    codepoint_t cp = pfx;
    if (pfx < 0x80) return cp;
    unsigned suffix_bytes;
    if (pfx < 0xE0) {
        if (pfx < 0xC2) {  // bare suffix or illegal prefix 0xC0 or 0xC2
            throw InvalidUTF8Encoding();
        }
        suffix_bytes = 1;
        cp &= 0x1F;
    }
    else if (pfx < 0xF0) { // [0xE0, 0xEF]
        cp &= 0x0F;
        suffix_bytes = 2;
    }
    else { // [0xF0, 0xFF]
        cp &= 0x0F;
        suffix_bytes = 3;
    }
    while (suffix_bytes--) {
        if (_cursor == _end) {
            throw InvalidUTF8Encoding();
        }
        unsigned char sfx = static_cast<unsigned char>(*_cursor++);
        if ((sfx & 0xC0) != 0x80) {
            throw InvalidUTF8Encoding();
        }
        cp = (cp << 6) | (sfx & 0x3F);
    }
    // It is an error if a 3-byte sequence is used to encode a codepoint < 0x800
    // or a 4-byte sequence is used to encode a codepoint < 0x10000.
    if ((pfx == 0xE0 && cp < 0x800) || (pfx == 0xF0 && cp < 0x10000)) {
        throw InvalidUTF8Encoding();
    }
    // It is an error if a 4-byte sequence is used to encode a codepoint 
    // above the Unicode maximum.   
    if (cp > CC::UNICODE_MAX) {
        throw InvalidUTF8Encoding();
    }
    return cp;
}

Name * RE_Parser::parse_property_expression() {
    const cursor_t start = _cursor;
    while (_cursor != _end && *_cursor != '}' and *_cursor != ':' and *_cursor != '=') {
        _cursor++;
    }
    if (_cursor != _end && *_cursor == '=') {
        const cursor_t prop_end = _cursor;
        _cursor++;
        const cursor_t val_start = _cursor;
        while (_cursor != _end && *_cursor != '}' and *_cursor != ':') {
            _cursor++;
        }
        // We have a property-name = value expression
        return makeName(std::string(start, prop_end), std::string(val_start, _cursor), Name::Type::UnicodeProperty);
    }
    else return makeName(std::string(start, _cursor), Name::Type::UnicodeProperty);
}

CharsetOperatorKind RE_Parser::getCharsetOperator() {
    throw_incomplete_expression_error_if_end_of_stream();
    switch (*_cursor) {
        case '&':
            ++_cursor;
            if (_cursor != _end && *_cursor == '&') {
                ++_cursor;
                return intersectOp;
            }
            else if (_cursor != _end && *_cursor == '[') {
                // Short-hand for intersectOp when a set follows
                return intersectOp;
            }
            else return ampChar;
        case '-':
            ++_cursor;
            if (_cursor != _end && *_cursor == '-') {
                ++_cursor;
                return setDiffOp;
            }
            else if (_cursor != _end && *_cursor == '[') {
                return setDiffOp;
            }
            else if (_cursor != _end && *_cursor == ']') {
                return hyphenChar;
            }
            else return rangeHyphen;
        case '[':
            ++_cursor;
            if (_cursor != _end && *_cursor == ':') {
                ++_cursor;
                return posixPropertyOpener;
            }
            else return setOpener;
        case ']':
            ++_cursor;
            return setCloser;
        case '\\':
            ++_cursor;
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
    codepoint_t lastCodepointItem;
    
    bool havePendingOperation = false;
    CharsetOperatorKind pendingOperationKind;
    RE * pendingOperand;
    
    // If the first character after the [ is a ^ (caret) then the matching character class is complemented.
    bool negated = false;
    if (_cursor != _end && *_cursor == '^') {
      negated = true;
      ++_cursor;
    }
    throw_incomplete_expression_error_if_end_of_stream();
    // Legacy rule: an unescaped ] may appear as a literal set character
    // if and only if it appears immediately after the opening [ or [^
    if ( *_cursor == ']' && LEGACY_UNESCAPED_RBRAK_RBRACE_ALLOWED) {
        cc->insert(']');
        lastItemKind = CodepointItem;
        lastCodepointItem = static_cast<codepoint_t> (']');
        ++_cursor;
    }
    else if ( *_cursor == '-' && LEGACY_UNESCAPED_HYPHEN_ALLOWED) {
        ++_cursor;
        cc->insert('-');
        lastItemKind = CodepointItem;
        lastCodepointItem = static_cast<codepoint_t> ('-');
                if (*_cursor == '-') throw ParseFailure("Set operator has no left operand.");
    }
    while (_cursor != _end) {
        CharsetOperatorKind op = getCharsetOperator();
        switch (op) {
            case intersectOp: case setDiffOp: {
                if (lastItemKind == NoItem) throw ParseFailure("Set operator has no left operand.");
                if (cc->begin() != cc->end()) subexprs.push_back(cc);
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
                if (lastItemKind == NoItem) throw ParseFailure("Set operator has no right operand.");
                if (cc->begin() != cc->end()) {
                    subexprs.push_back(cc);
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
                if (negated) return makeComplement(newOperand); 
                else return newOperand;
            }
            case setOpener: case posixPropertyOpener: {
                if (lastItemKind != NoItem) {
                    if (cc->begin() != cc->end()) subexprs.push_back(cc);
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
                    subexprs.push_back(pendingOperand);
                    havePendingOperation = false;
                }
                if (op == setOpener) {
                    subexprs.push_back(parse_charset());
                    lastItemKind = SetItem;
                }
                else if (op == posixPropertyOpener) {
                    bool negated = false;
                    if (*_cursor == '^') {
                        negated = true;
                        _cursor++;
                    }
                    RE * posixSet = parse_property_expression();
                    if (negated) posixSet = makeComplement(posixSet);
                    subexprs.push_back(posixSet);
                    lastItemKind = BrackettedSetItem;
                    if (_cursor == _end || *_cursor++ != ':' || _cursor == _end || *_cursor++ != ']')
                        throw ParseFailure("Posix set expression improperly terminated.");
                }
            }
            break;
            case rangeHyphen:
                if (lastItemKind != CodepointItem) throw ParseFailure("Range operator - has illegal left operand.");
                cc->insert_range(lastCodepointItem, parse_codepoint());
                lastItemKind = RangeItem;
                break;
            case hyphenChar:
                cc->insert('-');  
                lastItemKind = CodepointItem;
                lastCodepointItem = static_cast<codepoint_t> ('-');
                break;
            case ampChar:
                cc->insert('&'); 
                lastItemKind = CodepointItem;
                lastCodepointItem = static_cast<codepoint_t> ('&');
                break;
            case backSlash:
                throw_incomplete_expression_error_if_end_of_stream();
                if (isSetEscapeChar(*_cursor)) {
                    subexprs.push_back(parse_escaped_set());
                    lastItemKind = SetItem;
                }
                else {
                    lastCodepointItem = parse_escaped_codepoint();
                    cc->insert(lastCodepointItem);
                    lastItemKind = CodepointItem;
                }
                break;
            case emptyOperator:
                lastCodepointItem = parse_utf8_codepoint();
                cc->insert(lastCodepointItem);
                lastItemKind = CodepointItem;
                break;
        }
    }
    throw ParseFailure("Set expression not properly terminated.");
}


codepoint_t RE_Parser::parse_codepoint() {
    if (_cursor != _end && *_cursor == '\\') {
        _cursor++;
        return parse_escaped_codepoint();
    }
    else {
        return parse_utf8_codepoint();
    }
}

// A backslash escape was found, and various special cases (back reference,
// quoting with \Q, \E, sets (\p, \P, \d, \D, \w, \W, \s, \S), grapheme
// cluster \X have been ruled out.
// It may be one of several possibilities or an error sequence.
// 1. Special control codes (\a, \b, \e, \f, \n, \r, \t, \v)
// 2. General control codes c[@-_a-z?]
// 3. Restricted octal notation 0 - 0777
// 4. General octal notation o\{[0-7]+\}
// 5. General hex notation x\{[0-9A-Fa-f]+\}
// 6. An error for any unrecognized alphabetic escape 
// 7. An escaped ASCII symbol, standing for itself

codepoint_t RE_Parser::parse_escaped_codepoint() {
    codepoint_t cp_value;
    throw_incomplete_expression_error_if_end_of_stream();
    switch (*_cursor) {
        case 'a': ++_cursor; return 0x07; // BEL
        case 'b': ++_cursor; return 0x08; // BS
        case 'e': ++_cursor; return 0x1B; // ESC
        case 'f': ++_cursor; return 0x0C; // FF
        case 'n': ++_cursor; return 0x0A; // LF
        case 'r': ++_cursor; return 0x0D; // CR
        case 't': ++_cursor; return 0x09; // HT
        case 'v': ++_cursor; return 0x0B; // VT
        case 'c': // Control-escape based on next char
            ++_cursor;
            throw_incomplete_expression_error_if_end_of_stream();
            // \c@, \cA, ... \c_, or \ca, ..., \cz
            if (((*_cursor >= '@') && (*_cursor <= '_')) || ((*_cursor >= 'a') && (*_cursor <= 'z'))) {
                cp_value = static_cast<codepoint_t>(*_cursor & 0x1F);
                _cursor++;
                return cp_value;
            }
            else if (*_cursor++ == '?') return 0x7F;  // \c? ==> DEL
            else throw("Illegal \\c escape sequence");
        case '0': // Octal escape:  0 - 0377
            ++_cursor;
            return parse_octal_codepoint(0,3);
        case 'o': 
            ++_cursor;
            throw_incomplete_expression_error_if_end_of_stream();
            if (*_cursor == '{') {
                ++_cursor;
                cp_value = parse_octal_codepoint(1, 7);
                if (_cursor == _end || *_cursor++ != '}') throw ParseFailure("Malformed octal escape sequence");
                return cp_value;
            }
            else {
                throw ParseFailure("Malformed octal escape sequence");
            }
        case 'x': 
            ++_cursor;
            throw_incomplete_expression_error_if_end_of_stream();
            if (*_cursor == '{') {
              ++_cursor;
              cp_value = parse_hex_codepoint(1, 6);
              if (_cursor == _end || *_cursor++ != '}') throw ParseFailure("Malformed hex escape sequence");
              return cp_value;
            }
            else {
                return parse_hex_codepoint(1,2);  // ICU compatibility
            }
        case 'u':
            ++_cursor;
            throw_incomplete_expression_error_if_end_of_stream();
            if (*_cursor == '{') {
                ++_cursor;
                cp_value = parse_hex_codepoint(1, 6);
                if (_cursor == _end || *_cursor++ != '}') throw ParseFailure("Malformed hex escape sequence");
                return cp_value;
            }
            else {
                return parse_hex_codepoint(4,4);  // ICU compatibility
            }
        case 'U': 
            ++_cursor;
            return parse_hex_codepoint(8,8);  // ICU compatibility
        case 'N':
            ++_cursor;
            throw ParseFailure("\\N{...} character name syntax not yet supported.");

        default:
            // Escaped letters should be reserved for special functions.
            if (((*_cursor >= 'A') && (*_cursor <= 'Z')) || ((*_cursor >= 'a') && (*_cursor <= 'z')))
                throw ParseFailure("Undefined or unsupported escape sequence");
            else if ((*_cursor < 0x20) || (*_cursor >= 0x7F))
                throw ParseFailure("Illegal escape sequence");
            else return static_cast<codepoint_t>(*_cursor++);
    }
}


codepoint_t RE_Parser::parse_octal_codepoint(int mindigits, int maxdigits) {
    codepoint_t value = 0;
    int count = 0;
    while (_cursor != _end && count < maxdigits) {
        const char t = *_cursor;
        if (t < '0' || t > '7') {
            break;
        }
        value = value * 8 | (t - '0');
        ++_cursor;
        ++count;
    }
    if (count < mindigits) throw ParseFailure("Octal sequence has too few digits");
    if (value > CC::UNICODE_MAX) throw ParseFailure("Octal value too large");
    return value;
}

codepoint_t RE_Parser::parse_hex_codepoint(int mindigits, int maxdigits) {
    codepoint_t value = 0;
    int count = 0;
    while (_cursor != _end && isxdigit(*_cursor) && count < maxdigits) {
        const char t = *_cursor;
        if (isdigit(t)) {
            value = (value * 16) | (t - '0');
        }
        else {    
            value = (value * 16) | ((t | 32) - 'a') + 10;
        }
        ++_cursor;
        ++count;
    }
    if (count < mindigits) throw ParseFailure("Hexadecimal sequence has too few digits");
    if (value > CC::UNICODE_MAX) throw ParseFailure("Hexadecimal value too large");
    return value;
}


inline void RE_Parser::throw_incomplete_expression_error_if_end_of_stream() const {
    if (_cursor == _end) throw IncompleteRegularExpression();
}

CC * RE_Parser::build_CC(codepoint_t cp) {
    CC * cc = makeCC();
    CC_add_codepoint(cc, cp);
    return cc;
}

void RE_Parser::CC_add_codepoint(CC * cc, codepoint_t cp) {
    if (fModeFlagSet & CASE_INSENSITIVE_MODE_FLAG) {
        caseInsensitiveInsert(cc, cp);
    }
    else cc->insert(cp);
}

void RE_Parser::CC_add_range(CC * cc, codepoint_t lo, codepoint_t hi) {
    if (fModeFlagSet & CASE_INSENSITIVE_MODE_FLAG) {
        caseInsensitiveInsertRange(cc, lo, hi);
    }
    else cc-> insert_range(lo, hi);
}
    
    
}
