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
#include <algorithm>

namespace re {

RE * RE_Parser::parse(const std::string & regular_expression) {
    RE_Parser parser(regular_expression);
    RE * re = parser.parse_alt(false);
    if (re == nullptr) {
        throw ParseFailure("An unexpected parsing error occurred!");
    }
    return re;
}

inline RE_Parser::RE_Parser(const std::string & regular_expression)
: _cursor(regular_expression.begin())
, _end(regular_expression.end())
{

}

RE * RE_Parser::parse_alt(const bool subexpression) {
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
    else if (subexpression) {
        if (_cursor == _end || *_cursor != ')') {
            throw ParseFailure("Parenthesization error!");
        }
        ++_cursor;
    }
    else if (_cursor != _end) { // !subexpression
        throw ParseFailure("Cannot fully parse statement!");
    }
    return makeAlt(alt.begin(), alt.end());
}

inline RE * RE_Parser::parse_seq() {
    std::vector<RE *> seq;
    for (;;) {
        RE * re = parse_next_token();
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

RE * RE_Parser::parse_next_token() {
    RE * re = nullptr;
    if (_cursor != _end) {
        switch (*_cursor) {
            case '(':
                ++_cursor;
                re = parse_alt(true);
                break;
            case '^':
                ++_cursor;
                re = makeStart();
                break;
            case '$':
                ++_cursor;
                re = makeEnd();
                break;
            case '|': case ')':
                break;
            case '*': case '+': case '?': case '{': 
                throw NothingToRepeat();
            case ']': case '}':
                throw ParseFailure("Illegal metacharacter usage!");
            case '[':
                re = parse_charset();
                break;
            case '.': // the 'any' metacharacter
                re = parse_any_character();
                break;
	    case '\\':  // escape processing
                ++_cursor;
                re = parse_escaped();
		break;
            default:
                re = parse_literal();
                break;
        }
    }
    return re;
}

Any * RE_Parser::parse_any_character() {
    ++_cursor;
    return makeAny();
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
    RE * rep = nullptr;
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

inline RE * RE_Parser::parse_literal() {
    return makeCC(parse_utf8_codepoint());
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
      return makeCC(parse_escaped_codepoint());
}

RE * makeDigitSet() {
  return makeName("Nd", Name::Type::UnicodeCategory);
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

inline RE * RE_Parser::parse_escaped_set() {
    switch (*_cursor) {
        case 'P':
            return makeDiff(makeAny(), parse_unicode_category());
        case 'p':
            return parse_unicode_category();
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
	default:
	    throw ParseFailure("Internal error");
    }
}

unsigned RE_Parser::parse_utf8_codepoint() {
    unsigned c = static_cast<unsigned>(*_cursor++);
    if (c > 0x80) { // if non-ascii
        if (c < 0xC2) {
            throw InvalidUTF8Encoding();
        }
        else { // [0xC2, 0xFF]
            unsigned bytes = 0;
            if (c < 0xE0) { // [0xC2, 0xDF]
                c &= 0x1F;
                bytes = 1;
            }
            else if (c < 0xF0) { // [0xE0, 0xEF]
                c &= 0x0F;
                bytes = 2;
            }
            else { // [0xF0, 0xFF]
                c &= 0x0F;
                bytes = 3;
            }
            while (--bytes) {
                if (++_cursor == _end || (*_cursor & 0xC0) != 0x80) {
                    throw InvalidUTF8Encoding();
                }
                c = (c << 6) | static_cast<unsigned>(*_cursor & 0x3F);
                // It is an error if a 3-byte sequence is used to encode a codepoint < 0x800
                // or a 4-byte sequence is used to encode a codepoint < 0x10000.
                // if (((bytes == 1) && (c < 0x20)) || ((bytes == 2) && (c < 0x10))) {
                if ((c << (bytes - 1)) < 0x20) {
                    throw InvalidUTF8Encoding();
                }
            }
        }
    }
    // It is an error if a 4-byte sequence is used to encode a codepoint 
    // above the Unicode maximum.   
    if (c > CC::UNICODE_MAX) {
        throw InvalidUTF8Encoding();
    }
    return c;
}

Name * RE_Parser::parse_unicode_category() {
    if (++_cursor != _end && *_cursor == '{') {
        const cursor_t start = _cursor + 1;
        for (;;) {
            if (++_cursor == _end) {
                throw UnclosedUnicodeCharacterClass();
            }
            if (*_cursor == '}') {
                break;
            }
            ++_cursor;
        }
        return makeName(std::string(start, _cursor++), Name::Type::UnicodeCategory);
    }
    throw ParseFailure("Incorrect Unicode character class format!");
}

RE * RE_Parser::parse_charset() {
    CC * cc = makeCC();
    bool negated = false;
    cursor_t start = ++_cursor;
    while (_cursor != _end) {
        bool literal = true;
        switch (*_cursor) {
            case '^':
                // If the first character after the [ is a ^ (caret) then the matching character class is complemented.
                if ((start == _cursor) && !negated) {
                    negated = true;
                    start = ++_cursor; // move the start ahead in case the next character is a ] or -
                    literal = false;                    
                }
                break;
            case ']':
                // To include a ], put it immediately after the opening [ or [^; if it occurs later it will
                // close the bracket expression.
                if (start == _cursor) {
                    cc->insert(']');
                    ++_cursor;
                    literal = false;
                    break;
                }
                ++_cursor;
                if (negated) {
                    return makeDiff(makeAny(), cc);
                }
                return cc;
            // The hyphen (-) is not treated as a range separator if it appears first or last, or as the
            // endpoint of a range.
            case '-':
                if (true) {
                    literal = false;
                    const cursor_t next = _cursor + 1;
                    if (next == _end) {
                        throw UnclosedCharacterClass();
                    }
                    if ((start == _cursor) ? (*next != '-') : (*next == ']')) {
                        _cursor = next;
                        cc->insert('-');
                        break;
                    }
                }
                throw ParseFailure("Invalid Lower Range Bound!");
            // case ':':
        }
        if (literal) {
            unsigned low;
            if (parse_charset_literal(low)) {
                // the previous literal allows for a - to create a range; test for it
                if (_cursor == _end) {
                    break; // out of loop to failure handling
                }
                if (*_cursor == '-') { // in range unless the next character is a ']'
                    if (++_cursor == _end) {
                        break; // out of loop to failure handling
                    }
                    if (*_cursor != ']') {
                        unsigned high;
                        if (!parse_charset_literal(high)) {
                            throw ParseFailure("Invalid Upper Range Bound!");
                        }
                        cc->insert_range(low, high);
                    }
                    else {
                        cc->insert(low);
                        cc->insert('-');
                    }
                    continue;
                }
            }
            cc->insert(low);
        }
    }
    throw UnclosedCharacterClass();
}

inline bool RE_Parser::parse_charset_literal(unsigned & literal) {
    if (_cursor == _end) {
        return false;
    }
    if (*_cursor == '\\') {
		_cursor++;
		literal = parse_escaped_codepoint();
		return true;
    }
    else {
        literal = parse_utf8_codepoint();
        return true;
    }
    return false;
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

unsigned RE_Parser::parse_escaped_codepoint() {
	unsigned cp_value;
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
				cp_value = static_cast<unsigned>(*_cursor & 0x1F);
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
		default:
			if (((*_cursor >= 'A') && (*_cursor <= 'Z')) || ((*_cursor >= 'a') && (*_cursor <= 'z')))
				throw ParseFailure("Undefined or unsupported escape sequence");
			else if ((*_cursor < 0x20) || (*_cursor >= 0x7F))
				throw ParseFailure("Illegal escape sequence");
			else return static_cast<unsigned>(*_cursor++);
	}
}


unsigned RE_Parser::parse_octal_codepoint(int mindigits, int maxdigits) {
	unsigned value = 0;
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

unsigned RE_Parser::parse_hex_codepoint(int mindigits, int maxdigits) {
	unsigned value = 0;
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

}
