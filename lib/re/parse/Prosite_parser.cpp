/*
 *  Copyright (c) 2016 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#include "Prosite_parser.h"
#include <re/parsers/parser_helper.h>
#include <re/re_alt.h>
#include <re/re_any.h>
#include <re/re_seq.h>
#include <re/re_start.h>
#include <re/re_end.h>
#include <re/re_diff.h>
#include <re/re_rep.h>

namespace re{

    RE * RE_Parser_PROSITE::parse_RE() {
        return parse_seq();
    }

    RE * RE_Parser_PROSITE::parse_seq() {
        std::vector<RE *> seq;
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

    RE * RE_Parser_PROSITE::parse_next_item() {
        RE * re = nullptr;
        if (mCursor.more()) {       
            if (*mCursor == '-') {
                mCursor++;
            }
            switch (*mCursor) {
                case ']': case '}':
                    ParseFailure("Illegal Input");
                case '<': { // the N-terminal of the sequence ('<')
                    mCursor++;
                    return makeStart();
                }
                case '>': { // the C-terminal of the sequence ('>')
                    mCursor++;
                    return makeEnd();
                }
                case '[': { // Ambiguities are indicated by listing between '[ ]' the acceptable amino acids for a given position.
                    mCursor++;
                    return parse_prosite_alt();
                }
                case '{': { // Ambiguities are also indicated by listing between '{ }' the amino acids that are not accepted at a given position.
                    mCursor++;
                    RE * re_temp = parse_prosite_not();
                    return makeDiff(makeAny(), re_temp);
                }
                case 'x': // the 'any' metacharacter
                    mCursor++;
                    return makeAny();
                case '.': // ends the pattern
                    break;
                default:
                    re = createCC(parse_utf8_codepoint());
                    return re;
            }
        }
        return nullptr;
    }

    RE * RE_Parser_PROSITE::parse_prosite_alt() {
        std::vector<RE *> alt;
        while (*mCursor != ']') {
            RE * re = nullptr;
            if (*mCursor == '>') {
                re = makeEnd();
                mCursor++;
            } else {
                re = createCC(parse_utf8_codepoint());
            }
            alt.push_back(re);
        }
        mCursor++;
        return makeAlt(alt.begin(), alt.end());
    }

    RE * RE_Parser_PROSITE::parse_prosite_not() {
        std::vector<RE *> alt;
        while (*mCursor != '}') {
            RE * re = createCC(parse_utf8_codepoint());
            alt.push_back(re);
        }
        mCursor++;
        return makeAlt(alt.begin(), alt.end());
    }
    
    RE * RE_Parser_PROSITE::extend_item(RE * re) {
         if (LLVM_LIKELY(mCursor.more())) {             
             if (*mCursor == '(') {
                int lb = 0, ub = 0;
                std::tie(lb, ub) = parse_range_bound();
                if ((ub != Rep::UNBOUNDED_REP) && (lb > ub)) {
                    ParseFailure("Lower bound cannot exceed upper bound in bounded repetition");
                }
                ++mCursor;
                re = makeRep(re, lb, ub);
            }

         }
         return re;
    }

    std::pair<int, int> RE_Parser_PROSITE::parse_range_bound() {
        int lower_bound = 0, upper_bound = 0;
        mCursor++;
        lower_bound = RE_Parser::parse_int();
        if (*mCursor == ')') {
            upper_bound = lower_bound;
        } else if (*mCursor != ',') {
            ParseFailure("Bad lower bound!");
        } else {
            mCursor++;
            upper_bound = parse_int();
            if (*mCursor != ')') {
                ParseFailure("Bad upper bound!");
            }
        }
        return std::make_pair(lower_bound, upper_bound);
    }

}
