/*
 *  Copyright (c) 2018 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#include "GLOB_parser.h"
#include <re/re_cc.h>
#include <re/re_range.h>
#include <re/re_alt.h>
#include <re/re_seq.h>
#include <re/re_diff.h>
#include <re/re_rep.h>
#include <re/re_utility.h>

namespace re {

//  GLOB parsing for Filename expansion in accord with Posix rules
//  IEEE-1003.1 XCU section  2.13 Pattern Matching Notation

// GLOB parsing:  no alternation operators.
RE * FileGLOB_Parser::parse_alt() {
    return parse_seq();
}

RE * FileGLOB_Parser::parse_seq() {
    mExcludeDotContext = true;
    std::vector<RE *> seq;
    if (!mCursor.more()) return makeSeq();
    for (;;) {
        RE * re = parse_next_item();
        if (re == nullptr) {
            break;
        }
        seq.push_back(re);
    }
    return makeSeq(seq.begin(), seq.end());
}

const codepoint_t NUL = 0;

// We use symbolic constants for the path separator characters.
// PATTERN_PATH_SEPARATOR is the path separator character in GLOB patterns.
// FILE_PATH_SEPARATOR is the path separator in actual filenames.
//
const codepoint_t PATTERN_PATH_SEPARATOR = '/';
#ifdef WINDOWS
const codepoint_t FILE_PATH_SEPARATOR = '\\';
#else
const codepoint_t FILE_PATH_SEPARATOR = '/';
#endif
    
//
// GLOB metacharacters:
//  ? stands for any one character, except as below
//  * stands for zero or more characters, except as below
//  Exceptions:
//     Neither ? nore * may match any of the following.
//     (a) a NUL character.
//     (b) a slash ('/')
//     (c) a period ('.') when it appears as the first character of a
//         filename or path component (immediately after a '/').

static RE * makeAnyCC() {
    return makeComplement(makeCC(makeCC(NUL), makeCC(FILE_PATH_SEPARATOR)));
}

static RE * makeAnyButDot() {
    return makeComplement(makeCC(makeCC(makeCC(NUL), makeCC(FILE_PATH_SEPARATOR)), makeCC('.')));
}
    
RE * FileGLOB_Parser::parse_next_item() {
    if (mCursor.noMore()) return nullptr;
    if (accept('?')) {
        if (mExcludeDotContext) {
            mExcludeDotContext = false;  // After this ? metacharacter.
            return makeAnyButDot();
        }
        else return makeAnyCC();
    } else if (accept('*')) {
        // Accept and discard any additional (reundant) '*' characters,
        // and then check for a ? character.
        while (accept('*')) {}
        if (at('?')) {
            // Accept the ? and build the ?* regexp in place of *?
            RE * first = parse_next_item();  // Checks/sets mExcludeDotContext as required.
            return makeSeq({first, makeRep(makeAnyCC(), 0, Rep::UNBOUNDED_REP)});
        } else if (mExcludeDotContext) {
            mExcludeDotContext = false;   // After this * metacharacter.
            return makeRep(makeSeq({makeAnyButDot(), makeRep(makeAnyCC(), 0, Rep::UNBOUNDED_REP)}), 0, 1);
        } else {
            return makeRep(makeAnyCC(), 0, Rep::UNBOUNDED_REP);
        }
    } else if (accept('\\')) {
        mExcludeDotContext = at('/');
        return makeCC(parse_literal_codepoint());
    } else if (accept('[')) {
        RE * cc = parse_bracket_expr();
        mExcludeDotContext = false; // After the bracket expression, which cannot match /
        return cc;
    } else if (accept(PATTERN_PATH_SEPARATOR)) {
        mExcludeDotContext = true;
        return makeCC(FILE_PATH_SEPARATOR);
    } else {
        mExcludeDotContext = false;
        return makeCC(parse_literal_codepoint());
    }
}

// Parsing items within a bracket expression.
// Items represent individual characters or sets of characters.
// Ranges may be formed by individual character items separated by '-'.
// GLOB notation uses the "!" character for set negation.
// Note that there are no backslash escapes for GLOB bracket expressions.
RE * FileGLOB_Parser::parse_bracket_expr () {
    bool negated = accept('!');
    std::vector<RE *> items;
    do {
        if (accept(PATTERN_PATH_SEPARATOR)) {
            // A path separator ('/') is not legal in a GLOB bracket expr.  If a
            // found, the items and the bracket expr are interpreted as ordinary characters.
            // See section 2.13.3
            std::vector<RE *> seqItems;
            seqItems.push_back(makeCC('['));
            if (negated) seqItems.push_back(makeCC('!'));
            for (auto a : items) {
                if (Range * rg = llvm::dyn_cast<Range>(a)) {
                    seqItems.push_back(rg->getLo());
                    seqItems.push_back(makeCC('-'));
                    seqItems.push_back(rg->getHi());
                } else {
                    seqItems.push_back(a);
                }
            }
            seqItems.push_back(makeCC(FILE_PATH_SEPARATOR));
            return makeSeq(seqItems.begin(), seqItems.end());
        }
        if (accept('[')) {
            if (accept('=')) items.push_back(parse_equivalence_class());
            else if (accept('.')) items.push_back(range_extend(parse_collation_element()));
            else if (accept(':')) items.push_back(parse_Posix_class());
            else items.push_back(parse_bracket_expr());
        } else {
            items.push_back(range_extend(makeCC(parse_literal_codepoint())));
        }
    } while (mCursor.more() && !at(']'));
    require(']');
    if (negated) {
        if (mExcludeDotContext) {
            // In mExcludeDotContext, a dot cannot be matched by a negated bracket expression
            items.push_back(makeCC('.'));
        }
        return makeComplement(makeAlt(items.begin(), items.end()));
    }
    else {
        RE * t = makeAlt(items.begin(), items.end());
        if (mExcludeDotContext) {
            t = makeDiff(t, makeCC('.'));
        }
        return t;
    }
}

//  Given an individual character expression, check for and parse
//  a range extension if one exists, or return the individual expression.
RE * FileGLOB_Parser::range_extend(RE * char_expr1) {
    // A range extension is signalled by a hyphen '-'.
    if (!accept('-')) return char_expr1;
    RE * char_expr2 = nullptr;
    if (accept('[')) {
        if (accept('.')) char_expr2 = parse_collation_element();
        else ParseFailure("Error in range expression");
    } else {
        char_expr2 = makeCC(parse_literal_codepoint());
    }
    return makeRange(char_expr1, char_expr2);
}

}

