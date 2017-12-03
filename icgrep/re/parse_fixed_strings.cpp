/*
 *  Copyright (c) 2017 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#include <re/re_parser.h>
#include <re/parse_fixed_strings.h>
#include <re/re_alt.h>
#include <re/re_seq.h>

namespace re {

bool FixedStringParser::accept_alt_mark() {
    if (!mCursor.more() || (*mCursor != '\n')) return false;
    mCursor++;
    return true;
}

RE * FixedStringParser::parse_seq() {
    std::vector<RE *> seq;
    while (mCursor.more() && (*mCursor != '\n')) {
        seq.push_back(createCC(parse_literal_codepoint()));
    }
    return makeSeq(seq.begin(), seq.end());
}

}
