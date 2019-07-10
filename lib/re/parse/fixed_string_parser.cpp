/*
 *  Copyright (c) 2018 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#include <re/parsers/parser.h>
#include "fixed_string_parser.h"
#include <re/re_alt.h>
#include <re/re_seq.h>

namespace re {

RE * FixedStringParser::parse_alt() {
    std::vector<RE *> alt;
    do {
        alt.push_back(parse_seq());
    }
    while (accept('\n'));
    return makeAlt(alt.begin(), alt.end());
}

RE * FixedStringParser::parse_seq() {
    std::vector<RE *> seq;
    while (mCursor.more() && (!at('\n'))) {
        seq.push_back(createCC(parse_literal_codepoint()));
    }
    return makeSeq(seq.begin(), seq.end());
}

}
