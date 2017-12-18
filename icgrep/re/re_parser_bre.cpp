/*
 *  Copyright (c) 2016 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#include <re/re_parser_bre.h>
#include <re/re_alt.h>
#include <re/re_any.h>
#include <re/re_seq.h>
#include <re/re_start.h>
#include <re/re_end.h>
#include <re/re_assertion.h>
#include <re/re_rep.h>


namespace re {


RE * RE_Parser_BRE::parse_alt() {
    std::vector<RE *> alt;
    do {
        alt.push_back(parse_seq());
    }
    while (accept("\\|"));
    return makeAlt(alt.begin(), alt.end());
}

RE * RE_Parser_BRE::parse_seq() {
    std::vector<RE *> seq;
    if (!mCursor.more() || at("\\|") || ((mGroupsOpen > 0) && at("\\)"))) return makeSeq();
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


RE * RE_Parser_BRE::parse_next_item() {
    if (mCursor.noMore() || at('*') || at("\\?") || at("\\{") || at("\\|")) return nullptr;
    else if ((mGroupsOpen > 0) && at("\\)")) return nullptr;
    else if (accept('^')) return makeStart();
    else if (accept('$')) return makeEnd();
    else if (accept('.')) return makeAny();
    else if (accept("\\(")) return parse_group();
    else if (accept('[')) return parse_bracket_expr();
    else if (accept('\\')) return parse_escaped();
    else return createCC(parse_literal_codepoint());
}

// A parenthesized group.  Input precondition: the opening ( has been consumed
RE * RE_Parser_BRE::parse_group() {
    // Capturing paren group.
    mGroupsOpen++;
    RE * captured = parse_alt();
    mCaptureGroupCount++;
    std::string captureName = "\\" + std::to_string(mCaptureGroupCount);
    Name * const capture  = mMemoizer.memoize(makeCapture(captureName, captured));
    auto key = std::make_pair("", captureName);
    mNameMap.insert(std::make_pair(std::move(key), capture));
    if (!accept("\\)")) ParseFailure("Closing parenthesis required.");
    mGroupsOpen--;
    return capture;
}

// Extend a RE item with one or more quantifiers
RE * RE_Parser_BRE::extend_item(RE * re) {
    int lb, ub;
    if (accept('*')) {lb = 0; ub = Rep::UNBOUNDED_REP;}
    else if (accept("\\?")) {lb = 0; ub = 1;}
    else if (accept("\\+")) {lb = 1; ub = Rep::UNBOUNDED_REP;}
    else if (accept("\\{")) std::tie(lb, ub) = parse_range_bound();
    else {
        // No quantifier found.
        return re;
    }
    re = makeRep(re, lb, ub);
    // The quantified expression may be extended with a further quantifier, e,g., [a-z]\{6,7\}\{2,3\}
    return extend_item(re);
}

std::pair<int, int> RE_Parser_BRE::parse_range_bound() {
    int lb, ub;
    if (accept(',')) {
        lb = 0;
        ub = parse_int();
    } else {
        lb = parse_int();
        if (accept("\\}")) return std::make_pair(lb, lb);
        if (!accept(',')) ParseFailure("Expecting , or }");
        if (accept("\\}")) return std::make_pair(lb, Rep::UNBOUNDED_REP);
        ub = parse_int();
        if (ub < lb) ParseFailure("Upper bound less than lower bound");
    }
    if (accept("\\}")) return std::make_pair(lb, ub);
    else ParseFailure("Expecting \\}");
}

}

