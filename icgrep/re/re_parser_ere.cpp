/*
 *  Copyright (c) 2016 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#include <re/re_parser_ere.h>
#include <re/re_start.h>
#include <re/re_end.h>
#include <re/re_any.h>
#include <re/re_alt.h>
#include <re/re_seq.h>

namespace re {


RE * RE_Parser_ERE::parse_next_item() {
    if (mCursor.noMore() || atany("*?+{|")) return nullptr;
    else if ((mGroupsOpen > 0) && at(')')) return nullptr;
    else if (accept('^')) return makeStart();
    else if (accept('$')) return makeEnd();
    else if (accept('.')) return makeAny();
    else if (accept('(')) return parse_group();
    else if (accept('[')) return parse_bracket_expr();
    else if (accept('\\')) return parse_escaped();
    else return createCC(parse_literal_codepoint());
}

// A parenthesized group.  Input precondition: the opening ( has been consumed
RE * RE_Parser_ERE::parse_group() {
    // Capturing paren group.
    mGroupsOpen++;
    RE * captured = parse_alt();
    mCaptureGroupCount++;
    std::string captureName = "\\" + std::to_string(mCaptureGroupCount);
    Name * const capture  = mMemoizer.memoize(makeCapture(captureName, captured));
    auto key = std::make_pair("", captureName);
    mNameMap.insert(std::make_pair(std::move(key), capture));
    if (!accept(')')) ParseFailure("Closing parenthesis required.");
    mGroupsOpen--;
    return capture;
}

RE * RE_Parser_ERE::parse_escaped() {
    if (accept('b')) return makeWordBoundary();
    if (accept('B')) return makeWordNonBoundary();
    if (accept('s')) return makeWhitespaceSet();
    if (accept('S')) return makeComplement(makeWhitespaceSet());
    if (accept('<')) return makeWordBegin();
    if (accept('>')) return makeWordEnd();
    if (isdigit(*mCursor)) {
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
        return createCC(parse_literal_codepoint());
    }
}


// Parsing items within a bracket expression.
// Items represent individual characters or sets of characters.
// Ranges may be formed by individual character items separated by '-'.
RE * RE_Parser_ERE::parse_bracket_expr () {
    bool negated = accept('^');
    std::vector<RE *> items;
    do {
        if (accept('[')) {
            if (accept('=')) items.push_back(parse_equivalence_class());
            else if (accept('.')) items.push_back(range_extend(parse_collation_element()));
            else if (accept(':')) items.push_back(parse_Posix_class());
            else items.push_back(parse_bracket_expr());
        } else {
            items.push_back(range_extend(makeCC(parse_literal_codepoint())));
        }
    } while (mCursor.more() && !at(']'));
    RE * t = makeAlt(items.begin(), items.end());
    if (!accept(']')) ParseFailure("Expecting ]");
    if (negated) return makeComplement(t);
    else return t;
}
}
