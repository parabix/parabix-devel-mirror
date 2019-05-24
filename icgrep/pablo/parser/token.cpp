/*
 *  Copyright (c) 2019 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#include "token.h"

#include <iomanip>

#define CASE_AS_STRING(TOKEN_TYPE) case TokenType::TOKEN_TYPE: return #TOKEN_TYPE

namespace pablo {
namespace parse {

Token::Allocator Token::mAllocator;

std::string to_string(TokenType const & type) {
    switch (type) {
        CASE_AS_STRING(IDENTIFIER);
        CASE_AS_STRING(IF);
        CASE_AS_STRING(INT_LITERAL);
        CASE_AS_STRING(INT_TYPE);
        CASE_AS_STRING(KERNEL);
        CASE_AS_STRING(TYPE);
        CASE_AS_STRING(WHILE);
        CASE_AS_STRING(AND);
        CASE_AS_STRING(ARROW);
        CASE_AS_STRING(ASSIGN);
        CASE_AS_STRING(BAR);
        CASE_AS_STRING(COMMA);
        CASE_AS_STRING(DOT);
        CASE_AS_STRING(MUTABLE_ASSIGN);
        CASE_AS_STRING(SIG);
        CASE_AS_STRING(TILDE);
        CASE_AS_STRING(CARET);
        CASE_AS_STRING(L_PAREN);
        CASE_AS_STRING(R_PAREN);
        CASE_AS_STRING(L_SBRACE);
        CASE_AS_STRING(R_SBRACE);
        CASE_AS_STRING(L_BRACE);
        CASE_AS_STRING(R_BRACE);
        CASE_AS_STRING(L_ANGLE);
        CASE_AS_STRING(R_ANGLE);
        CASE_AS_STRING(EOF_TOKEN);
    default:
        assert ("illegal token type" && false);
        return "";
    }
}


Token::Token(TokenType type, std::string const & text, std::shared_ptr<SourceFile> source, size_t lineNum, size_t colNum, uint64_t value)
: mType(type)
, mText(text)
, mSourceRef(std::move(source))
, mLineNum(lineNum)
, mColNum(colNum + 1) // convert from 0-indexed to 1-indexed
, mValue(value)
{}

} // namespace pablo::parse
} // namespace pablo


std::ostream & operator << (std::ostream & out, pablo::parse::Token const & token) {
    out << std::setw(16) << std::left
        << pablo::parse::to_string(token.getType()) << " "
        << std::setw(24) << std::left
        << "'" + token.getText() + "'"
        << " @ "
        << token.getSourceRef()->getFilename() << ":"
        << token.getLineNum() << ":"
        << token.getColNum() ;
    return out;
}
