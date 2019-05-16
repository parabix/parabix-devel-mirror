/*
 *  Copyright (c) 2019 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#include "token.h"

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
        CASE_AS_STRING(WHILE);          
        CASE_AS_STRING(AND);            
        CASE_AS_STRING(ARROW);          
        CASE_AS_STRING(ASSIGN);         
        CASE_AS_STRING(BAR);            
        CASE_AS_STRING(COMMA);          
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

Token::Token(TokenType type, std::string const & text, size_t lineNum, size_t colNum, uint64_t value)
: mType(type)
, mText(text)
, mLineNum(lineNum)
, mColNum(colNum + 1) // convert from 0-indexed to 1-indexed
, mValue(value)
{}

}
}

std::ostream & operator << (std::ostream & out, pablo::parse::Token const & token) {
    out << pablo::parse::to_string(token.getType()) << " " << token.getLineNum() << ":" << token.getColNum() << " '" << token.getText() << "'";
    return out;
}
