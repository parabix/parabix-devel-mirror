/*
 *  Copyright (c) 2019 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#pragma once

#include <cstdlib>
#include <ostream>
#include <string>
#include <util/slab_allocator.h>

namespace pablo {
namespace parse {

enum class TokenType {
    EOF_TOKEN,      // end of file token

    /* === Word / Number Tokens === */
    IDENTIFIER,     // [_a-zA-Z][_a-zA-Z0-9]*
    IF,             // if
    INT_LITERAL,    // [0-9]+
    INT_TYPE,       // i[1-9][0-9]*
    KERNEL,         // kernel
    WHILE,          // while

    /* === Symbols === */
    AND,            // &
    ARROW,          // ->
    ASSIGN,         // =
    BAR,            // |
    COMMA,          // ,
    SIG,            // ::
    TILDE,          // ~
    CARET,          // ^

    /* === Braces === */
    L_PAREN,        // (
    R_PAREN,        // )
    L_SBRACE,       // [
    R_SBRACE,       // ]
    L_BRACE,        // {
    R_BRACE,        // }
    L_ANGLE,        // <
    R_ANGLE,        // >
};

std::string to_string(TokenType const & type);


class Token {
public:

    static Token * Create(TokenType type, std::string const & text, size_t lineNum, size_t colNum, uint64_t value = 0) {
        return new Token(type, text, lineNum, colNum, value);
    }

    static Token * CreateEOF(size_t lineNum, size_t colNum) {
         return Create(TokenType::EOF_TOKEN, "", lineNum, colNum);
    }

    static Token * CreateIdentifier(std::string const & name, size_t lineNum, size_t colNum) {
        return Create(TokenType::IDENTIFIER, name, lineNum, colNum);
    }

    static Token * CreateIf(size_t lineNum, size_t colNum) {
        return Create(TokenType::IF, "if", lineNum, colNum);
    }

    template<typename IntType>
    static Token * CreateIntLiteral(IntType value, size_t lineNum, size_t colNum) {
        return Create(TokenType::INT_LITERAL, std::to_string(value), lineNum, colNum, static_cast<uint64_t>(value));
    }

    static Token * CreateIntType(uint_fast16_t size, size_t lineNum, size_t colNum) {
        return Create(TokenType::INT_TYPE, "i" + std::to_string(size), lineNum, colNum, static_cast<uint64_t>(size));
    }

    static Token * CreateKernel(size_t lineNum, size_t colNum) {
        return Create(TokenType::KERNEL, "kernel", lineNum, colNum);
    }

    static Token * CreateWhile(size_t lineNum, size_t colNum) {
        return Create(TokenType::WHILE, "while", lineNum, colNum);
    }

    static Token * CreateAnd(size_t lineNum, size_t colNum) {
        return Create(TokenType::AND, "&", lineNum, colNum);
    }

    static Token * CreateArrow(size_t lineNum, size_t colNum) {
        return Create(TokenType::ARROW, "->", lineNum, colNum);
    }

    static Token * CreateAssign(size_t lineNum, size_t colNum) {
        return Create(TokenType::ASSIGN, "=", lineNum, colNum);
    }

    static Token * CreateBar(size_t lineNum, size_t colNum) {
        return Create(TokenType::BAR, "|", lineNum, colNum);
    }

    static Token * CreateComma(size_t lineNum, size_t colNum) {
        return Create(TokenType::COMMA, ",", lineNum, colNum);
    }

    static Token * CreateSig(size_t lineNum, size_t colNum) {
        return Create(TokenType::SIG, "::", lineNum, colNum);
    }

    static Token * CreateTilde(size_t lineNum, size_t colNum) {
        return Create(TokenType::TILDE, "~", lineNum, colNum);
    }

    static Token * CreateCaret(size_t lineNum, size_t colNum) {
        return Create(TokenType::CARET, "^", lineNum, colNum);
    }

    static Token * CreateLParen(size_t lineNum, size_t colNum) {
        return Create(TokenType::L_PAREN, "(", lineNum, colNum);
    }

    static Token * CreateRParen(size_t lineNum, size_t colNum) {
        return Create(TokenType::R_PAREN, ")", lineNum, colNum);
    }

    static Token * CreateLSBrace(size_t lineNum, size_t colNum) {
        return Create(TokenType::L_SBRACE, "[", lineNum, colNum);
    }

    static Token * CreateRSBrace(size_t lineNum, size_t colNum) {
        return Create(TokenType::R_SBRACE, "]", lineNum, colNum);
    }

    static Token * CreateLBrace(size_t lineNum, size_t colNum) {
        return Create(TokenType::L_BRACE, "{", lineNum, colNum);
    }

    static Token * CreateRBrace(size_t lineNum, size_t colNum) {
        return Create(TokenType::R_BRACE, "}", lineNum, colNum);
    }

    static Token * CreateLAngle(size_t lineNum, size_t colNum) {
        return Create(TokenType::L_ANGLE, "<", lineNum, colNum);
    }

    static Token * CreateRAngle(size_t lineNum, size_t colNum) {
        return Create(TokenType::R_ANGLE, ">", lineNum, colNum);
    }

public:
    using Allocator = SlabAllocator<Token *>;

    Token() = delete;

    TokenType getType() const { return mType; }
    std::string getText() const { return mText; }
    size_t getLineNum() const { return mLineNum; }
    size_t getColNum() const { return mColNum; }
    uint64_t getValue() const { return mValue; }
private:

    Token(TokenType type, std::string const & text, size_t lineNum, size_t colNum, uint64_t value);

    void * operator new (size_t size) {
        return mAllocator.allocate<uint8_t>(size);
    }

    TokenType       mType;
    std::string     mText;
    size_t          mLineNum;
    size_t          mColNum;
    uint64_t        mValue;

    static Allocator mAllocator;
};

} // namespace pablo::parse
} // namespace pablo

std::ostream & operator << (std::ostream & out, pablo::parse::Token const & token);
