/*
 *  Copyright (c) 2019 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#pragma once

#include <cstdlib>
#include <ostream>
#include <memory>
#include <string>
#include <pablo/parser/source_file.h>
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
    TYPE,           // type
    WHILE,          // while

    /* === Symbols === */
    AND,            // &
    ARROW,          // ->
    ASSIGN,         // =
    BAR,            // |
    COMMA,          // ,
    DOT,            // .
    MINUS,          // -
    PLUS,           // +
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

    IMAGINARY_FIELD // not an actual token, used for error messages
};

std::string to_string(TokenType const & type);


/**
 * Contains data about a single pablo token.
 *
 * Also exposes static methods for constructing new token instances. New token
 * instances are allocated using a slab allocater.
 */
class Token {
public:

    static Token * Create(TokenType type, std::string const & text, size_t lineNum, size_t colNum, std::shared_ptr<SourceFile> const & source, uint64_t value = 0) {
        return new Token(type, text, source, lineNum, colNum, value);
    }

    static Token * CreateEOF(size_t lineNum, size_t colNum, std::shared_ptr<SourceFile> const & source) {
         return Create(TokenType::EOF_TOKEN, "", lineNum, colNum, source);
    }

    static Token * CreateIdentifier(std::string const & name, size_t lineNum, size_t colNum, std::shared_ptr<SourceFile> const & source) {
        return Create(TokenType::IDENTIFIER, name, lineNum, colNum, source);
    }

    static Token * CreateIf(size_t lineNum, size_t colNum, std::shared_ptr<SourceFile> const & source) {
        return Create(TokenType::IF, "if", lineNum, colNum, source);
    }

    template<typename IntType>
    static Token * CreateIntLiteral(std::string const & text, IntType value, size_t lineNum, size_t colNum, std::shared_ptr<SourceFile> const & source) {
        return Create(TokenType::INT_LITERAL, text, lineNum, colNum, source, static_cast<uint64_t>(value));
    }

    static Token * CreateIntType(uint_fast16_t size, size_t lineNum, size_t colNum, std::shared_ptr<SourceFile> const & source) {
        return Create(TokenType::INT_TYPE, "i" + std::to_string(size), lineNum, colNum, source, static_cast<uint64_t>(size));
    }

    static Token * CreateKernel(size_t lineNum, size_t colNum, std::shared_ptr<SourceFile> const & source) {
        return Create(TokenType::KERNEL, "kernel", lineNum, colNum, source);
    }

    static Token * CreateType(size_t lineNum, size_t colNum, std::shared_ptr<SourceFile> const & source) {
        return Create(TokenType::TYPE, "type", lineNum, colNum, source);
    }

    static Token * CreateWhile(size_t lineNum, size_t colNum, std::shared_ptr<SourceFile> const & source) {
        return Create(TokenType::WHILE, "while", lineNum, colNum, source);
    }

    static Token * CreateAnd(size_t lineNum, size_t colNum, std::shared_ptr<SourceFile> const & source) {
        return Create(TokenType::AND, "&", lineNum, colNum, source);
    }

    static Token * CreateArrow(size_t lineNum, size_t colNum, std::shared_ptr<SourceFile> const & source) {
        return Create(TokenType::ARROW, "->", lineNum, colNum, source);
    }

    static Token * CreateAssign(size_t lineNum, size_t colNum, std::shared_ptr<SourceFile> const & source) {
        return Create(TokenType::ASSIGN, "=", lineNum, colNum, source);
    }

    static Token * CreateBar(size_t lineNum, size_t colNum, std::shared_ptr<SourceFile> const & source) {
        return Create(TokenType::BAR, "|", lineNum, colNum, source);
    }

    static Token * CreateComma(size_t lineNum, size_t colNum, std::shared_ptr<SourceFile> const & source) {
        return Create(TokenType::COMMA, ",", lineNum, colNum, source);
    }

    static Token * CreateDot(size_t lineNum, size_t colNum, std::shared_ptr<SourceFile> const & source) {
        return Create(TokenType::DOT, ".", lineNum, colNum, source);
    }

    static Token * CreateMinus(size_t lineNum, size_t colNum, std::shared_ptr<SourceFile> const & source) {
        return Create(TokenType::MINUS, "-", lineNum, colNum, source);
    }

    static Token * CreatePlus(size_t lineNum, size_t colNum, std::shared_ptr<SourceFile> const & source) {
        return Create(TokenType::PLUS, "+", lineNum, colNum, source);
    }

    static Token * CreateSig(size_t lineNum, size_t colNum, std::shared_ptr<SourceFile> const & source) {
        return Create(TokenType::SIG, "::", lineNum, colNum, source);
    }

    static Token * CreateTilde(size_t lineNum, size_t colNum, std::shared_ptr<SourceFile> const & source) {
        return Create(TokenType::TILDE, "~", lineNum, colNum, source);
    }

    static Token * CreateCaret(size_t lineNum, size_t colNum, std::shared_ptr<SourceFile> const & source) {
        return Create(TokenType::CARET, "^", lineNum, colNum, source);
    }

    static Token * CreateLParen(size_t lineNum, size_t colNum, std::shared_ptr<SourceFile> const & source) {
        return Create(TokenType::L_PAREN, "(", lineNum, colNum, source);
    }

    static Token * CreateRParen(size_t lineNum, size_t colNum, std::shared_ptr<SourceFile> const & source) {
        return Create(TokenType::R_PAREN, ")", lineNum, colNum, source);
    }

    static Token * CreateLSBrace(size_t lineNum, size_t colNum, std::shared_ptr<SourceFile> const & source) {
        return Create(TokenType::L_SBRACE, "[", lineNum, colNum, source);
    }

    static Token * CreateRSBrace(size_t lineNum, size_t colNum, std::shared_ptr<SourceFile> const & source) {
        return Create(TokenType::R_SBRACE, "]", lineNum, colNum, source);
    }

    static Token * CreateLBrace(size_t lineNum, size_t colNum, std::shared_ptr<SourceFile> const & source) {
        return Create(TokenType::L_BRACE, "{", lineNum, colNum, source);
    }

    static Token * CreateRBrace(size_t lineNum, size_t colNum, std::shared_ptr<SourceFile> const & source) {
        return Create(TokenType::R_BRACE, "}", lineNum, colNum, source);
    }

    static Token * CreateLAngle(size_t lineNum, size_t colNum, std::shared_ptr<SourceFile> const & source) {
        return Create(TokenType::L_ANGLE, "<", lineNum, colNum, source);
    }

    static Token * CreateRAngle(size_t lineNum, size_t colNum, std::shared_ptr<SourceFile> const & source) {
        return Create(TokenType::R_ANGLE, ">", lineNum, colNum, source);
    }

    static Token * CreateImaginaryField(Token * start, Token * end) {
        if (start == end) {
            return start;
        }
        size_t width = start->mLineNum == end->mLineNum ? (end->mColNum + end->mText.length()) - start->mColNum : 1;
        return Create(TokenType::IMAGINARY_FIELD, std::string(width, ' '),  start->mLineNum, start->mColNum - 1, start->mSourceRef);
    }

public:
    using Allocator = SlabAllocator<Token *>;

    Token() = delete;

    TokenType getType() const { return mType; }
    std::string getText() const { return mText; }
    std::shared_ptr<SourceFile> const & getSourceRef() const { return mSourceRef; }
    size_t getLineNum() const { return mLineNum; }
    size_t getColNum() const { return mColNum; }
    uint64_t getValue() const { return mValue; }
private:

    Token(TokenType type, std::string const & text, std::shared_ptr<SourceFile> source, size_t lineNum, size_t colNum, uint64_t value);

    void * operator new (size_t size) {
        return mAllocator.allocate<uint8_t>(size);
    }

    TokenType                   mType;
    std::string                 mText;
    std::shared_ptr<SourceFile> mSourceRef;
    size_t                      mLineNum;
    size_t                      mColNum;
    uint64_t                    mValue;

    static Allocator mAllocator;
};

} // namespace pablo::parse
} // namespace pablo

std::ostream & operator << (std::ostream & out, pablo::parse::Token const & token);
