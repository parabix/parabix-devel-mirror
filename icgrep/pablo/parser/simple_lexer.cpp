/*
 *  Copyright (c) 2019 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#include "simple_lexer.h"

#include <cctype>
#include <pablo/parser/error_text.h>

namespace pablo {
namespace parse {

inline static bool isTokenSeparator(char c) {
    return !(std::isalnum(c) || c == '_');
}

std::unique_ptr<std::vector<Token *>> SimpleLexer::tokenize(SourceFile & sourceFile) {
    mCurrentSource = &sourceFile;
    mErrorManager.setReferences(mCurrentSource, &mCurrentLineNum, &mCurrentColNum);
    std::unique_ptr<std::vector<Token *>> tokenList(new std::vector<Token *>{});
    while (sourceFile.nextLine(mCurrentLine)) {
        mCurrentColNum = 0;
        mCurrentLineNum++;
        Token * token = nullptr;
        char c;
        while (mCurrentColNum < mCurrentLine.length() && mErrorManager.canContinue()) {
            c = mCurrentLine[mCurrentColNum];
            if (std::isspace(c)) {
                mCurrentColNum++;
                continue;
            } else if (c == '#') {
                mCurrentColNum = mCurrentLine.length();
                continue;
            } else if (std::isdigit(c)) {
                token = extractIntLiteral();
            } else if (std::isalpha(c) || c == '_') {
                token = extractText();
            } else {
                token = extractSymbol();
            }
            if (token != nullptr) {
                tokenList->push_back(token);
            }
        }
    }

    if (mErrorManager.hasErrors()) {
        return nullptr;
    }
    tokenList->push_back(Token::CreateEOF(mCurrentLineNum == 0 ? 1 : mCurrentLineNum, mCurrentColNum));
    return tokenList;
}

Token * SimpleLexer::extractText() {
    const size_t col = mCurrentColNum;
    std::string builder{};
    bool canBeIntType = mCurrentLine[mCurrentColNum] == 'i';
    while (mCurrentColNum <= mCurrentLine.length() &&
           (std::isalnum(mCurrentLine[mCurrentColNum]) || mCurrentLine[mCurrentColNum] == '_'))
    {
        const char c = mCurrentLine[mCurrentColNum];
        builder.push_back(c);
        canBeIntType &= std::isdigit(c) || (c == 'i' && mCurrentColNum == col);
        mCurrentColNum++;
    }

    if (canBeIntType && builder.length() > 1) {
        return Token::CreateIntType(std::stoi(builder.substr(1)), mCurrentLineNum, col);
    }
    if (builder == "kernel") {
        return Token::CreateKernel(mCurrentLineNum, col);
    } else if (builder == "if") {
        return Token::CreateIf(mCurrentLineNum, col);
    } else if (builder == "while") {
        return Token::CreateWhile(mCurrentLineNum, col);
    } else {
        return Token::CreateIdentifier(builder, mCurrentLineNum, col);
    }
}

Token * SimpleLexer::extractIntLiteral() {
    const size_t col = mCurrentColNum;
    size_t consumedCount = 0;
    int64_t value = std::stol(mCurrentLine.substr(mCurrentColNum).to_string(), &consumedCount, /* auto base */ 0);
    mCurrentColNum += consumedCount;
    if (consumedCount == 0 || (mCurrentColNum < mCurrentLine.length() && !isTokenSeparator(mCurrentLine[mCurrentColNum]))) {
        mErrorManager.logError(errtxt_IllegalIntegerLiteral(), mCurrentSource, mCurrentLineNum, col);
        return nullptr;
    }
    return Token::CreateIntLiteral(value, mCurrentLineNum, col);
}

Token * SimpleLexer::extractSymbol() {
    const size_t col = mCurrentColNum;
    const char c = mCurrentLine[mCurrentColNum];
    Token * token = nullptr;
    switch (c) {
    case '&':
        token = Token::CreateAnd(mCurrentLineNum, col);
        break;
    case '=':
        token = Token::CreateAssign(mCurrentLineNum, col);
        break;
    case '|':
        token = Token::CreateBar(mCurrentLineNum, col);
        break;
    case ',':
        token = Token::CreateComma(mCurrentLineNum, col);
        break;
    case '~':
        token = Token::CreateTilde(mCurrentLineNum, col);
        break;
    case '^':
        token = Token::CreateCaret(mCurrentLineNum, col);
        break;
    case '(':
        token = Token::CreateLParen(mCurrentLineNum, col);
        break;
    case ')':
        token = Token::CreateRParen(mCurrentLineNum, col);
        break;
    case '[':
        token = Token::CreateLSBrace(mCurrentLineNum, col);
        break;
    case ']':
        token = Token::CreateRSBrace(mCurrentLineNum, col);
        break;
    case '{':
        token = Token::CreateLBrace(mCurrentLineNum, col);
        break;
    case '}':
        token = Token::CreateRBrace(mCurrentLineNum, col);
        break;
    case '<':
        token = Token::CreateLAngle(mCurrentLineNum, col);
        break;
    case '>':
        token = Token::CreateRAngle(mCurrentLineNum, col);
        break;
    case ':':
        if (mCurrentColNum + 1 < mCurrentLine.length() && mCurrentLine[mCurrentColNum + 1] == ':') {
            token = Token::CreateSig(mCurrentLineNum, col);
        } else {
            mErrorManager.logError(errtxt_IllegalSymbol(c), "::");
        }
        mCurrentColNum++;
        break;
    case '-':
        if (mCurrentColNum + 1 < mCurrentLine.length() && mCurrentLine[mCurrentColNum + 1] == '>') {
            token = Token::CreateArrow(mCurrentLineNum, col);
        } else {
            mErrorManager.logError(errtxt_IllegalSymbol(c), "->");
        }
        mCurrentColNum++;
        break;
    default:
        mErrorManager.logError(errtxt_IllegalSymbol(c));
        break;
    }
    mCurrentColNum++;
    return token;
}

SimpleLexer::SimpleLexer(ErrorContext const & errorContext)
: Lexer()
, mErrorManager(errorContext)
, mCurrentSource(nullptr)
, mCurrentLine()
, mCurrentLineNum(0)
, mCurrentColNum(0)
{}

} // namespace pablo::parse
} // namespace pablo
