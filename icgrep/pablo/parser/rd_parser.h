/*
 *  Copyright (c) 2019 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#pragma once

#include <pablo/parser/pablo_parser.h>

#include <memory>
#include <vector>
#include <pablo/parser/error.h>
#include <pablo/parser/lexer.h>
#include <pablo/parser/token.h>

namespace pablo {

class PabloBuilder;

namespace parse {

class RecursiveParser final : public PabloParser {
public:
    RecursiveParser() = delete;
    RecursiveParser(std::unique_ptr<Lexer> lexer, std::shared_ptr<ErrorManager> errorDelegate);

    boost::optional<std::vector<std::unique_ptr<PabloKernel>>> parse(std::shared_ptr<SourceFile> sourceFile) override;

    void unparse(std::ostream & out, std::vector<PabloKernel *> const & kernels) override;

private:

    std::unique_ptr<PabloKernel> parseKernelSignature();
    void parseBlock(PabloBuilder & pb);
    void parseStatement(PabloBuilder & pb);
    void parseIf(PabloBuilder & pb);
    void parseWhile(PabloBuilder & pb);
    void parseAssign(PabloBuilder & pb);
    void parseExpression(PabloBuilder & pb);
    void parseTerm(PabloBuilder & pb);
    void parsePrimitive(PabloBuilder & pb);
    void parseLiteral(PabloBuilder & pb);
    void parseVariable(PabloBuilder & pb);
    void parseFunctionCall(PabloBuilder & pb);

    Token * nextToken() { 
        return mIndex < mTokenList.size() ? mTokenList[mIndex++] : nullptr;
    }

    Token * peekToken() const {
        return mIndex < mTokenList.size() ? mTokenList[mIndex] : nullptr; 
    }

    std::unique_ptr<Lexer>          mLexer;
    std::shared_ptr<ErrorManager>   mErrorManager;
    std::vector<Token *>            mTokenList;
    size_t                          mIndex;
    std::shared_ptr<SourceFile>     mCurrentSource;
};

} // namespace pablo::parse
} // namespace pablo
