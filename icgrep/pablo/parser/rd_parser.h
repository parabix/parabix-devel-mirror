/*
 *  Copyright (c) 2019 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#pragma once

#include <pablo/parser/pablo_parser.h>

#include <memory>
#include <vector>
#include <llvm/ADT/StringMap.h>
#include <pablo/parser/error.h>
#include <pablo/parser/lexer.h>
#include <pablo/parser/symbol_table.h>
#include <pablo/parser/token.h>

namespace pablo {

class PabloAST;
class PabloBuilder;

namespace parse {

class RecursiveParser final : public PabloParser {
public:
    RecursiveParser() = delete;
    RecursiveParser(std::unique_ptr<Lexer> lexer, std::shared_ptr<ErrorManager> errorDelegate);

    bool parseKernel(std::shared_ptr<SourceFile> sourceFile, PabloSourceKernel * kernel, std::string const & kernelName) override;

    void unparse(std::ostream & out, std::vector<PabloKernel *> const & kernels) override;

    inline std::shared_ptr<ErrorManager> getErrorManager() const override { return mErrorManager; }

private:

    class ParserState {
    public:
        ParserState(RecursiveParser * parser, size_t index);
        ParserState(RecursiveParser * parser, PabloBuilder * pb, PabloSourceKernel * kernel);
        ~ParserState();

        RecursiveParser *   parser;
        PabloBuilder *      pb;
        PabloSourceKernel * kernel;
        size_t              index;
        SymbolTable *       symbolTable;

        Token * nextToken();
        Token * peekToken();
        Token * peekAhead(size_t n);
        Token * prevToken();

        void pushSymbolTable();
        void popSymbolTable();
    };


    void initializeParser();

    boost::optional<PabloKernelSignature> parseKernelSignature(ParserState & state);
    boost::optional<PabloKernelSignature::SignatureBindings> parseSignatureBindingList(ParserState & state, bool isInput);
    PabloType * parseSigType(ParserState & state);
    PabloType * parseTypeDefinition(ParserState & state);
    PabloAST * parseBlock(ParserState & state);
    PabloAST * parseStatement(ParserState & state);
    PabloAST * parseIf(ParserState & state);
    PabloAST * parseWhile(ParserState & state);
    PabloAST * parseAssign(ParserState & state);
    PabloAST * parseExpression(ParserState & state);
    PabloAST * extendExpression(PabloAST * lhs, ParserState & state);
    PabloAST * parseTerm(ParserState & state);
    PabloAST * extendTerm(PabloAST * lhs, ParserState & state);
    PabloAST * parseArithmeticExpr(ParserState & state);
    PabloAST * extendArithmeticExpr(PabloAST * lhs, ParserState & state);
    PabloAST * parseFactor(ParserState & state);
    PabloAST * parsePrimitive(ParserState & state);
    PabloAST * parseFunctionCall(ParserState & state);
    PabloAST * parseVariable(ParserState & state);
    PabloAST * parseLiteral(ParserState & state);

    PabloAST * createFunctionCall(ParserState & state, Token * func, std::vector<Token *> const & argTokens, std::vector<PabloAST *> const & args);

    std::unique_ptr<Lexer>          mLexer;
    std::shared_ptr<ErrorManager>   mErrorManager;
    std::vector<Token *>            mTokenList;
    std::shared_ptr<SourceFile>     mCurrentSource;
    llvm::StringMap<size_t>         mKernelLocations;
    llvm::StringMap<PabloType *>    mTypeDefTable;
};

} // namespace pablo::parse
} // namespace pablo
