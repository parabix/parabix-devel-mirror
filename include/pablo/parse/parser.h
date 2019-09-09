/*
 *  Copyright (c) 2019 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#pragma once

#include <memory>
#include <unordered_map>
#include <vector>
#include <boost/optional.hpp>
#include <llvm/ADT/StringMap.h>
#include <pablo/parse/error.h>
#include <pablo/parse/kernel_signature.h>
#include <pablo/parse/lexer.h>
#include <pablo/parse/symbol_table.h>
#include <pablo/parse/token.h>

namespace pablo {

class PabloAST;
class PabloBuilder;

namespace parse {

class PabloParser {
public:
    friend class PabloSourceKernel;

    static std::shared_ptr<PabloParser> Create(std::unique_ptr<Lexer> lexer, std::shared_ptr<ErrorManager> errorDelegate) {
        return std::make_shared<PabloParser>(std::move(lexer), std::move(errorDelegate));
    }

    static std::shared_ptr<PabloParser> Create() {
        auto em = ErrorManager::Create();
        return std::make_shared<PabloParser>(Lexer::Create(em), em);
    }

public:
    PabloParser() = delete;
    PabloParser(std::unique_ptr<Lexer> lexer, std::shared_ptr<ErrorManager> errorDelegate);

    ~PabloParser();

    bool parseKernel(std::shared_ptr<SourceFile> sourceFile, PabloSourceKernel * kernel, std::string const & kernelName);

    inline std::shared_ptr<ErrorManager> getErrorManager() const { return mErrorManager; }

private:

    struct SourceData {
        std::shared_ptr<SourceFile>     file;
        std::vector<Token *>            tokenList;
        llvm::StringMap<size_t>         kernelLocations;
        llvm::StringMap<PabloType *>    typeDefTable;
    };

    class ParserState {
    public:
        ParserState(PabloParser * parser, size_t index, SourceData * sourceData);
        ParserState(PabloParser * parser, PabloBuilder * pb, PabloSourceKernel * kernel, SourceData * sourceData);
        ~ParserState();

        Token * nextToken();
        Token * peekToken();
        Token * peekAhead(size_t n);
        Token * prevToken();

        void pushSymbolTable();
        void popSymbolTable();

        PabloParser *       parser;
        PabloBuilder *      pb;
        PabloSourceKernel * kernel;
        size_t              index;
        SymbolTable *       symbolTable;
        SourceData *        sourceData;
    };

    enum class AttributePosition { KERNEL, INPUT, OUTPUT };

    SourceData * generateSourceDate(std::shared_ptr<SourceFile> file, std::vector<Token *> const & tokenList);

    boost::optional<PabloKernelSignature> parseKernelSignature(ParserState & state);
    boost::optional<PabloKernelSignature::SignatureBindings> parseSignatureBindingList(ParserState & state, bool isInput);
    boost::optional<kernel::Attribute> parseAttribute(ParserState & state, AttributePosition attrPos);
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
    PabloAST * parseFactor(ParserState & state);
    PabloAST * parsePrimitive(ParserState & state);
    PabloAST * parseFunctionCall(ParserState & state);
    PabloAST * parseVariable(ParserState & state);
    PabloAST * parseLiteral(ParserState & state);

    PabloAST * createFunctionCall(ParserState & state, Token * func, std::vector<Token *> const & argTokens, std::vector<PabloAST *> const & args);

    std::unique_ptr<Lexer>                          mLexer;
    std::shared_ptr<ErrorManager>                   mErrorManager;
    std::unordered_map<SourceFile *, SourceData *>  mSources;
};

} // namespace pablo::parse
} // namespace pablo
