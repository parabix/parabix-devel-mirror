/*
 *  Copyright (c) 2019 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 */

#pragma once

#include <cassert>
#include <kernel/core/relationship.h>
#include <pablo/builder.hpp>
#include <pablo/pabloAST.h>
#include <pablo/parse/pablo_source_kernel.h>
#include <pablo/parse/token.h>
#include <pablo/parse/source_file.h>

namespace pablo {
namespace parse {

class ParserState {
public:
    ParserState(std::shared_ptr<SourceFile> source, std::shared_ptr<ErrorManager> em);
    ~ParserState();

    Token * nextToken();
    Token * peekToken();
    Token * peekAhead(size_t n);
    Token * prevToken();

    PabloBuilder * builder() noexcept { assert(mBuilder); return mBuilder; }
    size_t index() const noexcept { return mIndex; }

    void setBuilder(PabloBuilder * builder) { mBuilder = builder; }
    void setIndex(size_t i) noexcept { mIndex = i; }

private:
    size_t                        mIndex;
    PabloBuilder *                mBuilder;
    std::shared_ptr<ErrorManager> mErrorManager;
    SymbolTable *                 mSymbolTable;
};


class Parser {
public:

    /**
     * Constructs a kernel's I/O bindings and attributes by parsing a the
     * signature of a kernel with a specified name in a given source file.
     *
     * Returns a `ParserState` which may be used to parse the body of the
     * kernel at a later time.
     */
    static std::unique_ptr<ParserState> constructKernelSignature(
        PabloSourceKernel * kernel,
        std::string const & kernelName,
        std::shared_ptr<SourceFile> source,
        std::shared_ptr<ErrorManager> em,
        std::vector<kernel::Relationship *> const & inputArguments,
        std::vector<kernel::Relationship *> const & outputArguments);


    /**
     * Parses a kernel body using the `PabloBuilder` supplied to `state` to
     * construct each `PabloAST` node.
     *
     * Returns `true` if the kernel body was successfully built. Otherwise,
     * `false` is returned and the `ErrorManager` within `state` should be
     * queried for more information about the error.
     */
    static bool constructKernelBody(ParserState & state);

private:
    static PabloAST * parseBlock(ParserState & state);
    static PabloAST * parseStatement(ParserState & state);
    static PabloAST * parseIf(ParserState & state);
    static PabloAST * parseWhile(ParserState & state);
    static PabloAST * parseAssign(ParserState & state);
    static PabloAST * parseExpression(ParserState & state);
    static PabloAST * extendExpression(PabloAST * lhs, ParserState & state);
    static PabloAST * parseTerm(ParserState & state);
    static PabloAST * extendTerm(PabloAST * lhs, ParserState & state);
    static PabloAST * parseFactor(ParserState & state);
    static PabloAST * parsePrimitive(ParserState & state);
    static PabloAST * parseFunctionCall(ParserState & state);
    static PabloAST * parseVariable(ParserState & state);
    static PabloAST * parseLiteral(ParserState & state);

    static PabloAST * createFunctionCall(ParserState & state, Token * func, std::vector<Token *> const & argTokens, std::vector<PabloAST *> const & args);
};

} // namespace pablo::parse
} // namespace pablo
