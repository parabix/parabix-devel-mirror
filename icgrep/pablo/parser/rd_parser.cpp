/*
 *  Copyright (c) 2019 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#include "rd_parser.h"

#include <pablo/builder.hpp>

namespace pablo {
namespace parse {

boost::optional<std::vector<std::unique_ptr<PabloKernel>>> RecursiveParser::parse(std::shared_ptr<SourceFile> sourceFile) {
    {
        auto optList = mLexer->tokenize(sourceFile);
        if (!optList) {
            return boost::none;
        }
        mTokenList = std::move(optList.value());
    }
    mCurrentSource = sourceFile;
    std::vector<std::unique_ptr<PabloKernel>> kernels{};
    while (peekToken() && peekToken()->getType() != TokenType::EOF_TOKEN) {
        auto kernel = parseKernelSignature();
        if (!mErrorManager->shouldContinue() || kernel == nullptr) {
            return boost::none;
        }
        PabloBuilder pb(kernel->getEntryScope());
        parseBlock(pb);
        if (!mErrorManager->shouldContinue()) {
            return boost::none;
        }
        kernels.push_back(std::move(kernel));
    }

    return kernels;
}


void RecursiveParser::unparse(std::ostream & out, std::vector<PabloKernel *> const & kernels) {
    // TODO: implement me!
}


std::unique_ptr<PabloKernel> RecursiveParser::parseKernelSignature() {
    assert (peekToken());
    Token * const t = nextToken();
    if (t->getType() != TokenType::KERNEL) {
        mErrorManager->logFatalError(t, "unexpected top level declaration", "expected 'kernel'");
        return nullptr;
    }
    return nullptr;
}


void RecursiveParser::parseBlock(PabloBuilder & pb) {

}


RecursiveParser::RecursiveParser(std::unique_ptr<Lexer> lexer, std::shared_ptr<ErrorManager> errorDelegate)
: PabloParser()
, mLexer(std::move(lexer))
, mErrorManager(std::move(errorDelegate))
, mTokenList()
, mIndex(0)
, mCurrentSource(nullptr)
{}

} // namespace pablo::parse
} // namespace pablo
