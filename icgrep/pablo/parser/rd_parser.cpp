/*
 *  Copyright (c) 2019 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#include "rd_parser.h"

#include <kernels/core/streamset.h>
#include <llvm/Support/Casting.h>
#include <pablo/builder.hpp>
#include <pablo/branch.h>
#include <pablo/pe_ones.h>
#include <pablo/pe_zeroes.h>
#include <pablo/parser/error_text.h>
#include <pablo/ps_assign.h>


#define TOKEN_CHECK(TOKEN, EXPECTED_TYPE, HINT) {\
Token * __token_ = (TOKEN); \
if (__token_->getType() != EXPECTED_TYPE) { \
    mErrorManager->logError(__token_, errtxt_UnexpectedToken(__token_), HINT); \
    return nullptr; \
}}


#define TOKEN_CHECK_FATAL(TOKEN, EXPECTED_TYPE, HINT) {\
Token * __token_ = (TOKEN); \
if (__token_->getType() != EXPECTED_TYPE) { \
    mErrorManager->logFatalError(__token_, errtxt_UnexpectedToken(__token_), HINT); \
    return nullptr; \
}}


#define TOKEN_CHECK_BOOST_NONE(TOKEN, EXPECTED_TYPE, HINT) {\
Token * __token_ = (TOKEN); \
if (__token_->getType() != EXPECTED_TYPE) { \
    mErrorManager->logError(__token_, errtxt_UnexpectedToken(__token_), HINT); \
    return boost::none; \
}}


#define TOKEN_CHECK_FATAL_BOOST_NONE(TOKEN, EXPECTED_TYPE, HINT) {\
Token * __token_ = (TOKEN); \
if (__token_->getType() != EXPECTED_TYPE) { \
    mErrorManager->logFatalError(__token_, errtxt_UnexpectedToken(__token_), HINT); \
    return boost::none; \
}}


namespace pablo {
namespace parse {


Token * RecursiveParser::ParserState::nextToken() {
    assert (parser->mTokenList[index]);
    Token * t = parser->mTokenList[index];
    if (t->getType() != TokenType::EOF_TOKEN) {
        assert (index < parser->mTokenList.size() - 1);
        index++;
    }
    return t;
}


Token * RecursiveParser::ParserState::peekToken() {
    assert (parser->mTokenList[index]);
    return parser->mTokenList[index];
}


Token * RecursiveParser::ParserState::peekAhead(size_t n) {
    assert (parser->mTokenList[index]);
    // n - 1 as we are already pointing at the next token
    // peekAhead(1) is equivalent to peekToken()
    if (index + n - 1 >= parser->mTokenList.size()) {
        return parser->mTokenList.back();
    }
    return parser->mTokenList[index + n - 1];
}


Token * RecursiveParser::ParserState::prevToken() {
    assert (parser->mTokenList[index]);
    if (index > 0) {
        assert (parser->mTokenList[index - 1]);
        return parser->mTokenList[index - 1];
    }
    return parser->mTokenList[index];
}


void RecursiveParser::ParserState::pushSymbolTable() {
    auto sym = new SymbolTable(parser->mErrorManager, pb, symbolTable);
    symbolTable = sym;
}


void RecursiveParser::ParserState::popSymbolTable() {
    assert (symbolTable);
    auto sym = symbolTable;
    symbolTable = sym->getParent();
    delete sym;
}


RecursiveParser::ParserState::ParserState(RecursiveParser * parser, PabloBuilder * pb, PabloSourceKernel * kernel)
: parser(parser)
, pb(pb)
, kernel(kernel)
, index(0)
, symbolTable(nullptr)
{
    pushSymbolTable();
}

RecursiveParser::ParserState::~ParserState() {
    assert (symbolTable);
    popSymbolTable();
    assert (symbolTable == nullptr);
}


bool RecursiveParser::parseKernel(std::shared_ptr<SourceFile> sourceFile, PabloSourceKernel * kernel, std::string const & kernelName) {
    assert (kernel);

    if (mTokenList.empty()) {
        auto optList = mLexer->tokenize(sourceFile);
        if (!optList) {
            return false;
        }
        mTokenList = std::move(optList.value());
        locateKernels();
    }
    if (!mErrorManager->shouldContinue()) {
        return false;
    }
    mCurrentSource = sourceFile;
    PabloBuilder pb(kernel->getEntryScope());
    ParserState state(this, &pb, kernel);
    if (!kernelName.empty()) {
        auto location = mKernelLocations.find(kernelName);
        if (location == mKernelLocations.end()) {
            mErrorManager->logTextError(mCurrentSource, "No defintion for '" + kernelName + "' found.");
            return false;
        }
        state.index = location->getValue();
    }
    auto signature = parseKernelSignature(state);
    if (!signature || !mErrorManager->shouldContinue()) {
        return false;
    }
    // TODO: check signature
    auto block = parseBlock(state);
    if (block == nullptr || mErrorManager->hasErrors()) {
        return false;
    }
    return true;
}


void RecursiveParser::locateKernels() {
    for (size_t i = 0, j = 1; j < mTokenList.size(); i++, j++) {
        Token * const kernel = mTokenList[i];
        Token * const identifier = mTokenList[j];
        if (kernel->getType() == TokenType::KERNEL &&  identifier->getType() == TokenType::IDENTIFIER) {
            auto rt = mKernelLocations.try_emplace(identifier->getText(), i);
            if (!rt.second) {
                size_t prev = mKernelLocations.lookup(identifier->getText());
                Token * p = mTokenList[prev];
                std::string loc = std::to_string(p->getLineNum()) + ":" + std::to_string(p->getColNum());
                mErrorManager->logFatalError(identifier, "duplicate kernel declaration", "previous declaration at " + loc);
            }
        }
    }
}


void RecursiveParser::unparse(std::ostream & out, std::vector<PabloKernel *> const & kernels) {
    // TODO: implement me!
}


boost::optional<PabloKernelSignature> RecursiveParser::parseKernelSignature(ParserState & state) {
    Token * const kernel = state.nextToken();
    if (kernel->getType() != TokenType::KERNEL) {
        mErrorManager->logFatalError(kernel, "unexpected top level declaration", "expected 'kernel'");
        return boost::none;
    }
    Token * const name = state.nextToken();
    TOKEN_CHECK_FATAL_BOOST_NONE(name, TokenType::IDENTIFIER, "expected a kernel name");
    TOKEN_CHECK_FATAL_BOOST_NONE(state.nextToken(), TokenType::SIG, "expected '::'");

    auto inputBindings = parseSignatureBindingList(state, true);
    if (!inputBindings) {
        return boost::none;
    }
    TOKEN_CHECK_FATAL_BOOST_NONE(state.nextToken(), TokenType::ARROW, "expected '->'");
    auto outputBindings = parseSignatureBindingList(state, false);
    if (!outputBindings) {
        return boost::none;
    }
    return PabloKernelSignature(name->getText(), inputBindings.value(), outputBindings.value());
}


boost::optional<PabloKernelSignature::SignatureBindings> RecursiveParser::parseSignatureBindingList(ParserState & state, bool isInput) {
    TOKEN_CHECK_FATAL_BOOST_NONE(state.nextToken(), TokenType::L_SBRACE, "expected a '['");
    PabloKernelSignature::SignatureBindings bindings{};
    while (state.peekToken()->getType() != TokenType::R_SBRACE && state.peekToken()->getType() != TokenType::EOF_TOKEN) {
        auto * t = parseSigType(state);
        if (t == nullptr || !mErrorManager->shouldContinue()) {
            return boost::none;
        }
        Token * const name = state.nextToken();
        TOKEN_CHECK_FATAL_BOOST_NONE(name, TokenType::IDENTIFIER, "expected an identifier");
        if (isInput) {
            state.symbolTable->addInputVar(name, t, state.kernel);
        } else {
            state.symbolTable->addOutputVar(name, t, state.kernel);
        }
        if (!mErrorManager->shouldContinue()) {
            return boost::none;
        }
        bindings.emplace_back(name->getText(), t);
        if (state.peekToken()->getType() != TokenType::R_SBRACE) {
            TOKEN_CHECK_FATAL_BOOST_NONE(state.nextToken(), TokenType::COMMA, "expected a ','");
        }
    }
    TOKEN_CHECK_FATAL_BOOST_NONE(state.nextToken(), TokenType::R_SBRACE, "expected a ']'");
    return bindings;
}


PabloKernelSignature::Type * RecursiveParser::parseSigType(ParserState & state) {
    Token * const t = state.nextToken();
    TokenType const type = t->getType();
    if (type == TokenType::INT_TYPE) {
        return new PabloKernelSignature::IntType(t->getValue());
    } else if (type == TokenType::L_ANGLE) {
        Token * const elemType = state.nextToken();
        TOKEN_CHECK_FATAL(elemType, TokenType::INT_TYPE, "expected integer type (e.g., 'i8')");
        TOKEN_CHECK_FATAL(state.nextToken(), TokenType::R_ANGLE, "expected '>'");
        auto elemTy = new PabloKernelSignature::IntType(elemType->getValue());
        if (state.peekToken()->getType() == TokenType::L_SBRACE) {
            state.nextToken(); // consume '['
            Token * const streamCount = state.nextToken();
            TOKEN_CHECK_FATAL(streamCount, TokenType::INT_LITERAL, "expected an integer literal");
            TOKEN_CHECK_FATAL(state.nextToken(), TokenType::R_SBRACE, "expected a ']'");
            return new PabloKernelSignature::StreamSetType(elemTy, streamCount->getValue());
        } else {
            return new PabloKernelSignature::StreamType(elemTy);
        }
    } else {
        mErrorManager->logFatalError(t, "expected type");
        return nullptr;
    }
}


PabloAST * RecursiveParser::parseBlock(ParserState & state) {
    TOKEN_CHECK_FATAL(state.nextToken(), TokenType::L_BRACE, "expected a '{'");
    while (state.peekToken()->getType() != TokenType::R_BRACE && state.peekToken()->getType() != TokenType::EOF_TOKEN && mErrorManager->shouldContinue()) {
        auto stmt = parseStatement(state);
        if (stmt == nullptr) {
            return nullptr;
        }
    }
    if (!mErrorManager->shouldContinue()) {
        return nullptr;
    }
    TOKEN_CHECK_FATAL(state.nextToken(), TokenType::R_BRACE, "expected a '}'");
    return state.pb->getPabloBlock();
}


PabloAST * RecursiveParser::parseStatement(ParserState & state) {
    Token * const t = state.peekToken();
    if (t->getType() == TokenType::IF) {
        return parseIf(state);
    } else if (t->getType() == TokenType::WHILE) {
        return parseWhile(state);
    } else if (t->getType() == TokenType::IDENTIFIER) {
        return parseAssign(state);
    } else {
        mErrorManager->logFatalError(t, "expecting a statement (assignment, 'if', or 'while')");
        return nullptr;
    }
}


PabloAST * RecursiveParser::parseIf(ParserState & state) {
    assert (state.peekToken()->getType() == TokenType::IF);
    state.nextToken(); // consume 'if'
    PabloAST * const expr = parseExpression(state);
    if (expr == nullptr) {
        return nullptr;
    }

    PabloBuilder * previousScope = state.pb;
    // push new scope
    PabloBuilder newScope = state.pb->createScope();
    state.pb = &newScope;
    state.pushSymbolTable();

    PabloAST * const scopeBody = parseBlock(state);

    // pop scope
    state.popSymbolTable();
    state.pb = previousScope;

    if (scopeBody == nullptr) {
        return nullptr;
    }
    return llvm::cast<PabloAST>(state.pb->createIf(expr, llvm::cast<PabloBlock>(scopeBody)));
}


PabloAST * RecursiveParser::parseWhile(ParserState & state) {
    assert (state.peekToken()->getType() == TokenType::WHILE);
    state.nextToken(); // consume 'while'
    PabloAST * const expr = parseExpression(state);
    if (expr == nullptr) {
        return nullptr;
    }

    PabloBuilder * previousScope = state.pb;
    // push new scope
    PabloBuilder newScope = state.pb->createScope();
    state.pb = &newScope;
    state.pushSymbolTable();

    PabloAST * const scopeBody = parseBlock(state);

    // pop scope
    state.popSymbolTable();
    state.pb = previousScope;

    if (scopeBody == nullptr) {
        return nullptr;
    }
    return llvm::cast<PabloAST>(state.pb->createWhile(expr, llvm::cast<PabloBlock>(scopeBody)));
}


PabloAST * RecursiveParser::parseAssign(ParserState & state) {
    Token * const assignee = state.nextToken();
    assert (assignee->getType() == TokenType::IDENTIFIER);
    Token * index = nullptr;
    if (state.peekToken()->getType() == TokenType::L_SBRACE) {
        state.nextToken(); // consume '['
        index = state.nextToken();
        TOKEN_CHECK_FATAL(index, TokenType::INT_LITERAL, "expected integer literal");
        TOKEN_CHECK_FATAL(state.nextToken(), TokenType::R_SBRACE, "expected ']'");
    } else if (state.peekToken()->getType() == TokenType::DOT) {
        state.nextToken(); // consume '.'
        index = state.nextToken();
        TOKEN_CHECK_FATAL(index, TokenType::IDENTIFIER, "expected name");
    }

    Token * const op = state.nextToken();
    if (op->getType() != TokenType::ASSIGN && op->getType() != TokenType::MUTABLE_ASSIGN) {
        mErrorManager->logFatalError(op, errtxt_UnexpectedToken(op), "expected '=' or ':='");
        return nullptr;
    }

    PabloAST * const expr = parseExpression(state);
    if (expr == nullptr) {
        return nullptr;
    }
    if (op->getType() == TokenType::ASSIGN) {
        if (index == nullptr) {
            return state.symbolTable->assign(assignee, expr);
        } else {
            return state.symbolTable->indexedAssign(assignee, index, expr);
        }
    } else {
        if (index == nullptr) {
            assert (op->getType() == TokenType::MUTABLE_ASSIGN);
            return state.symbolTable->createVar(assignee, expr);
        } else {
            mErrorManager->logFatalError(op, "unnecessary mutable indexed assign", "use '=' instead");
            return nullptr;
        }
    }
}


PabloAST * RecursiveParser::parseExpression(ParserState & state) {
    PabloAST * const term = parseTerm(state);
    if (term == nullptr) {
        return nullptr;
    }
    return extendExpression(term, state);
}


PabloAST * RecursiveParser::extendExpression(PabloAST * lhs, ParserState & state) {
    Token * const t = state.peekToken();
    TokenType const type = t->getType();
    if (type == TokenType::BAR) {
        state.nextToken(); // consume '|'
        PabloAST * const term = parseTerm(state);
        if (term == nullptr) {
            return nullptr;
        }
        PabloAST * const expr = state.pb->createOr(lhs, term);
        return extendExpression(expr, state);
    } else if (type == TokenType::CARET) {
        state.nextToken(); // consume '^'
        PabloAST * const term = parseTerm(state);
        if (term == nullptr) {
            return nullptr;
        }
        PabloAST * const expr = state.pb->createXor(lhs, term);
        return extendExpression(expr, state);
    } else {
        return lhs;
    }
}


PabloAST * RecursiveParser::parseTerm(ParserState & state) {
    PabloAST * const factor = parseFactor(state);
    if (factor == nullptr) {
        return nullptr;
    }
    return extendTerm(factor, state);
}


PabloAST * RecursiveParser::extendTerm(PabloAST * lhs, ParserState & state) {
    Token * const t = state.peekToken();
    TokenType const type = t->getType();
    if (type == TokenType::AND) {
        state.nextToken(); // consume '&'
        PabloAST * const factor = parseFactor(state);
        if (factor == nullptr) {
            return nullptr;
        }
        PabloAST * const term = state.pb->createAnd(lhs, factor);
        return extendTerm(term, state);
    }
    return lhs;
}


PabloAST * RecursiveParser::parseFactor(ParserState & state) {
    TokenType peek = state.peekToken()->getType();
    if (peek == TokenType::TILDE) {
        state.nextToken(); // consume '~'
        PabloAST * const primitive = parsePrimitive(state);
        if (primitive == nullptr) {
            return nullptr;
        }
        return state.pb->createNot(primitive);
    } else {
        return parsePrimitive(state);
    }
}


PabloAST * RecursiveParser::parsePrimitive(ParserState & state) {
    Token * const first = state.peekToken();
    TokenType const firstType = first->getType();
    TokenType const secondType = state.peekAhead(2)->getType();
    if (firstType == TokenType::IDENTIFIER) {
        if (secondType == TokenType::L_PAREN) {
            return parseFunctionCall(state);
        } else {
            return parseVariable(state);
        }
    } else if (firstType == TokenType::INT_LITERAL || firstType == TokenType::L_ANGLE) {
        return parseLiteral(state);
    } else if (firstType == TokenType::L_PAREN) {
        state.nextToken(); // consume '('
        PabloAST * const expr = parseExpression(state);
        if (expr == nullptr) {
            return nullptr;
        }
        TOKEN_CHECK_FATAL(state.nextToken(), TokenType::R_PAREN, "expected ')'");
        return expr;
    } else {
        mErrorManager->logFatalError(first, errtxt_UnexpectedToken(first), "expected function call, variable, or literal");
        return nullptr;
    }
}


PabloAST * RecursiveParser::parseFunctionCall(ParserState & state) {
    Token * const identifier = state.nextToken();
    assert (identifier->getType() == TokenType::IDENTIFIER);
    TOKEN_CHECK_FATAL(state.nextToken(), TokenType::L_PAREN, "expected '('");
    std::vector<Token *> argTokens{};
    std::vector<PabloAST *> args{};
    if (state.peekToken()->getType() != TokenType::R_PAREN) {
        Token * argStartToken = state.peekToken();
        PabloAST * arg = parseExpression(state);
        if (arg == nullptr) {
            return nullptr;
        }
        Token * argEndToken = state.prevToken();
        argTokens.push_back(Token::CreateImaginaryField(argStartToken, argEndToken));
        args.push_back(arg);
        while (state.peekToken()->getType() == TokenType::COMMA) {
            state.nextToken(); // consume ','
            argStartToken = state.peekToken();
            arg = parseExpression(state);
            if (arg == nullptr) {
                return nullptr;
            }
            argEndToken = state.prevToken();
            argTokens.push_back(Token::CreateImaginaryField(argStartToken, argEndToken));
            args.push_back(arg);
        }
    }
    TOKEN_CHECK_FATAL(state.nextToken(), TokenType::R_PAREN, "expected ')'");
    return createFunctionCall(state, identifier, argTokens, args);
}


PabloAST * RecursiveParser::parseVariable(ParserState & state) {
    Token * const identifier = state.nextToken();
    assert (identifier->getType() == TokenType::IDENTIFIER);
    Token * const peekNext = state.peekToken();
    if (peekNext->getType() == TokenType::L_SBRACE || peekNext->getType() == TokenType::DOT) {
        Token * const op = state.nextToken();
        Token * const index = state.nextToken();
        if (op->getType() == TokenType::L_SBRACE) {
            TOKEN_CHECK_FATAL(state.nextToken(), TokenType::R_SBRACE, "expected ']'");
        }
        return state.symbolTable->indexedLookup(identifier, index);
    } else {
        return state.symbolTable->lookup(identifier);
    }
}


PabloAST * RecursiveParser::parseLiteral(ParserState & state) {
    Token * const peek = state.peekToken();
    TokenType peekType = peek->getType();
    if (peekType == TokenType::INT_LITERAL) {
        Token * integer = state.nextToken();
        return llvm::cast<PabloAST>(state.pb->getInteger((int64_t) integer->getValue()));
    } else if (peekType == TokenType::L_ANGLE) {
        state.nextToken(); // consume '<'
        Token * const i = state.nextToken();
        TOKEN_CHECK_FATAL(i, TokenType::INT_LITERAL, "expected integer literal");
        TOKEN_CHECK_FATAL(state.nextToken(), TokenType::R_ANGLE, "expected '>'");
        uint64_t val = i->getValue();
        switch (val) {
        case 0:
            return llvm::cast<PabloAST>(state.pb->createZeroes());
        case 1:
            return llvm::cast<PabloAST>(state.pb->createOnes());
        default:
            mErrorManager->logFatalError(i, "stream literals may only be '<0>' or '<1>'");
            return nullptr;
        }
    } else {
        mErrorManager->logFatalError(peek, errtxt_UnexpectedToken(peek), "expected scalar or stream literal");
        return nullptr;
    }
}


typedef PabloAST *(*FnGen)(ErrorManager * em, 
                           Token * funcToken, 
                           std::vector<Token *> const & argTokens, 
                           PabloBuilder *, 
                           std::vector<PabloAST *> const &);

static std::unique_ptr<llvm::StringMap<FnGen>> functionGenMap = nullptr;

#define FUNC_GEN_DEF(FUNCTION, BODY) \
functionGenMap->insert({FUNCTION, [](ErrorManager * em, Token * funcToken, std::vector<Token *> const & argTokens, PabloBuilder * pb, std::vector<PabloAST *> const & args) -> PabloAST * BODY})

#define ASSERT_ARG_NUM(COUNT) \
if (args.size() != COUNT) {\
    em->logFatalError(funcToken, errtxt_InvalidArgNum(funcToken->getText(), COUNT)); \
    return nullptr; \
}

#define ASSERT_ARG_TYPE_INT(ARG_IDX) \
if (!llvm::isa<Integer>(args[(ARG_IDX)])) { \
    em->logFatalError(argTokens[(ARG_IDX)], errtxt_InvalidArgType(), "expected 'integer'"); \
    return nullptr; \
}

static inline void lazyInitializeFunctionGenMap() {
    if (functionGenMap != nullptr) {
        return;
    } else {
        functionGenMap = std::unique_ptr<llvm::StringMap<FnGen>>(new llvm::StringMap<FnGen>{});
    }

    FUNC_GEN_DEF("advance", {
        ASSERT_ARG_NUM(2);
        ASSERT_ARG_TYPE_INT(1);
        return pb->createAdvance(args[0], llvm::cast<Integer>(args[1]));
    });

    FUNC_GEN_DEF("indexedAdvance", {
        ASSERT_ARG_NUM(3);
        ASSERT_ARG_TYPE_INT(1);
        return pb->createIndexedAdvance(args[0], args[1], llvm::cast<Integer>(args[3]));
    });

    FUNC_GEN_DEF("lookahead", {
        ASSERT_ARG_NUM(2);
        ASSERT_ARG_TYPE_INT(1);
        return pb->createLookahead(args[0], llvm::cast<Integer>(args[1]));
    });

    FUNC_GEN_DEF("repeat", {
        ASSERT_ARG_NUM(2);
        ASSERT_ARG_TYPE_INT(0);
        return pb->createRepeat(llvm::cast<Integer>(args[0]), args[1]);
    });
    
    FUNC_GEN_DEF("packL", {
        ASSERT_ARG_NUM(2);
        ASSERT_ARG_TYPE_INT(0);
        return pb->createPackL(llvm::cast<Integer>(args[0]), args[1]);
    });
    
    FUNC_GEN_DEF("packH", {
        ASSERT_ARG_NUM(2);
        ASSERT_ARG_TYPE_INT(0);
        return pb->createPackH(llvm::cast<Integer>(args[0]), args[1]);
    });

    FUNC_GEN_DEF("matchStar", {
        ASSERT_ARG_NUM(2);
        return pb->createMatchStar(args[0], args[1]);
    });

    FUNC_GEN_DEF("scanThru", {
        ASSERT_ARG_NUM(2);
        return pb->createScanThru(args[0], args[1]);
    });

    FUNC_GEN_DEF("scanTo", {
        ASSERT_ARG_NUM(2);
        return pb->createScanTo(args[0], args[1]);
    });


    FUNC_GEN_DEF("advanceThenScanThru", {
        ASSERT_ARG_NUM(2);
        return pb->createAdvanceThenScanThru(args[0], args[1]);
    });

    FUNC_GEN_DEF("advanceThenScanTo", {
        ASSERT_ARG_NUM(2);
        return pb->createAdvanceThenScanTo(args[0], args[1]);
    });

    FUNC_GEN_DEF("sel", {
        ASSERT_ARG_NUM(3);
        return pb->createSel(args[0], args[1], args[2]);
    });

    FUNC_GEN_DEF("count", {
        ASSERT_ARG_NUM(1);
        return pb->createCount(args[0]);
    });

    FUNC_GEN_DEF("inFile", {
        ASSERT_ARG_NUM(1);
        return pb->createInFile(args[0]);
    });

    FUNC_GEN_DEF("atEOF", {
        ASSERT_ARG_NUM(1);
        return pb->createAtEOF(args[0]);
    });
}


PabloAST * RecursiveParser::createFunctionCall(ParserState & state, Token * func, std::vector<Token *> const & argTokens, std::vector<PabloAST *> const & args) {
    assert (argTokens.size() == args.size());
    lazyInitializeFunctionGenMap();
    std::string id = func->getText();
    auto it = functionGenMap->find(id);
    if (it == functionGenMap->end()) {
        mErrorManager->logFatalError(func, errtxt_UndefinedFunction(id));
        return nullptr;
    }
    return (it->getValue())(mErrorManager.get(), func, argTokens, state.pb, args);
}


RecursiveParser::RecursiveParser(std::unique_ptr<Lexer> lexer, std::shared_ptr<ErrorManager> errorDelegate)
: PabloParser()
, mLexer(std::move(lexer))
, mErrorManager(std::move(errorDelegate))
, mTokenList()
, mCurrentSource(nullptr)
, mKernelLocations()
{}

} // namespace pablo::parse
} // namespace pablo
