/*
 *  Copyright (c) 2019 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#include "rd_parser.h"

#include <unordered_set>
#include <kernels/core/streamset.h>
#include <llvm/Support/Casting.h>
#include <pablo/builder.hpp>
#include <pablo/branch.h>
#include <pablo/pablo_intrinsic.h>
#include <pablo/pe_ones.h>
#include <pablo/pe_zeroes.h>
#include <pablo/parser/error_text.h>
#include <pablo/parser/pablo_type.h>
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
    assert (sourceData->tokenList[index]);
    Token * t = sourceData->tokenList[index];
    if (t->getType() != TokenType::EOF_TOKEN) {
        assert (index < sourceData->tokenList.size() - 1);
        index++;
    }
    return t;
}


Token * RecursiveParser::ParserState::peekToken() {
    assert (sourceData->tokenList[index]);
    return sourceData->tokenList[index];
}


Token * RecursiveParser::ParserState::peekAhead(size_t n) {
    assert (sourceData->tokenList[index]);
    // n - 1 as we are already pointing at the next token
    // peekAhead(1) is equivalent to peekToken()
    if (index + n - 1 >= sourceData->tokenList.size()) {
        return sourceData->tokenList.back();
    }
    return sourceData->tokenList[index + n - 1];
}


Token * RecursiveParser::ParserState::prevToken() {
    assert (sourceData->tokenList[index]);
    if (index > 0) {
        assert (sourceData->tokenList[index - 1]);
        return sourceData->tokenList[index - 1];
    }
    return sourceData->tokenList[index];
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


RecursiveParser::ParserState::ParserState(RecursiveParser * parser, size_t index, SourceData * sourceData)
: parser(parser)
, pb(nullptr)
, kernel(nullptr)
, index(index)
, symbolTable(nullptr)
, sourceData(sourceData)
{}


RecursiveParser::ParserState::ParserState(RecursiveParser * parser, PabloBuilder * pb, PabloSourceKernel * kernel, SourceData * sourceData)
: parser(parser)
, pb(pb)
, kernel(kernel)
, index(0)
, symbolTable(nullptr)
, sourceData(sourceData)
{
    pushSymbolTable();
}

RecursiveParser::ParserState::~ParserState() {
    if (symbolTable) {
        popSymbolTable();
    }
    assert (symbolTable == nullptr);
}


bool RecursiveParser::parseKernel(std::shared_ptr<SourceFile> sourceFile, PabloSourceKernel * kernel, std::string const & kernelName) {
    assert (kernel);
    assert (!kernelName.empty());

    // Construct source data for the source file or get existing source data if
    // we've already processed this file before.
    SourceData * data = nullptr;
    auto it = mSources.find(sourceFile.get());
    if (it == mSources.end()) {
        auto optList = mLexer->tokenize(sourceFile);
        if (!optList)
            return false;
        data = generateSourceDate(sourceFile, *optList);
        if (data == nullptr)
            return false;
        mSources.insert({sourceFile.get(), data});
    } else {
        data = it->second;
    }

    // Construct parser state object.
    PabloBuilder pb(kernel->getEntryScope());
    ParserState state(this, &pb, kernel, data);

    // Find location of the kernel to parse.
    auto kernelLocIt = data->kernelLocations.find(kernelName);
    if (kernelLocIt == data->kernelLocations.end()) {
        mErrorManager->logTextError(sourceFile, "No defintion for '" + kernelName + "' found.");
        return false;
    }
    state.index = kernelLocIt->getValue();

    // Parse kernel.
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


RecursiveParser::SourceData * RecursiveParser::generateSourceDate(std::shared_ptr<SourceFile> file, std::vector<Token *> const & tokenList) {
    llvm::StringMap<size_t> kernelLocations{};
    llvm::StringMap<PabloType *> typeDefTable{};
    for (size_t i = 0, j = 1; j < tokenList.size(); i++, j++) {
        Token * const keyword = tokenList[i];
        Token * const identifier = tokenList[j];

        if (keyword->getType() == TokenType::TYPE) {
            SourceData tmpSourceData{file, tokenList, {}, {}};
            ParserState state(this, i, &tmpSourceData);
            PabloType * const definedType = parseTypeDefinition(state);
            if (definedType == nullptr) {
                return nullptr;
            }
            assert (identifier->getType() == TokenType::IDENTIFIER);
            auto itOk = typeDefTable.insert({identifier->getText(), definedType});
            if (!itOk.second) {
                mErrorManager->logFatalError(identifier, errtxt_DuplicateTypeDefinition(identifier->getText()));
                return nullptr;
            }
        }

        if (keyword->getType() == TokenType::KERNEL && identifier->getType() == TokenType::IDENTIFIER) {
            auto rt = kernelLocations.try_emplace(identifier->getText(), i);
            if (!rt.second) {
                size_t prev = kernelLocations.lookup(identifier->getText());
                Token * p = tokenList[prev];
                mErrorManager->logFatalError(identifier, "duplicate kernel declaration");
                mErrorManager->logNote(p, errtxt_PreviousDefinitionNote());
            }
        }
    }
    return new SourceData{std::move(file), tokenList, std::move(kernelLocations), std::move(typeDefTable)};
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


PabloType * RecursiveParser::parseSigType(ParserState & state) {
    Token * const t = state.nextToken();
    TokenType const type = t->getType();
    if (type == TokenType::IDENTIFIER) {
        auto it = state.sourceData->typeDefTable.find(t->getText());
        if (it == state.sourceData->typeDefTable.end()) {
            mErrorManager->logFatalError(t, errtxt_UndefinedTypename(t->getText()));
            return nullptr;
        }
        return it->getValue();
    } else if (type == TokenType::INT_TYPE) {
        return new ScalarType(t->getValue());
    } else if (type == TokenType::L_ANGLE) {
        Token * const elemType = state.nextToken();
        TOKEN_CHECK_FATAL(elemType, TokenType::INT_TYPE, "expected integer type (e.g., 'i8')");
        TOKEN_CHECK_FATAL(state.nextToken(), TokenType::R_ANGLE, "expected '>'");
        if (state.peekToken()->getType() != TokenType::L_SBRACE) {
            return new StreamType(elemType->getValue());
        }

        // dealing with a streamset type
        state.nextToken(); // consume '['
        Token * const streamCount = state.nextToken();
        TOKEN_CHECK_FATAL(streamCount, TokenType::INT_LITERAL, "expected an integer literal");
        TOKEN_CHECK_FATAL(state.nextToken(), TokenType::R_SBRACE, "expected a ']'");
        StreamSetType * const streamSetType = new StreamSetType(elemType->getValue(), streamCount->getValue());
        if (state.peekToken()->getType() != TokenType::L_BRACE) {
            return streamSetType;
        }

        // dealing with a streamset with named streams
        state.nextToken(); // consume '{'
        std::vector<Token *> nameList{};
        std::unordered_set<Token *> nameSet{};
        Token * identifier = state.nextToken();
        TOKEN_CHECK_FATAL(identifier, TokenType::IDENTIFIER, "expected a name");
        nameSet.insert(identifier);
        nameList.push_back(identifier);
        while (state.peekToken()->getType() == TokenType::COMMA) {
            state.nextToken(); // consume ','
            identifier = state.nextToken();
            std::unordered_set<Token *>::iterator it;
            bool ok;
            std::tie(it, ok) = nameSet.insert(identifier);
            if (!ok) {
                mErrorManager->logError(identifier, errtxt_RedeclaredStreamName());
                mErrorManager->logNote(*it, errtxt_PreviousDefinitionNote());
                return nullptr;
            }
            nameList.push_back(identifier);
        }
        TOKEN_CHECK_FATAL(state.nextToken(), TokenType::R_BRACE, "expected '}'");
        std::vector<std::string> names{};
        for (auto const & t : nameList) {
            names.push_back(t->getText());
        }
        return new NamedStreamSetType(PabloType::GenerateAnonymousTypeName(), streamSetType, names);
    } else {
        mErrorManager->logFatalError(t, "expected type");
        return nullptr;
    }
}


PabloType * RecursiveParser::parseTypeDefinition(ParserState & state) {
    assert (state.peekToken()->getType() == TokenType::TYPE);
    state.nextToken(); // consume 'type'
    Token * const identifier = state.nextToken();
    TOKEN_CHECK_FATAL(identifier, TokenType::IDENTIFIER, "expected a typename");
    TOKEN_CHECK_FATAL(state.nextToken(), TokenType::ASSIGN, "expected '='");
    PabloType * type = parseSigType(state);
    if (type == nullptr) {
        return nullptr;
    }
    if (llvm::isa<NamedStreamSetType>(type)) {
        llvm::cast<NamedStreamSetType>(type)->setTypeName(identifier->getText());
        auto names = llvm::cast<NamedStreamSetType>(type)->getStreamNames();
    } else {
        type = new AliasType(identifier->getText(), type);
    }
    // make sure there isn't any bad tokens after the definition
    if (state.peekToken()->getType() != TokenType::TYPE && state.peekToken()->getType() != TokenType::KERNEL) {
        mErrorManager->logFatalError(state.peekToken(), "unexpected top level declaration", "expected 'type' or 'kernel'");
        return nullptr;
    }
    return type;
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

    TOKEN_CHECK_FATAL(state.nextToken(), TokenType::ASSIGN, "expected '='");
    PabloAST * const expr = parseExpression(state);
    if (expr == nullptr) {
        return nullptr;
    }
    if (index == nullptr) {
        return state.symbolTable->assign(assignee, expr);
    } else {
        return state.symbolTable->indexedAssign(assignee, index, expr);
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

    FUNC_GEN_DEF("Advance", {
        ASSERT_ARG_NUM(2);
        ASSERT_ARG_TYPE_INT(1);
        return pb->createAdvance(args[0], llvm::cast<Integer>(args[1]));
    });

    FUNC_GEN_DEF("IndexedAdvance", {
        ASSERT_ARG_NUM(3);
        ASSERT_ARG_TYPE_INT(1);
        return pb->createIndexedAdvance(args[0], args[1], llvm::cast<Integer>(args[2]));
    });

    FUNC_GEN_DEF("Lookahead", {
        ASSERT_ARG_NUM(2);
        ASSERT_ARG_TYPE_INT(1);
        return pb->createLookahead(args[0], llvm::cast<Integer>(args[1]));
    });

    FUNC_GEN_DEF("Repeat", {
        ASSERT_ARG_NUM(2);
        ASSERT_ARG_TYPE_INT(0);
        return pb->createRepeat(llvm::cast<Integer>(args[0]), args[1]);
    });

    FUNC_GEN_DEF("PackL", {
        ASSERT_ARG_NUM(2);
        ASSERT_ARG_TYPE_INT(0);
        return pb->createPackL(llvm::cast<Integer>(args[0]), args[1]);
    });

    FUNC_GEN_DEF("PackH", {
        ASSERT_ARG_NUM(2);
        ASSERT_ARG_TYPE_INT(0);
        return pb->createPackH(llvm::cast<Integer>(args[0]), args[1]);
    });

    FUNC_GEN_DEF("MatchStar", {
        ASSERT_ARG_NUM(2);
        return pb->createMatchStar(args[0], args[1]);
    });

    FUNC_GEN_DEF("ScanThru", {
        ASSERT_ARG_NUM(2);
        return pb->createScanThru(args[0], args[1]);
    });

    FUNC_GEN_DEF("ScanTo", {
        ASSERT_ARG_NUM(2);
        return pb->createScanTo(args[0], args[1]);
    });


    FUNC_GEN_DEF("AdvanceThenScanThru", {
        ASSERT_ARG_NUM(2);
        return pb->createAdvanceThenScanThru(args[0], args[1]);
    });

    FUNC_GEN_DEF("AdvanceThenScanTo", {
        ASSERT_ARG_NUM(2);
        return pb->createAdvanceThenScanTo(args[0], args[1]);
    });

    FUNC_GEN_DEF("Sel", {
        ASSERT_ARG_NUM(3);
        return pb->createSel(args[0], args[1], args[2]);
    });

    FUNC_GEN_DEF("Count", {
        ASSERT_ARG_NUM(1);
        return pb->createCount(args[0]);
    });

    FUNC_GEN_DEF("InFile", {
        ASSERT_ARG_NUM(1);
        return pb->createInFile(args[0]);
    });

    FUNC_GEN_DEF("AtEOF", {
        ASSERT_ARG_NUM(1);
        return pb->createAtEOF(args[0]);
    });

    FUNC_GEN_DEF("SpanUpTo", {
        ASSERT_ARG_NUM(2);
        return pb->createIntrinsicCall(Intrinsic::SpanUpTo, args);
    });

    FUNC_GEN_DEF("InclusiveSpan", {
        ASSERT_ARG_NUM(2);
        return pb->createIntrinsicCall(Intrinsic::InclusiveSpan, args);
    });

    FUNC_GEN_DEF("ExclusiveSpan", {
        ASSERT_ARG_NUM(2);
        return pb->createIntrinsicCall(Intrinsic::ExclusiveSpan, args);
    });

    FUNC_GEN_DEF("PrintRegister", {
        ASSERT_ARG_NUM(1);
        return pb->createIntrinsicCall(Intrinsic::PrintRegister, args);
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
, mSources()
{}


RecursiveParser::~RecursiveParser() {
    for (auto const & kv : mSources) {
        delete kv.second;
    }
}

} // namespace pablo::parse
} // namespace pablo
