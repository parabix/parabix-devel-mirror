/*
 * Copyright (c) 2019 International Characters.
 * This software is licensed to the public under the Open Software License 3.0.
 */

/*
    This file implements the parser for the static stream notation used when
    constructing kernel unit tests.

    The grammar for the stream notation is a follows:

        stream          ::= { <expression> }
        expression      ::= <group> | <repetition> | <digit>
        group           ::= "(" { <expression> } ")"
        repetition      ::= ( <group> | <digit> ) "{" <number> "}"
        digit           ::= <hex-digit> | <binary-digit>
            - depending on the stream type, only hex-digit for hex streams and
              only binary-digit for binary streams
        hex-digit       ::= [0-9a-fA-F]
        binary-digit    ::= [10.]
        number          ::= [0-9]+
 */

#pragma once

#include <cinttypes>
#include <memory>
#include <vector>
#include <llvm/Support/Casting.h>
#include <llvm/Support/ErrorHandling.h>

namespace ssn {

namespace ast {

struct Node {
    enum class ClassTypeId {
        DIGIT, REP, GROUP
    };

    static bool classof(const Node *) { return true; }
    static bool classof(const void *) { return false; }

    Node(ClassTypeId id) : id(id) {}

    virtual ~Node() = 0;

    ClassTypeId getClassTypeId() const noexcept { return id; }
private:
    ClassTypeId id;
};

inline Node::~Node() { }

#define AST_NODE_TYPE(T) \
    static bool classof(const Node * t) { return t->getClassTypeId() == ClassTypeId::T; } \
    static bool classof(const void *) { return false; } \

struct Digit : public Node {
    AST_NODE_TYPE(DIGIT);
    explicit Digit(uint8_t value) : Node(ClassTypeId::DIGIT), value(value) {}
    ~Digit() override = default;
    const uint8_t value;
};

struct Rep : public Node {
    AST_NODE_TYPE(REP);
    explicit Rep(std::unique_ptr<Node> && value, uint32_t count)
    : Node(ClassTypeId::REP), value(std::move(value)), count(count)
    {}
    ~Rep() override = default;
    const std::unique_ptr<Node> value;
    const uint32_t              count;
};

struct Group : public Node {
    AST_NODE_TYPE(GROUP);
    explicit Group(std::vector<std::unique_ptr<Node>> && value)
    : Node(ClassTypeId::GROUP), value(std::move(value))
    {}    
    ~Group() override = default;
    const std::vector<std::unique_ptr<Node>> value;
};

/**
 * Walks a given AST `Node`, calling `fn` with each `Digit` value encountered.
 */
template<typename Fn>
inline void walk(const std::unique_ptr<Node> & n, Fn fn) {
    if (auto group = llvm::dyn_cast<Group>(n.get())) {
        for (auto const & e : group->value) {
            walk(e, fn);
        }
    } else if (auto rep = llvm::dyn_cast<Rep>(n.get())) {
        for (uint32_t i = 0; i < rep->count; ++i) {
            walk(rep->value, fn);
        }
    } else if (auto digit = llvm::dyn_cast<Digit>(n.get())) {
        fn(digit->value);
    } else {
        llvm::report_fatal_error("unexpected ssn::ast::Node type");
    }
}

} // namespace ssn::ast

namespace parsing {

struct Token {
    enum class ClassTypeId {
        VAL, CHAR, NUM
    };
    static bool classof(const Token *) { return true; }
    static bool classof(const void *) { return false; }

    Token(ClassTypeId id) : id(id) {}

    virtual ~Token()  = 0;

    ClassTypeId getClassTypeId() const noexcept { return id; }
private:
    ClassTypeId id;
};

inline Token::~Token() { }

#define TOKEN_CLASS_TYPE(T) \
    static bool classof(const Token * t) { return t->getClassTypeId() == ClassTypeId::T; } \
    static bool classof(const void *) { return false; }

struct Val : public Token {
    TOKEN_CLASS_TYPE(VAL)
    explicit Val(uint8_t v) : Token(ClassTypeId::VAL), value(v) {}
    ~Val() override = default;
    const uint8_t value;
};

struct Char : public Token {
    TOKEN_CLASS_TYPE(CHAR)
    explicit Char(char c) : Token(ClassTypeId::CHAR), value(c) {}
    ~Char() override = default;
    const char value;
};

struct Number : public Token {
    TOKEN_CLASS_TYPE(NUM)
    explicit Number(uint32_t n) : Token(ClassTypeId::NUM), value(n) {}
    ~Number() override = default;
    const uint32_t value;
};

inline bool isControlChar(char c) noexcept {
    for (auto x : "(){}") {
        if (x == c) {
            return true;
        }
    }
    return false;
}

constexpr uint8_t HexTable[] = {
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,   0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,   0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,   0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,   0x08, 0x09, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,

    0xff, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f, 0xff,   0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,   0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f, 0xff,   0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,   0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
};

struct HexConverter {
    static uint8_t convert(char c) {
        auto val = HexTable[(int) c];
        if (val == 0xff) {
            llvm::report_fatal_error(std::string("'").append(1, c).append("' is not a valid hex digit"));
        }
        return val;
    }
};

struct BinaryConverter {
    static uint8_t convert(char c) {
        uint8_t val;
        if (c == '1') {
            val = 1;
        } else if (c == '0' || c == '.') {
            val = 0;
        } else {
            llvm::report_fatal_error(std::string("'").append(1, c).append("' is not a valid binary digit"));
        }
        return val;
    }
};

/**
 * `Tokenizer` wraps a C-style string allowing for sequential emission of
 * tokens from said string.
 * 
 * Template Arguments:
 *  - `Converter`: either `HexConverter` or `BinaryConveter`, used to convert
 *    non control characters to values.
 */
template<class Converter>
class Tokenizer {
public:
    Tokenizer() = delete;
    Tokenizer(Tokenizer const &) = delete;
    Tokenizer(Tokenizer &&) = default;
    explicit Tokenizer(const char * text)
    : mText(text), mIdx(0)
    {}

    /**
     * Tells this `Tokenizer` that the next call to `emit` should produce a 
     * `Number` token. The effect only lasts for a single call to `emit` as
     * the state will revert back to normal once the `Number` token has been
     * emitted.
     */
    void expectNumber() {
        mBuildNumber = true;
    }

    bool isFinished() noexcept {
        return mText[mIdx] == '\0';
    }

    const std::unique_ptr<Token> & peek() {
        mBuffer = emit();
        return mBuffer;
    }

    std::unique_ptr<Token> emit() {
        if (mBuffer != nullptr) {
            auto t = std::move(mBuffer);
            mBuffer = nullptr;
            return t;
        }
        // skip over any whitespace
        while (mText[mIdx] != '\0' && std::isspace(mText[mIdx])) {
            mIdx++;
        }
        if (mBuildNumber) {
            return emitNumber();
        }
        if (mText[mIdx] == '\0') {
            return nullptr;
        }
        auto c = mText[mIdx];
        mIdx++;
        if (isControlChar(c)) {
            return std::unique_ptr<Token>(new Char{c});
        } else {
            uint8_t val = Converter::convert(c);
            return std::unique_ptr<Token>(new Val{val});
        }
    }

private:
    std::unique_ptr<Token> emitNumber() {
        if (mText[mIdx] == '\0') {
            llvm::report_fatal_error("expected number, found EOF");
        }
        const char * begin = mText + mIdx;
        while (std::isdigit(mText[mIdx])) {
            mIdx++;
        }
        const char * end = mText + mIdx;
        if (begin == end) {
            llvm::report_fatal_error("expected a number, but did not find one");
        }
        // safe to cast from int to uint32_t as we aren't accepting '-'
        // as part of a number.
        auto x = (uint32_t) std::stoi(std::string(begin, end));
        mBuildNumber = false;
        return std::unique_ptr<Token>(new Number{x});
    }

    const char *           mText;
    size_t                 mIdx;
    bool                   mBuildNumber = false;
    std::unique_ptr<Token> mBuffer = nullptr;
};


/**
 * `Parser` converts a C-style string to an abstract syntax tree.
 * 
 * Template Arguments:
 *  - `Converter`: either `HexConverter` or `BinaryConverter`, passed to the
 *    parser's internal `Tokenizer`.
 */
template<class Converter>
class Parser {
public:
    explicit Parser(const char * text)
    : mTokenizer(text), mNodeList()
    {}

    std::unique_ptr<ast::Node> parse() {
        while (!mTokenizer.isFinished()) {
            parseExpression(mNodeList);
        }
        return std::unique_ptr<ast::Node>(new ast::Group(std::move(mNodeList)));
    }

private:
    void assertNotEOF(std::unique_ptr<Token> const & t) {
        if (t == nullptr) {
            llvm::report_fatal_error("stream parse error: unexpected EOF");
        }
    }

    void assertChar(std::unique_ptr<Token> const & t, char c) {
        assertNotEOF(t);
        if (auto ch = llvm::dyn_cast<Char>(t.get())) {
            if (ch->value != c) {
                llvm::report_fatal_error("stream parse error: unexpected token - expected '" + std::string(1, c) + "'");
            }
        } else {
            llvm::report_fatal_error("stream parse error: unexpected token - expected '" + std::string(1, c) + "'");
        }
    }

    void parseExpression(std::vector<std::unique_ptr<ast::Node>> & accumList) {
        auto token = mTokenizer.emit();
        assertNotEOF(token);
        if (llvm::isa<Char>(token)) {
            auto ch = llvm::cast<Char>(std::move(token));
            switch (ch->value) {
            case '(':
                parseGroup(accumList);
                break;
            case '{':
                parseRep(accumList);
                break;
            default:
                llvm::report_fatal_error("stream parse error: unexpected token '" + std::string(1, ch->value) + "'");
            }
        } else if (llvm::isa<Val>(token)) {
            auto val = llvm::cast<Val>(std::move(token));
            accumList.emplace_back(new ast::Digit(val->value));
        } else {
            llvm::report_fatal_error("stream parse error: unexpected token type");
        }
    }

    void parseGroup(std::vector<std::unique_ptr<ast::Node>> & accumList) {
        auto newList = std::vector<std::unique_ptr<ast::Node>>{};
        while (true) {
            parseExpression(newList);
            if (auto c = llvm::dyn_cast<Char>(mTokenizer.peek().get())) {
                if (c->value == ')') {
                    mTokenizer.emit(); // consume ')'
                    break;
                }
            }
        }
        accumList.emplace_back(new ast::Group{std::move(newList)});
    }

    void parseRep(std::vector<std::unique_ptr<ast::Node>> & accumList) {
        mTokenizer.expectNumber();
        auto token = mTokenizer.emit();
        auto num = llvm::unique_dyn_cast<Number>(token);
        auto closer = mTokenizer.emit();
        assertChar(closer, '}');
        if (accumList.size() == 0) {
            llvm::report_fatal_error("stream parse error: `{...}` may not be the first item in an expression");
        }
        auto prev = std::move(accumList.back());
        accumList.pop_back();
        accumList.emplace_back(new ast::Rep(std::move(prev), num->value));
    }

    Tokenizer<Converter>                    mTokenizer;
    std::vector<std::unique_ptr<ast::Node>> mNodeList;
};

} // namespace ssn::parsing

using parsing::HexConverter;
using parsing::BinaryConverter;

template<class Converter>
inline std::unique_ptr<ast::Node> parse(const char * text) {
    auto parser = parsing::Parser<Converter>(text);
    return parser.parse();
}

} // namespace ssn
