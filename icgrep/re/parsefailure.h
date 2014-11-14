/*
 *  Copyright (c) 2014 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#ifndef PARSEFAILURE_H
#define PARSEFAILURE_H

#include <string>
#include <stdexcept>

class ParseFailure : public std::exception {
public:
    ParseFailure(const std::string && msg) noexcept;
    virtual ~ParseFailure() noexcept;
    virtual const char* what() const noexcept;
private:
    inline ParseFailure() noexcept {}
    const std::string _msg;
};

class NoRegularExpressionFound : public ParseFailure {
public:
    NoRegularExpressionFound() noexcept : ParseFailure("No regular expression found!") { }
    virtual ~NoRegularExpressionFound() noexcept {}
};

class IncompleteRegularExpression : public ParseFailure {
public:
    IncompleteRegularExpression() noexcept : ParseFailure("Incomplete regular expression!") { }
    virtual ~IncompleteRegularExpression() noexcept {}
};

class UnclosedCharacterClass : public ParseFailure {
public:
    UnclosedCharacterClass() noexcept : ParseFailure("Unclosed character class!") { }
    virtual ~UnclosedCharacterClass() noexcept {}
};

class InvalidUTF8Encoding : public ParseFailure {
public:
    InvalidUTF8Encoding() noexcept : ParseFailure("Invalid UTF-8 encoding!") { }
    virtual ~InvalidUTF8Encoding() noexcept {}
};

class UnclosedUnicodeCharacterClass : public ParseFailure {
public:
    UnclosedUnicodeCharacterClass() noexcept : ParseFailure("Unclosed Unicode character class!") { }
    virtual ~UnclosedUnicodeCharacterClass() noexcept {}
};

class BadLowerBound : public ParseFailure {
public:
    BadLowerBound() noexcept : ParseFailure("Bad lower bound!") { }
    virtual ~BadLowerBound() noexcept {}
};

class BadUpperBound : public ParseFailure {
public:
    BadUpperBound() noexcept : ParseFailure("Bad upper bound!") { }
    virtual ~BadUpperBound() noexcept {}
};

class NothingToRepeat : public ParseFailure {
public:
    NothingToRepeat() noexcept : ParseFailure("Need something to repeat before *, +, ? or {.") { }
    virtual ~NothingToRepeat() noexcept {}
};

#endif // PARSEFAILURE_H
