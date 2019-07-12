/*
 *  Copyright (c) 2019 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#pragma once

#include <string>
#include <pablo/parse/token.h>

namespace pablo {
namespace parse {

inline std::string errtxt_IllegalSymbol(char c) {
    return "unexpected symbol '" + std::string{c} + "'";
}

inline std::string errtxt_IllegalSymbol(std::string const & s) {
    return "unexpected symbol '" + s + "'";
}

inline std::string errtxt_IllegalIntegerLiteral() {
    return "illegal integer literal";
}

inline std::string errtxt_UnexpectedToken(Token const * t) {
    if (t->getType() != TokenType::EOF_TOKEN) {
        return "unexpected token '" + t->getText() + "'";
    } else {
        return "unexpected end-of-file";
    }
}

inline std::string errtxt_RedefinitionError(std::string const & name) {
    return "redefinition of '" + name + "'";
}

inline std::string errtxt_RedefineIOVarError() {
    return "redefinition of an I/O variable";
}

inline std::string errtxt_HidesDeclaration(std::string const & name) {
    return "defintion of '" + name + "' hides a previous declaration";
}

inline std::string errtxt_PreviousDefinitionNote() {
    return "previous definition is here";
}

inline std::string errtxt_DefinitionNote(std::string const & name) {
    return "'" + name + "' is defined here";
}

inline std::string errtxt_UseOfUndefinedSymbol(std::string const & name) {
    return "use of undefined symbol '" + name + "'";
}

inline std::string errtxt_NonIndexableSymbol(std::string const & name) {
    return "'" + name + "' is of a non-indexable type";
}

inline std::string errtxt_InvalidArgNum(std::string const & funcName, size_t expectedNum) {
    return "invalid number of arguments for '" + funcName + "', expected " + std::to_string(expectedNum);
}

inline std::string errtxt_InvalidArgType() {
    return "invalid argument type";
}

inline std::string errtxt_UndefinedFunction(std::string const & name) {
    return "undefined function '" + name + "'";
}

inline std::string errtxt_RedeclaredStreamName() {
    return "redeclaration of stream name within this type";
}

inline std::string errtxt_DuplicateTypeDefinition(std::string const & name) {
    return "duplicate definition of type '" + name + "'";
}

inline std::string errtxt_UndefinedTypename(std::string const & name) {
    return "undefined typename or alias '" + name + "'";
}

inline std::string errtxt_VarNotDotIndexable(std::string const & name) {
    return "'" + name + "' is not indexable via '.' notation";
}

inline std::string errtxt_InvalidStreamName(std::string const & name, std::string const & streamName) {
    return "'" + name + "' does not contain a stream named: '" + streamName + "'";
}

} // namespace pablo::parse
} // namespace pablo
