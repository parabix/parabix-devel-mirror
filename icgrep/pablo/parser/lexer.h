/*
 *  Copyright (c) 2019 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#pragma once

#include <memory>
#include <vector>
#include <pablo/parser/error.h>
#include <pablo/parser/token.h>
#include <pablo/parser/source_file.h>

namespace pablo {
namespace parse {

class Lexer {
public:

    virtual ~Lexer() = default;

    /**
     * Converts an input stream of characters into a sequence of tokens. In the
     * event of a lexical error, nullptr is returned and access to the error
     * information can be gotten through the getErrorManager method.
     * 
     * @param in the input stream of characters to tokenize
     * @return a sequence of tokens
     */
    virtual std::unique_ptr<std::vector<Token *>> tokenize(SourceFile & sourceFile) = 0;

    virtual ErrorManager const & getErrorManager() const = 0;

};

} // namespace pablo::parse
} // namespace pablo
