/*
 *  Copyright (c) 2019 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 */

#pragma once

#include <vector>
#include <boost/make_unique.hpp>
#include <boost/optional.hpp>
#include <pablo/parse/error.h>

namespace pablo {
namespace parse {

/**
 * A simple byte-at-a-time, extraction based pablo lexer.
 *
 * Errors are manged by an external ErrorManager delegate. In the event of a
 * lexical error, use the external delegate to gain access to the generated
 * errors.
 */
class Lexer {
public:

    static std::unique_ptr<Lexer> Create(std::shared_ptr<ErrorManager> errorDelegate) {
        return boost::make_unique<Lexer>(std::move(errorDelegate));
    }

public:

    Lexer() = delete;

    /**
     * Constructs this lexer instance with a shared reference to an error
     * manager. The reference is shared so that other components may utilize
     * the same error manager instance.
     *
     * @param errorDelegate A shared pointer to an ErrorManager instance.
     */
    explicit Lexer(std::shared_ptr<ErrorManager> errorDelegate);

    /**
     * Converts an input stream of characters into a sequence of tokens. In the
     * event of a lexical error, boost::none is returned.
     *
     * Generated errors are stored and managed via the external ErrorManager.
     *
     * @param sourceFile A shared instance of the source file to tokenize.
     * @return A sequence of tokens or boost::none if a lexical error occurred.
     */
    boost::optional<std::vector<Token *>> tokenize(std::shared_ptr<SourceFile> sourceFile);

private:

    Token * extractText();
    Token * extractIntLiteral();
    Token * extractSymbol();
    Token * extractAttribute();

    std::shared_ptr<ErrorManager>   mErrorManager;
    std::shared_ptr<SourceFile>     mCurrentSource;
    boost::string_view              mCurrentLine;
    size_t                          mCurrentLineNum;
    size_t                          mCurrentColNum;
};

} // namespace pablo::parse
} // namespace pablo
