/*
 *  Copyright (c) 2019 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#pragma once

#include <vector>
#include <pablo/parser/error.h>
#include <pablo/parser/lexer.h>

namespace pablo {
namespace parse {

class SimpleLexer final : public Lexer {
public:

    explicit SimpleLexer(ErrorContext const & errorContext);

    std::unique_ptr<std::vector<Token *>> tokenize(std::istream & in) override;

    void setFilename(std::string const & filename) override;

    ErrorManager const & getErrorManager() const override {
        return mErrorManager;
    }

private:

    Token * extractText();
    Token * extractIntLiteral();
    Token * extractSymbol();

    ErrorManager    mErrorManager;
    std::string     mCurrentFilename;
    std::string     mCurrentLine;
    size_t          mCurrentLineNum;
    size_t          mCurrentColNum;
};

} // namespace pablo::parse
} // namespace pablo
