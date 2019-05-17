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

    std::unique_ptr<std::vector<Token *>> tokenize(SourceFile & sourceFile) override;

    ErrorManager const & getErrorManager() const override {
        return mErrorManager;
    }

private:

    Token * extractText();
    Token * extractIntLiteral();
    Token * extractSymbol();

    ErrorManager        mErrorManager;
    SourceFile const *  mCurrentSource;
    boost::string_view  mCurrentLine;
    size_t              mCurrentLineNum;
    size_t              mCurrentColNum;
};

} // namespace pablo::parse
} // namespace pablo
