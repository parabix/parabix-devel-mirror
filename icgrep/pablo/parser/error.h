/*
 *  Copyright (c) 2019 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#pragma once

#include <ostream>
#include <memory>
#include <string>
#include <vector>

namespace pablo {
namespace parse {

enum class ErrorType {
    WARNING,
    ERROR
};


class Error {
public:
    friend class ErrorManager;

    static std::unique_ptr<Error> CreateError(std::string const & text,
                                              std::string const & filename,
                                              std::string const & line,
                                              size_t lineNum, size_t colNum,
                                              std::string const & hint = "");

    static std::unique_ptr<Error> CreateWarning(std::string const & text,
                                                std::string const & filename,
                                                std::string const & line,
                                                size_t lineNum, size_t colNum,
                                                std::string const & hint = "");

private:
    Error(ErrorType type,
          std::string const & text,
          std::string const & filename,
          std::string const & line,
          size_t lineNum, size_t colNum,
          std::string const & hint);

    ErrorType   mType;
    std::string mText;
    std::string mFilename;
    std::string mLine;
    size_t      mLineNum;
    size_t      mColNum;
    std::string mHint;
};


struct ErrorContext {
    ErrorContext();

    bool canUseColor() const;

    std::ostream &  outStream;
    bool            useWarnings;
    bool            useFatalErrors;
    bool            useLivePrint;
    size_t          maxErrorNum;
};


class ErrorManager {
public:
    ErrorManager() = delete;
    explicit ErrorManager(ErrorContext const & context);

    void logError(std::string const & text, std::string const & hint = "");
    void logWarning(std::string const & text, std::string const & hint = "");
    void logError(std::string const & text, std::string const & line, size_t lineNum, size_t colNum, std::string const & hint = "");
    void logWarning(std::string const & text, std::string const & line, size_t lineNum, size_t colNum, std::string const & hint = "");

    bool canContinue() const {
        return mCanContinue;
    }

    bool hasErrors() const {
        return mErrorCount != 0;
    }

    void dumpErrors() const;

    void setReferences(std::string const * lineRef, size_t const * lineNumRef, size_t const * colNumRef);

    void setFilename(std::string const & filename) {
        mFilename = filename;
    }

private:
    std::string renderError(std::unique_ptr<Error> const & e) const;

    ErrorContext                        mContext;
    std::vector<std::unique_ptr<Error>> mErrorList;
    std::string                         mFilename;
    size_t                              mErrorCount;
    bool                                mCanContinue;
    std::string const *                 mLineRef;
    size_t const *                      mLineNumRef;
    size_t const *                      mColNumRef;
};

} // namespace pablo::parse
} // namespace pablo
