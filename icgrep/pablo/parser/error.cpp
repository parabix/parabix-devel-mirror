/*
 *  Copyright (c) 2019 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#include "error.h"

#include <cassert>
#include <iostream>

namespace pablo {
namespace parse {

std::unique_ptr<Error> Error::CreateError(std::string const & text,
                                          std::string const & filename,
                                          std::string const & line,
                                          size_t lineNum, size_t colNum,
                                          std::string const & hint)
{
    return std::unique_ptr<Error>(new Error(ErrorType::ERROR, text, filename, line, lineNum, colNum, hint));
}

std::unique_ptr<Error> Error::CreateWarning(std::string const & text,
                                            std::string const & filename,
                                            std::string const & line,
                                            size_t lineNum, size_t colNum,
                                            std::string const & hint)
{
    return std::unique_ptr<Error>(new Error(ErrorType::WARNING, text, filename, line, lineNum, colNum, hint));
}

Error::Error(ErrorType type,
             std::string const & text,
             std::string const & filename,
             std::string const & line,
             size_t lineNum, size_t colNum,
             std::string const & hint)
: mType(type)
, mText(text)
, mFilename(filename)
, mLine(line)
, mLineNum(lineNum)
, mColNum(colNum)
, mHint(hint)
{}


bool ErrorContext::canUseColor() const {
#ifdef WIN32
    return false;
#else 
    return &outStream == &std::cout || &outStream == &std::cerr;
#endif
}

ErrorContext::ErrorContext()
: outStream(std::cerr)
, useWarnings(true)
, useFatalErrors(false)
, useLivePrint(true)
, maxErrorNum(20)
{}


void ErrorManager::logError(std::string const & text, std::string const & hint) {
    assert ("location references not set" && mLineNumRef && mColNumRef);

    return logError(text, *mLineRef, *mLineNumRef, *mColNumRef, hint);
}

void ErrorManager::logWarning(std::string const & text, std::string const & hint) {
    assert ("location references not set" && mLineNumRef && mColNumRef);

    return logWarning(text, *mLineRef, *mLineNumRef, *mColNumRef, hint);
}

void ErrorManager::logError(std::string const & text, std::string const & line, size_t lineNum, size_t colNum, std::string const & hint) {
    assert (!mFilename.empty());
    auto e = Error::CreateError(text, mFilename, line, lineNum, colNum, hint);
    if (mContext.useLivePrint) {
        mContext.outStream << renderError(e);
    }
    mErrorCount++;
    if (mContext.useFatalErrors || mErrorCount >= mContext.maxErrorNum) {
        mCanContinue = false;
    }
    mErrorList.push_back(std::move(e));
}

void ErrorManager::logWarning(std::string const & text, std::string const & line, size_t lineNum, size_t colNum, std::string const & hint) {
    assert (!mFilename.empty());
    auto e = Error::CreateWarning(text, mFilename, line, lineNum, colNum, hint);
    if (mContext.useLivePrint) {
        mContext.outStream << renderError(e);
    }
    mErrorList.push_back(std::move(e));
}

void ErrorManager::dumpErrors() const {
    if (!mContext.useLivePrint) {
        for (auto const & e : mErrorList) {
            mContext.outStream << renderError(e);
        }
    }
    if (mErrorCount != 0) {
        mContext.outStream << "Found " << std::to_string(mErrorCount) << " error" << (mErrorCount >= 2 ? "s." : ".") << "\n";
    }
}

void ErrorManager::setReferences(std::string const * lineRef, size_t const * lineNumRef, size_t const * colNumRef) {
    assert (lineRef && lineNumRef && colNumRef);
    mLineRef = lineRef;
    mLineNumRef = lineNumRef;
    mColNumRef = colNumRef;
}

std::string ErrorManager::renderError(std::unique_ptr<Error> const & e) const {
    const std::string RED = mContext.canUseColor() ? "\u001b[31m" : "";
    const std::string PURPLE = mContext.canUseColor() ? "\u001b[35m" : "";
    const std::string GREEN = mContext.canUseColor() ? "\u001b[32m" : "";
    const std::string NORMAL = mContext.canUseColor() ? "\u001b[0m" : "";
    std::string errText;
    switch (e->mType) {
    case ErrorType::ERROR:
        errText += RED + "error" + NORMAL;
        break;
    case ErrorType::WARNING:
        errText += PURPLE + "warning" + NORMAL;
        break;
    default:
        assert ("invalid enumeration value" && false);
        break;
    }
    errText += " @ " + e->mFilename + ":" + std::to_string(e->mLineNum) + ":" + std::to_string(e->mColNum + 1) + "\n";
    std::string lineNum = std::to_string(e->mLineNum);
    std::string padding(lineNum.length(), ' ');
    errText += " " + padding + " | " + e->mText + "\n";
    errText += " " + lineNum + " | " + e->mLine + "\n";
    errText += " " + padding + " | " + std::string(e->mColNum, ' ') + GREEN + "^" + (e->mHint.empty() ? "" : " " + e->mHint) + NORMAL + "\n\n";
    return errText;
}

ErrorManager::ErrorManager(ErrorContext const & context)
: mContext(context)
, mErrorList()
, mFilename()
, mErrorCount(0)
, mCanContinue(true)
, mLineRef(nullptr)
, mLineNumRef(nullptr)
, mColNumRef(nullptr)
{
    mErrorList.reserve(context.maxErrorNum);
}

} // namespace pablo::parse
} // namespace pablo
