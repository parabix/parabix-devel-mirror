/*
 *  Copyright (c) 2019 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#include <pablo/parse/error.h>

#include <cassert>
#include <iostream>
#include <boost/make_unique.hpp>
#include <pablo/parse/source_file.h>
#include <pablo/parse/token.h>

namespace pablo {
namespace parse {

using namespace __internal;


Error::Error(Error const &other)
: type(other.type)
, text(other.text)
, source(other.source)
, lineNum(other.lineNum)
, colNum(other.colNum)
, hint(other.hint)
, width(other.width)
{}


Error::Error(ErrorType type,
             std::string const & text,
             std::weak_ptr<SourceFile> source,
             size_t lineNum,
             size_t colNum,
             std::string const & hint,
             size_t width)
: type(type)
, text(text)
, source(std::move(source))
, lineNum(lineNum)
, colNum(colNum)
, hint(hint)
, width(width)
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


void ErrorManager::log(std::unique_ptr<Error> e) {
    switch (e->type) {
    case ErrorType::NOTE:
        if (mContext.useLivePrint) {
            mContext.outStream << renderError(e);
        }
        mErrorList.push_back(std::move(e));
        break;
    case ErrorType::WARNING:
        if (mContext.useLivePrint) {
            mContext.outStream << renderError(e);
        }
        if (mContext.useWarnings) {
            mErrorList.push_back(std::move(e));
        }
        break;
    case ErrorType::TEXT_ONLY:
        /* fallthrough */
    case ErrorType::FATAL:
        mShouldContinue = false;
        /* fallthrough */
    case ErrorType::ERROR:
        if (mContext.useLivePrint) {
            mContext.outStream << renderError(e);
        }
        mErrorList.push_back(std::move(e));
        mErrorCount++;
        break;
    default:
        assert ("unexpected error type" && false);
        break;
    }
}


void ErrorManager::logTextError(std::weak_ptr<SourceFile> source, std::string const & text) {
    log(boost::make_unique<Error>(ErrorType::TEXT_ONLY, text, std::move(source), 0, 0, "", 0));
}


void ErrorManager::logError(std::weak_ptr<SourceFile> source, size_t lineNum, size_t colNum, std::string const & text, std::string const & hint) {
    log(boost::make_unique<Error>(ErrorType::ERROR, text, std::move(source), lineNum, colNum, hint, /*width:*/ 1));
}


void ErrorManager::logWarning(std::weak_ptr<SourceFile> source, size_t lineNum, size_t colNum, std::string const & text, std::string const & hint) {
    log(boost::make_unique<Error>(ErrorType::WARNING, text, std::move(source), lineNum, colNum, hint, /*width:*/ 1));
}


void ErrorManager::logFatalError(std::weak_ptr<SourceFile> source, size_t lineNum, size_t colNum, std::string const & text, std::string const & hint) {
    log(boost::make_unique<Error>(ErrorType::FATAL, text, std::move(source), lineNum, colNum, hint, /*width:*/ 1));
}


void ErrorManager::logNote(std::weak_ptr<SourceFile> source, size_t lineNum, size_t colNum, std::string const & text, std::string const & hint) {
    log(boost::make_unique<Error>(ErrorType::NOTE, text, std::move(source), lineNum, colNum, hint, /*width*/ 1));
}


void ErrorManager::logError(Token * t, std::string const & text, std::string const & hint) {
    log(boost::make_unique<Error>(ErrorType::ERROR, text, t->getSourceRef(), t->getLineNum(), t->getColNum(), hint, t->getText().length()));
}


void ErrorManager::logWarning(Token * t, std::string const & text, std::string const & hint) {
    log(boost::make_unique<Error>(ErrorType::WARNING, text, t->getSourceRef(), t->getLineNum(), t->getColNum(), hint, t->getText().length()));
}


void ErrorManager::logFatalError(Token * t, std::string const & text, std::string const & hint) {
    log(boost::make_unique<Error>(ErrorType::FATAL, text, t->getSourceRef(), t->getLineNum(), t->getColNum(), hint, t->getText().length()));
}


void ErrorManager::logNote(Token * t, std::string const & text, std::string const & hint) {
    log(boost::make_unique<Error>(ErrorType::NOTE, text, t->getSourceRef(), t->getLineNum(), t->getColNum(), hint, t->getText().length()));
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


std::string ErrorManager::renderError(std::unique_ptr<Error> const & e) const {
    const std::string BLACK = mContext.canUseColor() ? "\u001b[90m" : "";
    const std::string RED = mContext.canUseColor() ? "\u001b[31m" : "";
    const std::string PURPLE = mContext.canUseColor() ? "\u001b[35m" : "";
    const std::string GREEN = mContext.canUseColor() ? "\u001b[32m" : "";
    const std::string NORMAL = mContext.canUseColor() ? "\u001b[0m" : "";
    std::string errText{};
    std::string sourceLine{};
    std::string filename{};
    if (auto source = e->source.lock()) {
        if (e->lineNum != 0) { // lineNum == 0 for text only errors
            sourceLine = source->line(e->lineNum).to_string();
        }
        filename = source->getFilename();
    } else {
        assert ("source file reference is no longer valid" && false);
        sourceLine = "<source file information is not available>";
        filename = sourceLine;
    }

    switch (e->type) {
    case ErrorType::TEXT_ONLY:
        errText += RED + "error" + NORMAL;
        errText += " in " + filename + "\n";
        errText += e->text + "\n\n";
        return errText;
    case ErrorType::FATAL:
    /* fallthrough */
    case ErrorType::ERROR:
        errText += RED + "error" + NORMAL;
        break;
    case ErrorType::WARNING:
        errText += PURPLE + "warning" + NORMAL;
        break;
    case ErrorType::NOTE:
        errText += BLACK + "note" + NORMAL;
        break;
    default:
        assert ("invalid enumeration value" && false);
        break;
    }
    
    errText += " at " + filename + ":" + std::to_string(e->lineNum) + ":" + std::to_string(e->colNum) + "\n";
    std::string lineNum = std::to_string(e->lineNum);
    std::string padding(lineNum.length(), ' ');
    errText += " " + padding + " | " + e->text + "\n";
    // append '\n' to source line if one doesn't exist
    if (sourceLine.back() != '\n')
        sourceLine.push_back('\n');
    errText += " " + lineNum + " | " + sourceLine;
    std::string marker = "^" + std::string(e->width - 1, '~');
    errText += " " + padding + " | " + std::string(e->colNum - 1, ' ') + GREEN + marker + (e->hint.empty() ? "" : " " + e->hint) + NORMAL + "\n\n";
    return errText;
}


ErrorManager::ErrorManager()
: mContext(ErrorContext{})
, mErrorList()
, mErrorCount(0)
, mShouldContinue(true)
{
    mErrorList.reserve(mContext.maxErrorNum);
}


ErrorManager::ErrorManager(ErrorContext const & context)
: mContext(context)
, mErrorList()
, mErrorCount(0)
, mShouldContinue(true)
{
    mErrorList.reserve(context.maxErrorNum);
}

} // namespace pablo::parse
} // namespace pablo
