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
#include <pablo/parser/source_file.h>

namespace pablo {
namespace parse {

class Token;

namespace __internal {

enum class ErrorType {
    WARNING,
    ERROR,
    FATAL,
    TEXT_ONLY,
    NOTE
};


struct Error {
    Error(Error const & other);
    Error(ErrorType type,
          std::string const & text,
          std::shared_ptr<SourceFile> source,
          size_t lineNum,
          size_t colNum,
          std::string const & hint,
          size_t width);

    ErrorType                   type;
    std::string                 text;
    std::shared_ptr<SourceFile> source;
    size_t                      lineNum;
    size_t                      colNum;
    std::string                 hint;
    size_t                      width;
};

} // namespace __internal

/**
 * Defines parameters which determin how a ErrorManager instance operates.
 */
struct ErrorContext {
    ErrorContext();

    /**
     * Determines if ANSI coloring can be used when rending errors.
     *
     * By default, only returns true if OS is not Windows and the output stream
     * is either std::cout or std::cerr.
     */
    bool canUseColor() const;

    std::ostream &  outStream;
    bool            useWarnings;
    bool            useFatalErrors;
    bool            useLivePrint;
    size_t          maxErrorNum;
};


/**
 * Class responsible for managing lexical and parsing errors. Exposes methods
 * for logging new errors which can be used by lexers/parsers to offload the
 * responsability of error management to an external object.
 *
 * This class may be overriden to provide a differnet method for rending errors
 * as text.
 */
class ErrorManager {
public:

    static std::shared_ptr<ErrorManager> Create() {
        return std::make_shared<ErrorManager>();
    }

public:

    /**
     * Constructs an instance using the default ErrorContext.
     */
    ErrorManager();

    /**
     * Constructs an instance using a prespecified ErrorContext. Values in the
     * context will be queried to determine how this ErrorManager will behave.
     *
     * @param context An ErrorContext instance.
     */
    explicit ErrorManager(ErrorContext const & context);

    virtual ~ErrorManager() = default;

    /**
     * Logs an error without of location data. Errors of these types are treated
     * as fatal errors.
     *
     * @param source The source file in which the error occurred.
     * @param text A description of the error.
     */
    void logTextError(std::shared_ptr<SourceFile> source, std::string const & text);

    /**
     * Logs an error at a specified position in a source file.
     *
     * @param source The source file in which the error occurred.
     * @param lineNum The line number at which the error occurred.
     * @param colNum The column number at which the error occurred.
     * @param text A description of the error.
     * @param hint An optional hint about resolving the error.
     */
    void logError(std::shared_ptr<SourceFile> source, size_t lineNum, size_t colNum, std::string const & text, std::string const & hint = "");

    /**
     * Logs a warning at a specified position in a source file.
     *
     * @param source The source file in which the error occurred.
     * @param lineNum The line number at which the error occurred.
     * @param colNum The column number at which the error occurred.
     * @param text A description of the error.
     * @param hint An optional hint about resolving the error.
     */
    void logWarning(std::shared_ptr<SourceFile> source, size_t lineNum, size_t colNum, std::string const & text, std::string const & hint = "");

    /**
     * Logs a warning at a specified position in a source file.
     *
     * @param source The source file in which the error occurred.
     * @param lineNum The line number at which the error occurred.
     * @param colNum The column number at which the error occurred.
     * @param text A description of the error.
     * @param hint An optional hint about resolving the error.
     */
    void logFatalError(std::shared_ptr<SourceFile> source, size_t lineNum, size_t colNum, std::string const & text, std::string const & hint = "");

    /**
     * Logs a note at a specified position in a source file.
     *
     * @param source The source file where the note should be logged.
     * @param lineNum The line number where the note should be logged.
     * @param colNum The column number where the note should be logged.
     * @param text The textual body of the note.
     * @param hint An optional textual hint.
     */
    void logNote(std::shared_ptr<SourceFile> source, size_t lineNum, size_t colNum, std::string const & text, std::string const & hint = "");

    /**
     * Logs an error at a specified token. Source file and positional
     * information are inferred from the token.
     *
     * @param t The erroneous token.
     * @param text A description of the error.
     * @param hint An optional hint about resolving the error.
     */
    void logError(Token * t, std::string const & text, std::string const & hint = "");

    /**
     * Logs a warning at a specified token. Source file and positional
     * information are inferred from the token.
     *
     * @param t The erroneous token.
     * @param text A description of the error.
     * @param hint An optional hint about resolving the error.
     */
    void logWarning(Token * t, std::string const & text, std::string const & hint = "");

    /**
     * Logs a fatal error at a specified token. Source file and positional
     * information are inferred from the token.
     *
     * @param t The erroneous token.
     * @param text A description of the error.
     * @param hint An optional hint about resolving the error.
     */
    void logFatalError(Token * t, std::string const & text, std::string const & hint = "");

    /**
     * Logs a note at a specified token. Source file and positional info are
     * inferred from the token.
     *
     * @param t The token
     * @param text The textual body of the note.
     * @param hint An optional hint.
     */
    void logNote(Token * t, std::string const & text, std::string const & hint = "");

    /**
     * Returns true if the logging entity (lexer, parser, etc.) can continue
     * its operation or whether it should abort.
     *
     * Normally this will return true unless the maximum number of errors has
     * been reached or a fatal error has been logged.
     *
     * Note that even if it says that the logging entity cannot continue, the
     * error manager will still accept and log errors normally. This is simply
     * meant as a way to tell the logging entity that it may want to stop as
     * something has gone horribly wrong.
     */
    bool shouldContinue() const {
        return mShouldContinue;
    }

    /**
     * Returns true if an error has been logged.
     *
     * Note, warnings do not contribute to this.
     */
    bool hasErrors() const {
        return mErrorCount != 0;
    }

    /**
     * Returns the number of errors (NOT warnings) logged by this object.
     *
     * For the total number of error entities (including warnings) logged, use
     * `numLoggedEntities` instead.
     *
     * @return The error count.
     */
    size_t errorCount() const {
        return mErrorCount;
    }

    /**
     * Returns the number of logged error entities (including warnings). More
     * explicitly, returns the number of times, `log` has been called.
     *
     * Note that this may differ from the value returned by `errorCount` as
     * warnings do not directly contribute to the error count.
     *
     * @return The number of error entities.
     */
    size_t numLoggedEntities() const {
        return mErrorList.size();
    }

    /**
     * Converts all logged errors to a textual representation then dumps
     * them into the output stream defined in this object's ErrorContext.
     *
     * If no errors have been logged, this method does nothing.
     */
    void dumpErrors() const;

protected:

    /**
     * Converts an error into a textual representation which can be written
     * to a file or printed to a terminal. This method may be overriden by
     * subclasses to change how errors are rendered without changing how
     * they are handled internally.
     *
     * @param e An immutable reference to an error instance.
     * @return A textual representation of the error.
     */
    virtual std::string renderError(std::unique_ptr<__internal::Error> const & e) const;

private:

    /**
     * Logs an instance of an within this manager. Updates the state of this
     * object depending on the type of error and the context in which it is
     * logged. For example, this may include updating the error count as well
     * as setting the "can continue" flag to false.
     *
     * @param error Instance of the error to be logged. The lifetime of the
     *              instance is managed by this object.
     */
    void log(std::unique_ptr<__internal::Error> error);

protected:
    ErrorContext                                    mContext;
    std::vector<std::unique_ptr<__internal::Error>> mErrorList;
    size_t                                          mErrorCount;
    bool                                            mShouldContinue;
};

} // namespace pablo::parse
} // namespace pablo
