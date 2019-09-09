/*
 *  Copyright (c) 2019 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#pragma once

#include <string>
#include <vector>
#include <boost/iostreams/device/mapped_file.hpp>
#include <boost/utility/string_view.hpp>

namespace pablo {
namespace parse {

class Token;
class Lexer;

/**
 * Wraps around boost::iostreams::mapped_file_source to provide read-only
 * access to a file with LF terminated line granularity.
 * 
 * Implements a lazy approach to determinting where lines start and end in the
 * file. This means that only lines which have been scanned through using the
 * `nextLine` method can be referenced back with the `line` method.
 */
class SourceFile {
public:

    /**
     * Creates a source file instance by opening the file at a path relative to
     * some static base path.
     * 
     * The base path is: $HOME/.cache/parabix
     * 
     * @param path  The path to the source file relative to the base path. For
     *              example, path = "src.pablo" will get the file:
     *              '$HOME/.cache/parabix/src.pablo'.
     * @return A pointer to the newly created source file or nullptr if unable
     *  to open the file.
     */
    static std::shared_ptr<SourceFile> Relative(std::string const & path);

    /**
     * Creates a source file instance by opening the file at an unmodified path.
     * The path supplied to the function will not be modified i.e., 'src.pablo'
     * will get the file named 'src.pablo' in the current working directory of
     * the application.
     * 
     * @param path  A path to a source file.
     * @return A pointer to the newly created source file or nullptr if unable
     *  to open the file.
     */
    static std::shared_ptr<SourceFile> Absolute(std::string const & path);

public:
    SourceFile() = delete;
    explicit SourceFile(std::string const & filename);
    ~SourceFile();

    /**
     * Scans through the next line in the source file and stores a string_view
     * for the line in {@param view} as well as internally so that the line can
     * be referenced later through the `line` method.
     * 
     * Returns true if there are more source lines in the file or false if all
     * lines have been scanned through.
     * 
     * @param view where to store the string_view for the consumed source line
     * @return false if at EOF, true otherwise
     */
    bool nextLine(boost::string_view & view);

    /**
     * Returns a string view reference for a given (1-indexed) line number. The
     * line number must be less than or equal to the number of lines already
     * iterated through in the file. This means that if this source file has
     * only been scanned up to line 20, line 21 and above cannot be referenced
     * through this method.
     * 
     * @param num a 1-indexed line number
     * @return a string_view reference for the line
     */
    boost::string_view const & line(size_t num) const;

    std::string const & getFilename() const { return mFilename; }

    /**
     * Tokenizes this file, storing the tokens in an internal buffer.
     */
    bool lex(Lexer & lexer);

    std::vector<Token *> const & getTokenList() const { assert(mTokenList); return *mTokenList; }
private:
    std::string                             mFilename;
    boost::iostreams::mapped_file_source    mSource;
    std::vector<boost::string_view>         mLineRefs;
    const char *                            mCursor;
    std::vector<Token *> *                  mTokenList;
};

} // namespace pablo::parse
} // namespace pablo
