/*
 *  Copyright (c) 2018 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#include "file_select.h"
#include <llvm/Support/CommandLine.h>
#include <llvm/Support/ErrorHandling.h>
#include <llvm/Support/Signals.h>
#include <llvm/Support/raw_ostream.h>
#include <boost/filesystem.hpp>
#include <toolchain/toolchain.h>
#include <re/re_re.h>
#include <re/re_alt.h>
#include <re/re_seq.h>
#include <re/re_start.h>
#include <re/re_end.h>
#include <re/re_cc.h>
#include <re/re_assertion.h>
#include <re/re_toolchain.h>
#include <re/parsers/parser.h>
#include <re/printer_re.h>
#include <grep/grep_engine.h>
#include <grep/searchable_buffer.h>
#include <toolchain/cpudriver.h>
#include <fstream>
#include <string>
#include <re/printer_re.h>

using namespace llvm;

namespace argv {

static cl::OptionCategory Input_Options("File Selection Options", "These options control the input sources.");

bool NoMessagesFlag;
static cl::opt<bool, true> NoMessagesOption("s", cl::location(NoMessagesFlag), cl::desc("Suppress messages for file errors."), cl::cat(Input_Options), cl::Grouping);
static cl::alias NoMessagesAlias("no-messages", cl::desc("Alias for -s"), cl::aliasopt(NoMessagesOption));

bool RecursiveFlag;
static cl::opt<bool, true> RecursiveOption("r", cl::location(RecursiveFlag), cl::desc("Recursively process files within directories, (but follow only top-level symlinks unless -R)."), cl::cat(Input_Options), cl::Grouping);
static cl::alias RecursiveAlias("recursive", cl::desc("Alias for -r"), cl::aliasopt(RecursiveOption));

bool DereferenceRecursiveFlag;
static cl::opt<bool, true> DereferenceRecursiveOption("R", cl::location(DereferenceRecursiveFlag), cl::desc("Recursively process files within directories, following symlinks at all levels."), cl::cat(Input_Options), cl::Grouping);
static cl::alias DereferenceRecursiveAlias("dereference-recursive", cl::desc("Alias for -R"), cl::aliasopt(DereferenceRecursiveOption));


bool MmapFlag;
static cl::opt<bool, true> MmapOption("mmap", cl::location(MmapFlag),  cl::init(1), cl::desc("Use mmap for file input (default)."), cl::cat(Input_Options));

static cl::list<std::string> ExcludeFiles("exclude", cl::ZeroOrMore,
                                          cl::desc("Exclude files matching the given filename GLOB pattern."), cl::cat(Input_Options));

static cl::opt<std::string> ExcludeFromFlag("exclude-from", cl::desc("Exclude files matching filename GLOB patterns from the given file."), cl::cat(Input_Options));

static cl::list<std::string> ExcludeDirectories("exclude-dir", cl::desc("Exclude directories matching the given pattern."), cl::ZeroOrMore, cl::cat(Input_Options));
    
static cl::opt<std::string> ExcludePerDirectory("exclude-per-directory",
                                                cl::desc(".gitignore (default) or other file specifying files to exclude."),
                                                cl::init(".gitignore"), cl::cat(Input_Options));
    
static cl::list<std::string> IncludeDirectories("include-dir", cl::desc("Include directories matching the given pattern."), cl::ZeroOrMore, cl::cat(Input_Options));

static cl::list<std::string> IncludeFiles("include", cl::ZeroOrMore, cl::desc("Include only files matching the given filename GLOB pattern(s)."), cl::cat(Input_Options));

DevDirAction DevicesFlag;
static cl::opt<DevDirAction, true> DevicesOption("D", cl::desc("Processing mode for devices:"),
                                                 cl::values(clEnumValN(Read, "read", "Treat devices as files to be searched."),
                                                            clEnumValN(Skip, "skip", "Silently skip devices.")
                                                            CL_ENUM_VAL_SENTINEL), cl::cat(Input_Options), cl::location(DevicesFlag), cl::init(Read));
static cl::alias DevicesAlias("devices", cl::desc("Alias for -D"), cl::aliasopt(DevicesOption));

DevDirAction DirectoriesFlag;
static cl::opt<DevDirAction, true> DirectoriesOption("d", cl::desc("Processing mode for directories:"),
                                                     cl::values(clEnumValN(Read, "read", "Print an error message for any listed directories."),
                                                                clEnumValN(Skip, "skip", "Silently skip directories."),
                                                                clEnumValN(Recurse, "recurse", "Recursive process directories, equivalent to -r.")
                                                                CL_ENUM_VAL_SENTINEL), cl::cat(Input_Options), cl::location(DirectoriesFlag), cl::init(Read));
static cl::alias DirectoriesAlias("directories", cl::desc("Alias for -d"), cl::aliasopt(DirectoriesOption));
    
static cl::opt<bool> TraceFileSelect("TraceFileSelect", cl::desc("Trace file selection"), cl::cat(Input_Options));

// Command line arguments to specify file and directory includes/excludes
// use GLOB syntax, matching any full pathname suffix after a "/", or
// the full filename of any recursively selected file or directory.
re::RE * anchorToFullFileName(re::RE * glob) {
    return re::makeSeq({re::makeAlt({re::makeStart(), re::makeCC('/')}), glob, re::makeEnd()});
}

bool UseStdIn;

re::RE * getDirectoryPattern() {
    std::vector<re::RE *> includedPatterns;
    re::RE * dirRE = nullptr;
    if (IncludeDirectories.empty()) {
        dirRE = re::makeEnd();  // matches every line.
    } else {
        auto it = IncludeDirectories.begin();
        while (it != IncludeDirectories.end()) {
            includedPatterns.push_back(re::RE_Parser::parse(*it, re::DEFAULT_MODE, re::RE_Syntax::FileGLOB));
            ++it;
        }
        dirRE = anchorToFullFileName(re::makeAlt(includedPatterns.begin(), includedPatterns.end()));
    }
    if (ExcludeDirectories.empty()) {
        return dirRE;
    } else {
        std::vector<re::RE *> excludedPatterns;
        auto it = ExcludeDirectories.begin();
        while (it != ExcludeDirectories.end()) {
            re::RE * glob = re::RE_Parser::parse(*it, re::DEFAULT_MODE, re::RE_Syntax::FileGLOB);
            excludedPatterns.push_back(glob);
            ++it;
        }
        re::RE * excl = anchorToFullFileName(re::makeAlt(excludedPatterns.begin(), excludedPatterns.end()));
        return re::makeSeq({dirRE, re::makeNegativeLookBehindAssertion(anchorToFullFileName(excl))});
    }
}
    
re::RE * getFilePattern() {
    std::vector<re::RE *> includedPatterns;
    re::RE * fileRE = nullptr;
    if (IncludeFiles.empty()) {
        fileRE = re::makeEnd();  // matches every line.
    } else {
        auto it = IncludeFiles.begin();
        while (it != IncludeFiles.end()) {
            includedPatterns.push_back(re::RE_Parser::parse(*it, re::DEFAULT_MODE, re::RE_Syntax::FileGLOB));
            ++it;
        }
        fileRE = anchorToFullFileName(re::makeAlt(includedPatterns.begin(), includedPatterns.end()));
    }
    std::vector<re::RE *> excludedPatterns;
    if (!ExcludeFiles.empty()) {
        auto it = ExcludeFiles.begin();
        while (it != ExcludeFiles.end()) {
            re::RE * glob = re::RE_Parser::parse(*it, re::DEFAULT_MODE, re::RE_Syntax::FileGLOB);
            excludedPatterns.push_back(glob);
            ++it;
        }
    }
    if (ExcludeFromFlag != "") {
        std::ifstream globFile(ExcludeFromFlag.c_str());
        std::string r;
        if (globFile.is_open()) {
            while (std::getline(globFile, r)) {
                re::RE * glob = re::RE_Parser::parse(r, re::DEFAULT_MODE, re::RE_Syntax::FileGLOB);
                excludedPatterns.push_back(glob);
            }
            globFile.close();
        }
    }
    if (excludedPatterns.empty()) return fileRE;
    re::RE * exclRE = anchorToFullFileName(re::makeAlt(excludedPatterns.begin(), excludedPatterns.end()));
    return re::makeSeq({fileRE, re::makeNegativeLookBehindAssertion(exclRE)});
}

namespace fs = boost::filesystem;

void selectPath(std::vector<fs::path> & collectedPaths, fs::path p) {
    collectedPaths.push_back(p);
    if (TraceFileSelect) {
        llvm::errs() << "Selecting: " << p.string() << "\n";
    }
}

//
//  Directory List: a set of directory paths that have been
//  examined to identify candidate files for searching, together
//  with a count of the number of candidate files in each directory.
//
//  FileName Buffer: an ordered sequence of NUL terminated filenames
//  for each candidate produced in the directory traversal.
//  The first mFullPathEntries entries are CWD paths.  Subsequent entries
//  are base file names relative to a directory.   The set
//  of all entries for a given directory are consecutive in the
//  buffer, and the sets are ordered consecutively by directory
//  index in the Directory List.
//
//  CollectedPaths: a vector of file paths to which the
//  selected files are added.

class FileSelectAccumulator : public grep::MatchAccumulator {
public:
    FileSelectAccumulator(std::vector<fs::path> & collectedPaths) :
        mCollectedPaths(collectedPaths),
        mFullPathEntries(0)
    {}
    void setFullPathEntries(unsigned entries) {mFullPathEntries = entries; mDirectoryIndex = 0;}
    void reset();
    void addDirectory(fs::path dirPath, unsigned cumulativeEntryCount);
    void accumulate_match(const size_t lineNum, char * line_start, char * line_end) override;
protected:
    std::vector<fs::path> & mCollectedPaths;
    unsigned mFullPathEntries;
    unsigned mDirectoryIndex;
    std::vector<fs::path> mDirectoryList;
    std::vector<unsigned> mCumulativeEntryCount;
};

void FileSelectAccumulator::reset() {
    mCollectedPaths.clear();
    mFullPathEntries = 0;
    mDirectoryIndex = 0;
    mDirectoryList.clear();
    mCumulativeEntryCount.clear();
}

void FileSelectAccumulator::addDirectory(fs::path dirPath, unsigned cumulativeEntryCount) {
    mDirectoryList.push_back(dirPath);
    mCumulativeEntryCount.push_back(cumulativeEntryCount);
}

void FileSelectAccumulator::accumulate_match(const size_t fileIdx, char * name_start, char * name_end) {
    fs::path p(std::string(name_start, name_end - name_start));
    if (fileIdx < mFullPathEntries) {
        selectPath(mCollectedPaths, p);
   } else {
        assert(mDirectoryIndex < mDirectoryList.size());
        while (fileIdx >= mCumulativeEntryCount[mDirectoryIndex]) {
            mDirectoryIndex++;
        }
        selectPath(mCollectedPaths, mDirectoryList[mDirectoryIndex]/std::string(name_start, name_end - name_start));
    }
}

//
// Parsing files using .gitignore conventions.
// Given REs that select directories and files to be processed, override
// these selections for each directory or file that is to be ignored.
//
std::pair<re::RE *, re::RE *> parseIgnoreFile(fs::path dirpath, std::string ignoreFileName, re::RE * dirRE, re::RE * fileSelectRE) {
    fs::path ignoreFilePath = dirpath/ignoreFileName;
    re::RE * updatedDirRE = dirRE;
    re::RE * updatedFileRE = fileSelectRE;
    std::ifstream ignoreFile(ignoreFilePath.string());
    if (ignoreFile.is_open()) {
        if (TraceFileSelect) {
            llvm::errs() << "Parsing exclude file: " << ignoreFilePath.string() << "\n";
        }
        std::string line;
        while (std::getline(ignoreFile, line)) {
            bool is_directory_only_pattern = false;
            bool is_include_override = false;
            if (line.empty() || (line[0] == '#')) continue;  // skip empty and comment lines.
            unsigned line_start = 0;
            unsigned line_end = line.size() - 1;
            while ((line[line_end] == ' ') && (line_end > 0)) {
                line_end--;
            }
            if (line_end == 0) continue;  // skip blank lines.
            if (line[line_end] == '\\') {
                // Escape character found, but is it an escaped escape (\\)?
                // Determine whether we have an odd or even number of escapes.
                unsigned escape_pos = line_end;
                while ((escape_pos > 0) && (line[escape_pos-1] == '\\')) {
                    escape_pos--;
                }
                if (((line_end - escape_pos) & 1) == 1) {
                    // Odd number of escapes - the final space is escaped, not trimmed.
                    line_end++;
                }
            }
            if (line[line_end] == '/') {
                is_directory_only_pattern = true;
                line_end--;
            }
            if (line[0] == '!') { // negated ignore is an overriding include.
                is_include_override = true;
                line_start++;
            }
            // Convert any local patterns to full path patterns.
            if (line[0] == '/') {
                line_start++;
            }
            if ((line_start != 0) || (line_end != line.size() - 1)) {
                line = line.substr(line_start, line_end - line_start + 1);
            }
            re::RE * lineRE = re::RE_Parser::parse(line, re::DEFAULT_MODE, re::RE_Syntax::GitGLOB);
            if (is_include_override) {
                updatedDirRE = re::makeAlt({updatedDirRE, lineRE});
                if (!is_directory_only_pattern) {
                    updatedFileRE = re::makeAlt({updatedFileRE, lineRE});
                }
            } else { // A path to be ignored.
                lineRE = makeNegativeLookBehindAssertion(lineRE);
                updatedDirRE = re::makeSeq({updatedDirRE, lineRE});
                if (!is_directory_only_pattern) {
                    updatedFileRE = re::makeSeq({updatedFileRE, lineRE});
                }
            }
        }
        ignoreFile.close();
    }
    return std::make_pair(updatedDirRE, updatedFileRE);
}

void recursiveFileSelect(CPUDriver & driver, fs::path dirpath,
                         grep::InternalSearchEngine & inheritedDirectoryEngine, re::RE * directoryRE,
                         grep::InternalSearchEngine & inheritedFileEngine, re::RE * fileRE,
                         std::vector<fs::path> & collectedPaths) {

    // First update the search REs with any local .gitignore or other exclude file.
    bool hasLocalIgnoreFile = !ExcludePerDirectory.empty() && fs::exists(dirpath/ExcludePerDirectory);
    re::RE * updatedDirRE = directoryRE;
    re::RE * updatedFileRE = fileRE;
    if (hasLocalIgnoreFile) {
        std::tie(updatedDirRE, updatedFileRE) = parseIgnoreFile(dirpath, ExcludePerDirectory, directoryRE, fileRE);
    }
    // Gather files and subdirectories.
    grep::SearchableBuffer subdirCandidates;
    grep::SearchableBuffer fileCandidates;
    
    boost::system::error_code errc;
    fs::directory_iterator di(dirpath, errc);
    if (errc) {
        // If we cannot enter the directory, keep it in the list of files,
        // for possible error reporting.
        if (!NoMessagesFlag) {
            collectedPaths.push_back(dirpath);
        }
        return;
    }
    const auto di_end = fs::directory_iterator();
    while (di != di_end) {
        const auto & e = di->path();
        boost::system::error_code errc;
        const auto s = fs::status(e, errc);
        if (errc) {
            // If there was an error, we leave the file in the fileCandidates
            // list for later error processing.
            if (!NoMessagesFlag) {
                fileCandidates.append(e.string());
            }
        } else if (fs::is_directory(s)) {
            if (fs::is_symlink(e) && !DereferenceRecursiveFlag) {
                di.increment(errc);
                continue;
            }
            subdirCandidates.append(e.string());
        } else if (fs::is_regular_file(s) || DevicesFlag == Read) {
            fileCandidates.append(e.string());
        }
        boost::system::error_code errc2;
        di.increment(errc2);
        if (errc2) break;
    }
    // For each directory, update counts for candidates generated at this level.
    //
    FileSelectAccumulator fileAccum(collectedPaths);
    std::vector<fs::path> selectedDirectories;
    FileSelectAccumulator directoryAccum(selectedDirectories);
    fileAccum.setFullPathEntries(fileCandidates.getCandidateCount());
    directoryAccum.setFullPathEntries(subdirCandidates.getCandidateCount());

    if (hasLocalIgnoreFile) {
        // Compile the update file and directory selection REs, and apply them to
        // choose files and directories for processing.
        // Apply the file selection RE to choose files for processing, adding
        // them to the global list of selected files.
        grep::InternalSearchEngine fileSelectEngine(driver);
        fileSelectEngine.setRecordBreak(grep::GrepRecordBreakKind::Null);
        fileSelectEngine.grepCodeGen(updatedFileRE);
        fileSelectEngine.doGrep(fileCandidates.data(), fileCandidates.size(), fileAccum);
        // Now select subdirectories for recursive processing.
        grep::InternalSearchEngine directorySelectEngine(driver);
        directorySelectEngine.setRecordBreak(grep::GrepRecordBreakKind::Null);
        directorySelectEngine.grepCodeGen(updatedDirRE);
        directorySelectEngine.doGrep(subdirCandidates.data(), subdirCandidates.size(), directoryAccum);
        // Work through the subdirectories, invoking the recursive processing for each.
        for (const auto & subdir : selectedDirectories) {
            recursiveFileSelect(driver, subdir,
                                directorySelectEngine, updatedDirRE,
                                fileSelectEngine, updatedFileRE,
                                collectedPaths);
        }
    } else {
        inheritedFileEngine.doGrep(fileCandidates.data(), fileCandidates.size(), fileAccum);
        inheritedDirectoryEngine.doGrep(subdirCandidates.data(), subdirCandidates.size(), directoryAccum);
        for (const auto & subdir : selectedDirectories) {
            recursiveFileSelect(driver, subdir,
                                inheritedDirectoryEngine, directoryRE,
                                inheritedFileEngine, fileRE,
                                collectedPaths);
        }
    }
}

std::vector<fs::path> getFullFileList(CPUDriver & driver, cl::list<std::string> & inputFiles) {
    // The vector to accumulate the full list of collected files to be searched.
    std::vector<fs::path> collectedPaths;

    // In this pass through command line arguments and the file hierarchy,
    // we are just gathering file and subdirectory entries, so we silently
    // ignore errors.  We use the boost::filesystem operations that set
    // error codes rather than raise exceptions.

    // In non-recursive greps with no include/exclude processing, we simply assemble the
    // paths.
    if ((DirectoriesFlag != Recurse) && (ExcludeFiles.empty()) && (IncludeFiles.empty()) && (ExcludeFromFlag.empty())) {
        for (const std::string & f : inputFiles) {
            if (f == "-") {  // stdin, will always be searched.
                argv::UseStdIn = true;
                continue;
            }
            fs::path p(f);
            boost::system::error_code errc;
            fs::file_status s = fs::status(p, errc);
            if (errc) {
                // If there was an error, we leave the file in the fileCandidates
                // list for later error processing.
                if (!NoMessagesFlag) {
                    selectPath(collectedPaths, p);
                }
            } else if (fs::is_directory(s)) {
                if (DirectoriesFlag == Read) {
                    selectPath(collectedPaths, p);
                }
            } else if (fs::is_regular_file(s)) {
                selectPath(collectedPaths, p);
            } else {
                // Devices and unknown file types
                if (DevicesFlag == Read) {
                    selectPath(collectedPaths, p);
                }
            }
        }
        return collectedPaths;
    }

    // Otherwise we may need to filter paths according to some include/exclude rules.
    grep::SearchableBuffer dirCandidates;
    grep::SearchableBuffer fileCandidates;

    // First level of processing:  command line files and directories.
    for (const std::string & f : inputFiles) {
        if (f == "-") {  // stdin, will always be searched.
            argv::UseStdIn = true;
            continue;
        }
        fs::path p(f);
        boost::system::error_code errc;
        fs::file_status s = fs::status(p, errc);
        if (errc) {
            // If there was an error, we leave the file in the fileCandidates
            // list for later error processing.
            if (!NoMessagesFlag) fileCandidates.append(p.string());
        } else if (fs::is_directory(s)) {
            if (DirectoriesFlag == Recurse) {
                dirCandidates.append(p.string());
            } else if (DirectoriesFlag == Read) {
                fileCandidates.append(p.string());
            }
        } else if (fs::is_regular_file(s)) {
            fileCandidates.append(p.string());
        } else {
            // Devices and unknown file types
            if (DevicesFlag == Read) {
                fileCandidates.append(p.string());
            }
        }
    }

    auto commandLineDirCandidates = dirCandidates.getCandidateCount();
    auto commandLineFileCandidates = fileCandidates.getCandidateCount();
    FileSelectAccumulator fileAccum(collectedPaths);
    fileAccum.setFullPathEntries(commandLineFileCandidates);
    //
    // Apply the file selection RE to choose files for processing, adding
    // them to the global list of selected files.
    re::RE * fileSelectRE = getFilePattern();
    grep::InternalSearchEngine fileSelectEngine(driver);
    fileSelectEngine.setRecordBreak(grep::GrepRecordBreakKind::Null);
    fileSelectEngine.grepCodeGen(fileSelectRE);
    fileSelectEngine.doGrep(fileCandidates.data(), fileCandidates.size(), fileAccum);
    //
    //
    if (commandLineDirCandidates > 0) {
        // Recursive processing of directories has been requested and we have
        // candidate directories from the command line.

        // selectedDirectories will accumulate hold the results of directory
        // include/exclude filtering at each level of processing.
        std::vector<fs::path> selectedDirectories;
        FileSelectAccumulator directoryAccum(selectedDirectories);
        re::RE * directoryRE = getDirectoryPattern();
        grep::InternalSearchEngine directorySelectEngine(driver);
        directorySelectEngine.setRecordBreak(grep::GrepRecordBreakKind::Null);
        directorySelectEngine.grepCodeGen(directoryRE);

        // The initial grep search determines which of the command line directories to process.
        // Each of these candidates is a full path return from command line argument processing.
        directoryAccum.setFullPathEntries(dirCandidates.getCandidateCount());
        directorySelectEngine.doGrep(dirCandidates.data(), dirCandidates.size(), directoryAccum);
        
        // Select files from subdirectories using the recursive process.
        for (const auto & dirpath : selectedDirectories) {
            recursiveFileSelect(driver, dirpath,
                                directorySelectEngine, directoryRE,
                                fileSelectEngine, fileSelectRE,
                                collectedPaths);
        }
    }
    return collectedPaths;
}

}
