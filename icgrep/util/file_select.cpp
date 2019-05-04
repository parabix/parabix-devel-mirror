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
#include <re/parsers/parser.h>
#include <re/re_alt.h>
#include <re/re_seq.h>
#include <re/re_start.h>
#include <re/re_end.h>
#include <re/re_cc.h>
#include <re/re_toolchain.h>
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

std::string ExcludeFlag;
static cl::opt<std::string, true> ExcludeOption("exclude", cl::location(ExcludeFlag), cl::desc("Exclude files matching the given filename GLOB pattern."), cl::cat(Input_Options));

std::string ExcludeFromFlag;
static cl::opt<std::string, true> ExcludeFromOption("exclude-from", cl::location(ExcludeFromFlag), cl::desc("Exclude files matching filename GLOB patterns from the given file."), cl::cat(Input_Options));

std::string ExcludeDirFlag;
static cl::opt<std::string, true> ExcludeDirOption("exclude-dir", cl::location(ExcludeDirFlag), cl::desc("Exclude directories matching the given pattern."),
                                                   cl::init(".svn"), cl::cat(Input_Options));

std::string IncludeDirFlag;
static cl::opt<std::string, true> IncludeDirOption("include-dir", cl::location(IncludeDirFlag), cl::desc("Include directories matching the given pattern."), cl::cat(Input_Options));

std::string IncludeFlag;
static cl::opt<std::string, true> IncludeOption("include", cl::location(IncludeFlag), cl::desc("Include only files matching the given filename GLOB pattern."), cl::cat(Input_Options));

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

// Command line arguments to specify file and directory includes/excludes
// use GLOB syntax, matching any full pathname suffix after a "/", or
// the full filename of any recursively selected file or directory.
re::RE * anchorToFullFileName(re::RE * glob) {
    return re::makeSeq({re::makeAlt({re::makeStart(), re::makeCC('/')}), glob, re::makeEnd()});
}

bool UseStdIn;

re::RE * getDirectoryExcludePattern() {
    if (ExcludeDirFlag != "") {
        auto excludeDir = re::RE_Parser::parse(ExcludeDirFlag, re::DEFAULT_MODE, re::RE_Syntax::FileGLOB);
        return anchorToFullFileName(excludeDir);
    } else {
        return re::makeAlt();  // matches nothing, so excludes nothing.
    }
}

re::RE * getDirectoryIncludePattern() {
    if (IncludeDirFlag != "") {
        auto dir = re::RE_Parser::parse(IncludeDirFlag, re::DEFAULT_MODE, re::RE_Syntax::FileGLOB);
        return anchorToFullFileName(dir);
    } else {
        return re::makeEnd();  // matches every line..
    }
}

re::RE * getFileExcludePattern() {
    std::vector<re::RE *> patterns;
    if (ExcludeFlag != "") {
        re::RE * glob = re::RE_Parser::parse(ExcludeFlag, re::DEFAULT_MODE, re::RE_Syntax::FileGLOB);
        patterns.push_back(glob);
    }
    if (ExcludeFromFlag != "") {
        std::ifstream globFile(ExcludeFromFlag.c_str());
        std::string r;
        if (globFile.is_open()) {
            while (std::getline(globFile, r)) {
                re::RE * glob = re::RE_Parser::parse(r, re::DEFAULT_MODE, re::RE_Syntax::FileGLOB);
                patterns.push_back(glob);
            }
            globFile.close();
        }
    }
    if (patterns.empty()) return re::makeAlt();  // matches nothing, so excludes nothing.
    return anchorToFullFileName(re::makeAlt(patterns.begin(), patterns.end()));
}

re::RE * getFileIncludePattern() {
    if (IncludeFlag != "") {
        re::RE * includeSpec = re::RE_Parser::parse(IncludeFlag, re::DEFAULT_MODE, re::RE_Syntax::FileGLOB);
        includeSpec = anchorToFullFileName(includeSpec);
        return includeSpec;
    } else {
        return re::makeEnd();  // matches every line.
    }
}

namespace fs = boost::filesystem;

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
        mCollectedPaths.push_back(p);
   } else {
        assert(mDirectoryIndex < mDirectoryList.size());
        while (fileIdx >= mCumulativeEntryCount[mDirectoryIndex]) {
            mDirectoryIndex++;
        }
        mCollectedPaths.emplace_back(mDirectoryList[mDirectoryIndex]/std::string(name_start, name_end - name_start));
    }
}

std::vector<fs::path> getFullFileList(cl::list<std::string> & inputFiles) {
    // The vector to accumulate the full list of collected files to be searched.
    std::vector<fs::path> collectedPaths;

    // In this pass through command line arguments and the file hierarchy,
    // we are just gathering file and subdirectory entries, so we silently
    // ignore errors.  We use the boost::filesystem operations that set
    // error codes rather than raise exceptions.

    // In non-recursive greps with no include/exclude processing, we simply assemble the
    // paths.
    if ((DirectoriesFlag != Recurse) && (ExcludeFlag.empty()) && (IncludeFlag.empty()) && (ExcludeFromFlag.empty())) {
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
                if (!NoMessagesFlag) collectedPaths.push_back(p);
            } else if (fs::is_directory(s)) {
                if (DirectoriesFlag == Read) {
                    collectedPaths.push_back(p);
                }
            } else if (fs::is_regular_file(s)) {
                collectedPaths.push_back(p);
            } else {
                // Devices and unknown file types
                if (DevicesFlag == Read) {
                    collectedPaths.push_back(p);
                }
            }
        }
        return collectedPaths;
    }

    // Otherwise we need to filter paths according to some include/exclude rules.

    FileSelectAccumulator fileAccum(collectedPaths);

    // At each level we gather candidate file and directory names and then
    // filter the names based on -include, -exclude, -include-dir, -excclude-dir,
    // and -exclude-from settings.
    //
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
    fileAccum.setFullPathEntries(commandLineFileCandidates);
    if (commandLineDirCandidates > 0) {
        // Recursive processing of directories has been requested and we have
        // candidate directories from the command line.

        // selectedDirectories will accumulate hold the results of directory
        // include/exclude filtering at each level of processing.
        std::vector<fs::path> selectedDirectories;
        FileSelectAccumulator directoryAccum(selectedDirectories);

        CPUDriver driver("driver");
        grep::InternalSearchEngine directorySelectEngine(driver);
        directorySelectEngine.setRecordBreak(grep::GrepRecordBreakKind::Null);
        directorySelectEngine.grepCodeGen(getDirectoryIncludePattern(), getDirectoryExcludePattern());

        // The initial grep search determines which of the command line directories to process.
        // Each of these candidates is a full path return from command line argument processing.
        directoryAccum.setFullPathEntries(dirCandidates.getCandidateCount());
        directorySelectEngine.doGrep(dirCandidates.data(), dirCandidates.size(), directoryAccum);
        grep::SearchableBuffer subdirCandidates;

        while (!selectedDirectories.empty()) {
            // We now iterate through the full list of directories, gathering
            // entries from each.
            // (a) File entries are added into the global list of fileCandidates.
            // (b) Directory entries are added into a new list of candidates at each level.

            std::vector<fs::path> currentDirectories;
            assert (currentDirectories.empty());
            currentDirectories.swap(selectedDirectories);
            assert (selectedDirectories.empty());
            assert (!currentDirectories.empty());

            subdirCandidates.reset();
            directoryAccum.reset();

            // Iterate through all directories, collecting subdirectory and file candidates.
            for (const auto & dirpath : currentDirectories) {
                boost::system::error_code errc;
                fs::directory_iterator di(dirpath, errc);
                if (errc) {
                    // If we cannot enter the directory, keep it in the list of files,
                    // for possible error reporting.
                    if (!NoMessagesFlag) {
                        fileCandidates.append(dirpath.filename().string());
                    }
                    continue;
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
                            fileCandidates.append(e.filename().string());
                        }
                    } else if (fs::is_directory(s)) {
                        if (fs::is_symlink(e) && !DereferenceRecursiveFlag) {
                            di.increment(errc);
                            continue;
                        }
                        subdirCandidates.append(e.filename().string());
                    } else if (fs::is_regular_file(s) || DevicesFlag == Read) {
                        fileCandidates.append(e.filename().string());
                    }
                    boost::system::error_code errc2;
                    di.increment(errc2);
                    if (errc2) break;
                }
                // For each directory, update counts for candidates generated at this level.
                //
                directoryAccum.addDirectory(dirpath, subdirCandidates.getCandidateCount());
                fileAccum.addDirectory(dirpath, fileCandidates.getCandidateCount());
            }


            // Now do the search to produce the next level of selected subdirectories
            directorySelectEngine.doGrep(subdirCandidates.data(), subdirCandidates.size(), directoryAccum);
            // Thre search result has been written to directoryList, continue while we
            // have new subdirectories.
        }
    }
    //  All directories have been processed and all the fileCandidates in the SearchBuffer.
    //  Now determine which of the candidates should included or excluded from the search.
    //  The results will be accumulated in collectedPaths.

    CPUDriver driver("driver");
    grep::InternalSearchEngine fileSelectEngine(driver);
    fileSelectEngine.setRecordBreak(grep::GrepRecordBreakKind::Null);
    fileSelectEngine.grepCodeGen(getFileIncludePattern(), getFileExcludePattern());
    fileSelectEngine.doGrep(fileCandidates.data(), fileCandidates.size(), fileAccum);
    return collectedPaths;
}

}
