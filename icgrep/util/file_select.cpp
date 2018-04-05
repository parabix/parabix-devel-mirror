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
#include <re/re_toolchain.h>
#include <fstream>
#include <string>

using namespace llvm;

namespace argv {
    
static cl::OptionCategory Input_Options("File Selection Options", "These options control the input sources.");

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
static cl::opt<std::string, true> ExcludeDirOption("exclude-dir", cl::location(ExcludeDirFlag), cl::desc("Exclude directories matching the given pattern."), cl::cat(Input_Options));

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
    if (patterns.empty()) return nullptr;
    return re::makeAlt(patterns.begin(), patterns.end());
}

re::RE * getDirectoryExcludePattern() {
    if (ExcludeDirFlag != "") {
        return re::RE_Parser::parse(ExcludeDirFlag, re::DEFAULT_MODE, re::RE_Syntax::FileGLOB);
    }
    return nullptr;
}

re::RE * getFileIncludePattern() {
    if (IncludeFlag != "") {
        return re::RE_Parser::parse(IncludeFlag, re::DEFAULT_MODE, re::RE_Syntax::FileGLOB);
    }
    return nullptr;
}

// Include is the default unless a -include= option exists and is prior to any -exclude
// or -exclude-dir option.
bool includeIsDefault() {
    if (IncludeFlag == "") return true;
    if ((ExcludeFlag != "") && (ExcludeOption.getPosition() < IncludeOption.getPosition())) return true;
    if ((ExcludeDirFlag != "") && (ExcludeDirOption.getPosition() < IncludeOption.getPosition())) return true;
    return false;
}
    
    
    
namespace fs = boost::filesystem;

// This is a stub, to be expanded later.
bool excludeDirectory(fs::path dirpath) { return dirpath.filename() == ".svn";}

// Determine whether to skip a path based on -D skip or -d skip settings.
bool skip_path(fs::path p) {
    switch (fs::status(p).type()) {
        case fs::directory_file: return DirectoriesFlag == Skip;
        case fs::block_file:
        case fs::character_file:
        case fs::fifo_file:
        case fs::socket_file:
            return DevicesFlag == Skip;
        default:
            return false;
    }
}

void getSubdirectoryFiles(fs::path dirpath, std::vector<std::string> & collectedFiles) {
    boost::system::error_code errc;
    fs::directory_iterator di(dirpath, errc);
    fs::directory_iterator di_end;
    if (errc) {
        // If we cannot enter the directory, keep it in the list of files.
        collectedFiles.push_back(dirpath.string());
        return;
    }
    //FileAccumulator accum(dirpath, collectedFiles);
    while (di != di_end) {
        auto & e = di->path();
        if (fs::is_directory(e)) {
            if (fs::is_symlink(e) && !DereferenceRecursiveFlag) {
                continue;
            }
            if (!excludeDirectory(e)) {
                getSubdirectoryFiles(e, collectedFiles);
            }
        } else {
            if (!skip_path(e)) {
                collectedFiles.push_back(e.string());
            }
        }
        di.increment(errc);
        if (errc) {
            collectedFiles.push_back(e.string());
        }
    }
}

std::vector<std::string> getFullFileList(cl::list<std::string> & inputFiles) {
    std::vector<std::string> expanded_paths;
    boost::system::error_code errc;
    for (const std::string & f : inputFiles) {
        //        if (f == "-") {
        //            continue;
        //        }
        fs::path p(f);
        if (skip_path(p)) {
            continue;
        }
        if (LLVM_UNLIKELY((DirectoriesFlag == Recurse) && fs::is_directory(p))) {
            if (!excludeDirectory(p)) {
                getSubdirectoryFiles(p, expanded_paths);
            }
        } else {
            expanded_paths.push_back(p.string());
        }
    }
    return expanded_paths;
}

}
