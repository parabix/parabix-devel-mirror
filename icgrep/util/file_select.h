/*
 *  Copyright (c) 2018 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 *
 *  This file defines commmand-line options and parameters for 
 *  file selection as used by various command line tools.
 *
 */
#ifndef FILE_SELECT_H
#define FILE_SELECT_H
 
#include <string>       // for string
#include <vector>
#include <llvm/Support/CommandLine.h>
#include <boost/filesystem.hpp>

namespace re {class RE;}

namespace argv {

enum DevDirAction {Read, Skip, Recurse};
extern DevDirAction DevicesFlag;
extern DevDirAction DirectoriesFlag;

    
    // Use DirectoriesFlag==Recurse to test for recursive mode.
extern bool RecursiveFlag; 
extern bool DereferenceRecursiveFlag; // -R

extern bool MmapFlag; // -mmap

extern std::string ExcludeFlag; // -exclude
extern std::string ExcludeFromFlag; // -exclude-from
extern std::string ExcludeDirFlag; // -exclude-dir
extern std::string IncludeFlag; // -include

// File exclude pattern parsed from ExcludeFlag and ExcludeFromFlag
re::RE * getFileExcludePattern();

// File exclude pattern parsed from ExcludeDirFlag.
re::RE * getDirectoryExcludePattern();

// File include pattern parsed from InludeFlag
re::RE * getFileIncludePattern();

// Determine whether include is the default for files that do not match any pattern.
bool includeIsDefault();

extern bool UseStdIn;
    
std::vector<boost::filesystem::path> getFullFileList(llvm::cl::list<std::string> & inputFiles);
}
#endif
