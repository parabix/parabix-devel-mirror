/*
 *  Copyright (c) 2016 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#include <cstdio>
#include <vector>
#include <llvm/Support/CommandLine.h>
#include <llvm/Support/ErrorHandling.h>
#include <llvm/Support/Signals.h>
#include <llvm/Support/raw_ostream.h>
#include <re/re_alt.h>
#include <re/re_seq.h>
#include <re/re_start.h>
#include <re/re_end.h>
#include <re/re_parser.h>
#include <re/re_utility.h>
#include <grep_engine.h>
#include <grep_interface.h>
#include <fstream>
#include <string>
#include <toolchain/toolchain.h>
#include <re/re_toolchain.h>
#include <pablo/pablo_toolchain.h>
#include <boost/filesystem.hpp>
#include <iostream> // MEEE
#ifdef PRINT_TIMING_INFORMATION
#include <hrtime.h>
#include <util/papi_helper.hpp>
#endif
#include <sys/stat.h>
#include <fcntl.h>

using namespace llvm;

static cl::opt<bool> UTF_16("UTF-16", cl::desc("Regular expressions over the UTF-16 representation of Unicode."));

static cl::list<std::string> inputFiles(cl::Positional, cl::desc("<regex> <input file ...>"), cl::OneOrMore);

static cl::opt<int> Threads("t", cl::desc("Total number of threads."), cl::init(1));

static cl::opt<bool> MultiGrepKernels("enable-multigrep-kernels", cl::desc("Construct separated kernels for each regular expression"));
static cl::opt<int> REsPerGroup("re-num", cl::desc("Number of regular expressions processed by each kernel."), cl::init(1));
static std::vector<std::string> allFiles;

static re::ModeFlagSet globalFlags = 0;

std::vector<re::RE *> readExpressions() {
  
    if (grep::FileFlag != "") {
        std::ifstream regexFile(grep::FileFlag.c_str());
        std::string r;
        if (regexFile.is_open()) {
            while (std::getline(regexFile, r)) {
                grep::RegexpVector.push_back(r);
            }
            regexFile.close();
        }
    }
    
    // if there are no regexes specified through -e or -f, the first positional argument
    // must be a regex, not an input file.
    
    if (grep::RegexpVector.size() == 0) {
        grep::RegexpVector.push_back(inputFiles[0]);
        inputFiles.erase(inputFiles.begin());
    }
    if (grep::IgnoreCaseFlag) {
        globalFlags |= re::CASE_INSENSITIVE_MODE_FLAG;
    }

    std::vector<re::RE *> REs;
    for (unsigned i = 0; i < grep::RegexpVector.size(); i++) {
        re::RE * re_ast = re::RE_Parser::parse(grep::RegexpVector[i], globalFlags, grep::RegexpSyntax);
        REs.push_back(re_ast);
    }

    if (MultiGrepKernels) {
        std::vector<re::RE *> groups;
        auto start = REs.begin();
        auto end = start + REsPerGroup;
        while (end < REs.end()) {
            groups.push_back(re::makeAlt(start, end));
            start = end;
            end += REsPerGroup;
        }
        if ((REs.end() - start) > 1) {
            groups.push_back(re::makeAlt(start, REs.end()));
        } else {
            groups.push_back(*start);
        }
        REs.swap(groups);
    } else if (REs.size() > 1) {
        re::RE * re_ast = re::makeAlt(REs.begin(), REs.end());
        REs.assign({re_ast});
    }

    for (re::RE *& re_ast : REs) {
        if (grep::WordRegexpFlag) {
            re_ast = re::makeSeq({re::makeWordBoundary(), re_ast, re::makeWordBoundary()});
        }
        if (grep::LineRegexpFlag) {
            re_ast = re::makeSeq({re::makeStart(), re_ast, re::makeEnd()});
        }
    }

    return REs;
}


// This is a stub, to be expanded later.
bool excludeDirectory(boost::filesystem::path dirpath) { return dirpath.filename() == ".svn";}

// Determine whether to skip a path based on -D skip or -d skip settings.
bool skip_path(boost::filesystem::path p) {
    using namespace boost::filesystem;
    switch (status(p).type()) {
        case directory_file: return grep::DirectoriesFlag == grep::Skip;
        case block_file:
        case character_file:
        case fifo_file:
        case socket_file:
            return grep::DevicesFlag == grep::Skip;
        default:
            return false;
    }
}

std::vector<std::string> getFullFileList(cl::list<std::string> & inputFiles) {
    using namespace boost::filesystem;
    symlink_option follow_symlink = grep::DereferenceRecursiveFlag ? symlink_option::recurse : symlink_option::none;
    std::vector<std::string> expanded_paths;
    boost::system::error_code errc;
    for (const std::string & f : inputFiles) {
        //        if (f == "-") {
        //            continue;
        //        }
        path p(f);
        if (skip_path(p)) {
            continue;
        }
        if (LLVM_UNLIKELY((grep::DirectoriesFlag == grep::Recurse) && is_directory(p))) {
            if (!excludeDirectory(p)) {
                recursive_directory_iterator di(p, follow_symlink, errc), end;
                if (errc) {
                    // If we cannot enter the directory, keep it in the list of files.
                    expanded_paths.push_back(f);
                    continue;
                }
                while (di != end) {
                    auto & e = di->path();
                    if (is_directory(e)) {
                        if (LLVM_UNLIKELY(excludeDirectory(e))) {
                            di.no_push();
                        }
                    } else {
                        if (!skip_path(e)) expanded_paths.push_back(e.string());
                    }
                    di.increment(errc);
                    if (errc) {
                        expanded_paths.push_back(e.string());
                    }
                }
            }
        } else {
            expanded_paths.push_back(p.string());
        }
    }
    return expanded_paths;
}

int main(int argc, char *argv[]) {

    grep::InitializeCommandLineInterface(argc, argv);
    
    const auto REs = readExpressions();

    allFiles = getFullFileList(inputFiles);

    grep::GrepEngine grepEngine;

    if (allFiles.empty()) {

        grepEngine.grepCodeGen(REs, grep::Mode, UTF_16, GrepSource::StdIn);
        allFiles = { "-" };
        grep::initFileResult(allFiles);
        grepEngine.doGrep(STDIN_FILENO, 0);

    } else {
        
        setNVPTXOption();
        
        if (codegen::NVPTX) {
            grepEngine.grepCodeGen_nvptx(REs, grep::Mode, UTF_16);
            for (unsigned i = 0; i != allFiles.size(); ++i) {
                grepEngine.doGrep(allFiles[i]);
            }
            return 0;
        } else {
            grepEngine.grepCodeGen(REs, grep::Mode, UTF_16, GrepSource::File);
        }

        grep::initFileResult(allFiles);

        if (Threads <= 1) {
            for (unsigned i = 0; i != allFiles.size(); ++i) {
                grepEngine.doGrep(allFiles[i], i);
            }
        } else if (Threads > 1) {
            const unsigned numOfThreads = Threads; // <- convert the command line value into an integer to allow stack allocation
            pthread_t threads[numOfThreads];

            for(unsigned long i = 0; i < numOfThreads; ++i){
                const int rc = pthread_create(&threads[i], nullptr, grep::DoGrepThreadFunction, (void *)&grepEngine);
                if (rc) {
                    llvm::report_fatal_error("Failed to create thread: code " + std::to_string(rc));
                }
            }
            for(unsigned i = 0; i < numOfThreads; ++i) {
                void * status = nullptr;
                const int rc = pthread_join(threads[i], &status);
                if (rc) {
                    llvm::report_fatal_error("Failed to join thread: code " + std::to_string(rc));
                }
            }
        }

    }
    
    grep::PrintResults();
    
    return 0;
}
