/*
 *  Copyright (c) 2014-8 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#include <cstdio>
#include <vector>
#include <llvm/Support/CommandLine.h>
#include <llvm/Support/ErrorHandling.h>
#include <llvm/Support/PrettyStackTrace.h>
#include <llvm/Support/Signals.h>
#include <llvm/Support/ManagedStatic.h>
#include <llvm/Support/raw_ostream.h>
#include <re/adt/re_alt.h>
#include <re/adt/re_seq.h>
#include <re/adt/re_start.h>
#include <re/adt/re_end.h>
#include <re/adt/re_utility.h>
#include <re/parse/parser.h>
#include <re/toolchain/toolchain.h>
#include <grep/grep_engine.h>
#include "grep_interface.h"
#include <fstream>
#include <string>
#include <toolchain/toolchain.h>
#include <pablo/pablo_toolchain.h>
#include <boost/filesystem.hpp>
#include <fileselect/file_select.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <llvm/ADT/STLExtras.h> // for make_unique
#include <kernel/pipeline/driver/cpudriver.h>

using namespace llvm;

static cl::list<std::string> inputFiles(cl::Positional, cl::desc("<regex> <input file ...>"), cl::OneOrMore);

static cl::opt<bool> ByteMode("enable-byte-mode", cl::desc("Process regular expressions in byte mode"));

static cl::opt<int> REsPerGroup("re-num", cl::desc("Number of regular expressions processed by each kernel."), cl::init(0));

static re::ModeFlagSet globalFlags = re::MULTILINE_MODE_FLAG;

std::vector<re::RE *> readExpressions() {

    if (argv::FileFlag != "") {
        std::ifstream regexFile(argv::FileFlag.c_str());
        std::string r;
        if (regexFile.is_open()) {
            while (std::getline(regexFile, r)) {
                argv::RegexpVector.push_back(r);
            }
            regexFile.close();
        }
    }

    // if there are no regexes specified through -e or -f, the first positional argument
    // must be a regex, not an input file.

    if (argv::RegexpVector.size() == 0) {
        argv::RegexpVector.push_back(inputFiles[0]);
        inputFiles.erase(inputFiles.begin());
    }
    if (argv::IgnoreCaseFlag) {
        globalFlags |= re::CASE_INSENSITIVE_MODE_FLAG;
    }

    std::vector<re::RE *> REs;
    for (unsigned i = 0; i < argv::RegexpVector.size(); i++) {
        re::RE * re_ast = re::RE_Parser::parse(argv::RegexpVector[i], globalFlags, argv::RegexpSyntax, ByteMode);
        REs.push_back(re_ast);
    }


    // If there are multiple REs, combine them into groups.
    // A separate kernel will be created for each group.
    if (REs.size() > 1) {
        if (REsPerGroup == 0) {
            // If no grouping factor is specified, we use a default formula.
            REsPerGroup = (REs.size() + codegen::ThreadNum) / (codegen::ThreadNum + 1);
        }
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
    }
    for (re::RE *& re_ast : REs) {
        assert (re_ast);
        if (argv::WordRegexpFlag) {
            re_ast = re::makeSeq({re::makeWordBoundary(), re_ast, re::makeWordBoundary()});
        }
        if (argv::LineRegexpFlag) {
            re_ast = re::makeSeq({re::makeStart(), re_ast, re::makeEnd()});
        }
    }

    return REs;
}

namespace fs = boost::filesystem;

int main(int argc, char *argv[]) {
    llvm_shutdown_obj shutdown;
    argv::InitializeCommandLineInterface(argc, argv);
    CPUDriver driver("icgrep");

    auto REs = readExpressions();

    const auto allFiles = argv::getFullFileList(driver, inputFiles);
    if (inputFiles.empty()) {
        argv::UseStdIn = true;
    } else if ((allFiles.size() > 1) && !argv::NoFilenameFlag) {
        argv::WithFilenameFlag = true;
    }

    std::unique_ptr<grep::GrepEngine> grep;
    switch (argv::Mode) {
        case argv::NormalMode:
            grep = make_unique<grep::EmitMatchesEngine>(driver);
            if (argv::MaxCountFlag) grep->setMaxCount(argv::MaxCountFlag);
            if (argv::WithFilenameFlag) grep->showFileNames();
            if (argv::LineNumberFlag) grep->showLineNumbers();
            if (argv::InitialTabFlag) grep->setInitialTab();
           break;
        case argv::CountOnly:
            grep = make_unique<grep::CountOnlyEngine>(driver);
            if (argv::WithFilenameFlag) grep->showFileNames();
            if (argv::MaxCountFlag) grep->setMaxCount(argv::MaxCountFlag);
           break;
        case argv::FilesWithMatch:
        case argv::FilesWithoutMatch:
            grep = make_unique<grep::MatchOnlyEngine>(driver, argv::Mode == argv::FilesWithMatch, argv::NullFlag);
            break;
        case argv::QuietMode:
            grep = make_unique<grep::QuietModeEngine>(driver);
            break;
        default: llvm_unreachable("Invalid grep mode!");
    }
    if (argv::IgnoreCaseFlag) grep->setCaseInsensitive();
    if (argv::InvertMatchFlag) grep->setInvertMatches();
    if (argv::UnicodeLinesFlag) {
        grep->setRecordBreak(grep::GrepRecordBreakKind::Unicode);
    } else if (argv::NullDataFlag) {
        grep->setRecordBreak(grep::GrepRecordBreakKind::Null);
    } else {
        grep->setRecordBreak(grep::GrepRecordBreakKind::LF);
    }
    grep->setContextLines(argv::BeforeContext, argv::AfterContext);

    grep->setStdinLabel(argv::LabelFlag);
    if (argv::UseStdIn) grep->setGrepStdIn();
    if (argv::NoMessagesFlag) grep->suppressFileMessages();
    if (argv::MmapFlag) grep->setPreferMMap();
    grep->setBinaryFilesOption(argv::BinaryFilesFlag);
    grep->initREs(REs);
    grep->grepCodeGen();
    grep->initFileResult(allFiles); // unnecessary copy!
    const bool matchFound = grep->searchAllFiles();

    return matchFound ? argv::MatchFoundExitCode : argv::MatchNotFoundExitCode;
}
