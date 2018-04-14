/*
 *  Copyright (c) 2014-8 International Characters.
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
#include <re/parsers/parser.h>
#include <re/re_utility.h>
#include <grep/grep_engine.h>
#include <grep_interface.h>
#include <fstream>
#include <string>
#include <toolchain/toolchain.h>
#include <re/re_toolchain.h>
#include <pablo/pablo_toolchain.h>
#include <boost/filesystem.hpp>
#include <util/file_select.h>
#include <sys/stat.h>
#include <fcntl.h>

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
        codegen::SegmentPipelineParallel = true;
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

    argv::InitializeCommandLineInterface(argc, argv);
    
    auto REs = readExpressions();

    std::vector<fs::path> allFiles = argv::getFullFileList(inputFiles);
    if (inputFiles.empty()) {
        argv::UseStdIn = true;
    }
    else if ((allFiles.size() > 1) && !argv::NoFilenameFlag) {
        argv::WithFilenameFlag = true;
    }

    grep::GrepEngine * grepEngine = nullptr;
    
    switch (argv::Mode) {
        case argv::NormalMode:
            grepEngine = new grep::EmitMatchesEngine();
            if (argv::MaxCountFlag) grepEngine->setMaxCount(argv::MaxCountFlag);
            if (argv::WithFilenameFlag) grepEngine->showFileNames();
            if (argv::LineNumberFlag) grepEngine->showLineNumbers();
            if (argv::InitialTabFlag) grepEngine->setInitialTab();
           break;
        case argv::CountOnly:
            grepEngine = new grep::CountOnlyEngine();
            if (argv::WithFilenameFlag) grepEngine->showFileNames();
            if (argv::MaxCountFlag) grepEngine->setMaxCount(argv::MaxCountFlag);
           break;
        case argv::FilesWithMatch:
        case argv::FilesWithoutMatch:
            grepEngine = new grep::MatchOnlyEngine(argv::Mode == argv::FilesWithMatch, argv::NullFlag);
            break;
        case argv::QuietMode:
            grepEngine = new grep::QuietModeEngine(); break;
        default: llvm_unreachable("Invalid grep mode!");
    }
    if (argv::IgnoreCaseFlag) grepEngine->setCaseInsensitive();
    if (argv::InvertMatchFlag) grepEngine->setInvertMatches();
    if (argv::UnicodeLinesFlag) {
        grepEngine->setRecordBreak(grep::GrepRecordBreakKind::Unicode);
    } else if (argv::NullDataFlag) {
        grepEngine->setRecordBreak(grep::GrepRecordBreakKind::Null);
    } else {
        grepEngine->setRecordBreak(grep::GrepRecordBreakKind::LF);
    }
    grepEngine->setStdinLabel(argv::LabelFlag);
    if (argv::UseStdIn) grepEngine->setGrepStdIn();
    if (argv::NoMessagesFlag) grepEngine->suppressFileMessages();
    if (argv::MmapFlag) grepEngine->setPreferMMap();
    grepEngine->initREs(REs);
    grepEngine->grepCodeGen();
    grepEngine->initFileResult(allFiles);
    bool matchFound = grepEngine->searchAllFiles();
    delete(grepEngine);
    
    return matchFound ? argv::MatchFoundExitCode : argv::MatchNotFoundExitCode;
}
