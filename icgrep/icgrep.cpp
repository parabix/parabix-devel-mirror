/*
 *  Copyright (c) 2016 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#include <cstdio>

#include <llvm/Support/CommandLine.h>
#include <re/re_alt.h>
#include <re/re_parser.h>
#include <grep_engine.h>
#include <fstream>
#include <string>

#include <boost/uuid/sha1.hpp>
#include <toolchain.h>
#include <atomic>

static cl::OptionCategory aRegexSourceOptions("Regular Expression Options",
                                       "These options control the regular expression source.");

// static cl::OptionCategory bGrepOutputOptions("Output Options",
//                                       "These options control the output.");

static cl::list<std::string> inputFiles(cl::Positional, cl::desc("<regex> <input file ...>"), cl::OneOrMore);

// static cl::opt<bool> CountOnly("c", cl::desc("Count and display the matching lines per file only."), cl::cat(bGrepOutputOptions));
// static cl::alias CountOnlyLong("count", cl::desc("Alias for -c"), cl::aliasopt(CountOnly));
// static cl::opt<bool> NormalizeLineBreaks("normalize-line-breaks", cl::desc("Normalize line breaks to std::endl."), cl::init(false),  cl::cat(bGrepOutputOptions));

// static cl::opt<bool> ShowFileNames("H", cl::desc("Show the file name with each matching line."), cl::cat(bGrepOutputOptions));
// static cl::alias ShowFileNamesLong("with-filename", cl::desc("Alias for -H"), cl::aliasopt(ShowFileNames));

static cl::opt<bool> CaseInsensitive("i", cl::desc("Ignore case distinctions in the pattern and the file."), cl::cat(aRegexSourceOptions));
// static cl::opt<bool> ShowLineNumbers("n", cl::desc("Show the line number with each matching line."), cl::cat(bGrepOutputOptions));
// static cl::alias ShowLineNumbersLong("line-number", cl::desc("Alias for -n"), cl::aliasopt(ShowLineNumbers));

static cl::list<std::string> regexVector("e", cl::desc("Regular expression"), cl::ZeroOrMore, cl::cat(aRegexSourceOptions));
static cl::opt<std::string> RegexFilename("f", cl::desc("Take regular expressions (one per line) from a file"), cl::value_desc("regex file"), cl::init(""), cl::cat(aRegexSourceOptions));
static cl::opt<std::string> IRFileName("precompiled", cl::desc("Use precompiled regular expression"), cl::value_desc("LLVM IR file"), cl::init(""), cl::cat(aRegexSourceOptions));

static cl::opt<int> Threads("t", cl::desc("Total number of threads."), cl::init(1));



static unsigned firstInputFile = 1;  // Normal case when first positional arg is a regex.
static std::string allREs;
static re::ModeFlagSet globalFlags = 0;

re::RE * get_icgrep_RE() {
  
    //std::vector<std::string> regexVector;
    if (RegexFilename != "") {
        std::ifstream regexFile(RegexFilename.c_str());
        std::string r;
        if (regexFile.is_open()) {
            while (std::getline(regexFile, r)) {
                regexVector.push_back(r);
            }
            regexFile.close();
        }
    }
    
    // if there are no regexes specified through -e or -f, the first positional argument
    // must be a regex, not an input file.
    
    if (regexVector.size() == 0) {
        regexVector.push_back(inputFiles[0]);
        firstInputFile = 1;
    }
    else {
        firstInputFile = 0;
    }
    
    if (CaseInsensitive) globalFlags |= re::CASE_INSENSITIVE_MODE_FLAG;

  
    std::vector<re::RE *> REs;
    re::RE * re_ast = nullptr;
    for (unsigned i = 0; i < regexVector.size(); i++) {
        re_ast = re::RE_Parser::parse(regexVector[i], globalFlags);
        REs.push_back(re_ast);
        allREs += regexVector[i] + "\n";
    }
    if (REs.size() > 1) {
        re_ast = re::makeAlt(REs.begin(), REs.end());
    }
    
    return re_ast;
}

std::string sha1sum(const std::string & str) {
    char buffer[41];    // 40 hex-digits and the terminating null
    unsigned int digest[5];     // 160 bits in total

    boost::uuids::detail::sha1 sha1;
    sha1.process_bytes(str.c_str(), str.size());
    sha1.get_digest(digest);
    snprintf(buffer, sizeof(buffer), "%.8x%.8x%.8x%.8x%.8x",
             digest[0], digest[1], digest[2], digest[3], digest[4]);
    return std::string(buffer);
}

GrepEngine grepEngine;
std::atomic<int> fileCount;

void *DoGrep(void *threadid)
{
    long tid;
    tid = (long)threadid;
    if (tid+1 < inputFiles.size())
        grepEngine.doGrep(inputFiles[tid+1]);

    pthread_exit(NULL);
}

int main(int argc, char *argv[]) {
    StringMap<cl::Option*> Map;
    cl::getRegisteredOptions(Map);
    Map["time-passes"]->setHiddenFlag(cl::Hidden);
    Map["disable-spill-fusing"]->setHiddenFlag(cl::Hidden);
    Map["enable-misched"]->setHiddenFlag(cl::Hidden);
    Map["enable-tbaa"]->setHiddenFlag(cl::Hidden);
    Map["exhaustive-register-search"]->setHiddenFlag(cl::Hidden);
    Map["join-liveintervals"]->setHiddenFlag(cl::Hidden);
    Map["limit-float-precision"]->setHiddenFlag(cl::Hidden);
    Map["mc-x86-disable-arith-relaxation"]->setHiddenFlag(cl::Hidden);
    Map["limit-float-precision"]->setHiddenFlag(cl::Hidden);
    Map["print-after-all"]->setHiddenFlag(cl::Hidden);
    Map["print-before-all"]->setHiddenFlag(cl::Hidden);
    Map["print-machineinstrs"]->setHiddenFlag(cl::Hidden);
    Map["regalloc"]->setHiddenFlag(cl::Hidden);
    Map["rng-seed"]->setHiddenFlag(cl::Hidden);
    Map["stackmap-version"]->setHiddenFlag(cl::Hidden);
    Map["x86-asm-syntax"]->setHiddenFlag(cl::Hidden);
    Map["verify-debug-info"]->setHiddenFlag(cl::Hidden);
    Map["verify-dom-info"]->setHiddenFlag(cl::Hidden);
    Map["verify-loop-info"]->setHiddenFlag(cl::Hidden);
    Map["verify-regalloc"]->setHiddenFlag(cl::Hidden);
    Map["verify-scev"]->setHiddenFlag(cl::Hidden);
    Map["x86-recip-refinement-steps"]->setHiddenFlag(cl::Hidden);
    Map["rewrite-map-file"]->setHiddenFlag(cl::Hidden);

    cl::ParseCommandLineOptions(argc, argv);
    
    re::RE * re_ast = get_icgrep_RE();
    std::string module_name = "grepcode:" + sha1sum(allREs) + ":" + std::to_string(globalFlags);
    
    grepEngine.grepCodeGen(module_name, re_ast);

    initResult(inputFiles, inputFiles.size());
    if (Threads <= 1) {
        for (unsigned i = firstInputFile; i != inputFiles.size(); ++i) {
            grepEngine.doGrep(inputFiles[i]);
        }        
    } else if (Threads > 1) {
        const unsigned numOfThreads = Threads; // <- convert the command line value into an integer to allow stack allocation
        pthread_t threads[numOfThreads];

        for(unsigned long i = 0; i < numOfThreads; ++i){
            const int rc = pthread_create(&threads[i], NULL, DoGrep, (void *)i);
            if (rc) {
                throw std::runtime_error("Failed to create thread: code " + std::to_string(rc));
            }
        }

        for(unsigned i = 0; i < numOfThreads; ++i) {
            void * status = nullptr;
            const int rc = pthread_join(threads[i], &status);
            if (rc) {
                throw std::runtime_error("Failed to join thread: code " + std::to_string(rc));
            }
        }
    }
    PrintResult();   
    
    return 0;
}
