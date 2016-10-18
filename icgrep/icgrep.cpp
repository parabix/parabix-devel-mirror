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
#include <re/re_alt.h>
#include <re/re_parser.h>
#include <grep_engine.h>
#include <fstream>
#include <string>

#include <boost/uuid/sha1.hpp>
#include <toolchain.h>
#include <re/re_toolchain.h>
#include <pablo/pablo_toolchain.h>
#include <mutex>
#include <boost/filesystem.hpp>

#include <iostream> // MEEE

#ifdef PRINT_TIMING_INFORMATION
#include <hrtime.h>
#include <util/papi_helper.hpp>
#endif

static cl::OptionCategory LegacyGrepOptions("A. Standard Grep Options",
                                       "These are standard grep options intended for compatibility with typical grep usage.");

#ifdef FUTURE
static cl::OptionCategory RegexpOptions("Regular Expression Interpretation", "These options control regular expression interpretation");
static cl::opt<re::RE_Syntax> RegexpSyntax(cl::desc("Regular expression syntax:"),
    cl::values(
        clEnumValN(re::RE_Syntax::FixedStrings, "F", "Fixed strings, separated by newlines"),
        clEnumValN(re::RE_Syntax::BRE, "G", "Posix basic regular expression (BRE) syntax"),
        clEnumValN(re::RE_Syntax::ERE, "E", "Posix extended regular expression (ERE) syntax"),
        clEnumValN(re::RE_Syntax::PCRE, "P", "Perl-compatible regular expression (PCRE) syntax - default"),
               clEnumValEnd), cl::cat(LegacyGrepOptions), cl::Grouping, cl::init(re::RE_Syntax::PCRE));
#endif

static cl::opt<bool> UTF_16("UTF-16", cl::desc("Regular expressions over the UTF-16 representation of Unicode."), cl::cat(LegacyGrepOptions));
static cl::OptionCategory EnhancedGrepOptions("B. Enhanced Grep Options",
                                       "These are additional options for icgrep functionality and performance.");
static cl::opt<bool> CountOnly("c", cl::desc("Count and display the matching lines per file only."), cl::cat(LegacyGrepOptions), cl::Grouping);
static cl::alias CountOnlyLong("count", cl::desc("Alias for -c"), cl::aliasopt(CountOnly));

static cl::list<std::string> inputFiles(cl::Positional, cl::desc("<regex> <input file ...>"), cl::OneOrMore);

static cl::opt<bool> EnterDirectoriesRecursively("r", cl::desc("Recursively process files within directories, (but follow only top-level symlinks unless -R)."), cl::cat(LegacyGrepOptions), cl::Grouping);
static cl::opt<bool> FollowSubdirectorySymlinks("R", cl::desc("Recursively process files within directories, following symlinks at all levels."), cl::cat(LegacyGrepOptions), cl::Grouping);
static cl::opt<bool> CaseInsensitive("i", cl::desc("Ignore case distinctions in the pattern and the file."), cl::cat(LegacyGrepOptions), cl::Grouping);


static cl::list<std::string> regexVector("e", cl::desc("Regular expression"), cl::ZeroOrMore, cl::cat(LegacyGrepOptions));
static cl::opt<std::string> RegexFilename("f", cl::desc("Take regular expressions (one per line) from a file"), cl::value_desc("regex file"), cl::init(""), cl::cat(LegacyGrepOptions));
static cl::opt<std::string> IRFileName("precompiled", cl::desc("Use precompiled regular expression"), cl::value_desc("LLVM IR file"), cl::init(""));

static cl::opt<int> Threads("t", cl::desc("Total number of threads."), cl::init(1));

static cl::opt<bool> GrepSupport("gs", cl::desc("Grep support. Pipe the output of icgrep into grep. \
         Gives you colored output + back-referencing capability."), cl::cat(EnhancedGrepOptions));


static std::vector<std::string> allFiles;
//
// Handler for errors reported through llvm::report_fatal_error.  Report
// and signal error code 2 (grep convention).
// 
static void icgrep_error_handler(void *UserData, const std::string &Message,
                             bool GenCrashDiag) {

    // Modified from LLVM's internal report_fatal_error logic.
    SmallVector<char, 64> Buffer;
    raw_svector_ostream OS(Buffer);
    OS << "icgrep ERROR: " << Message << "\n";
    StringRef MessageStr = OS.str();
    ssize_t written = ::write(2, MessageStr.data(), MessageStr.size());
    (void)written; // If something went wrong, we deliberately just give up.

    // Run the interrupt handlers to make sure any special cleanups get done, in
    // particular that we remove files registered with RemoveFileOnSignal.
    llvm::sys::RunInterruptHandlers();
    exit(2);
}

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
        inputFiles.erase(inputFiles.begin());
    }
    if (CaseInsensitive) globalFlags |= re::CASE_INSENSITIVE_MODE_FLAG;

  
    std::vector<re::RE *> REs;
    re::RE * re_ast = nullptr;
    for (unsigned i = 0; i < regexVector.size(); i++) {
#ifdef FUTURE
        re_ast = re::RE_Parser::parse(regexVector[i], globalFlags, RegexpSyntax);
#else
        re_ast = re::RE_Parser::parse(regexVector[i], globalFlags);
#endif
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

std::vector<size_t> total_CountOnly;
std::mutex count_mutex;
size_t fileCount;
void *DoGrep(void *args)
{
    size_t fileIdx;
    GrepEngine * grepEngine = (GrepEngine *)args;

    count_mutex.lock();
    fileIdx = fileCount;
    fileCount++;
    count_mutex.unlock();

    while (fileIdx < allFiles.size()){
        grepEngine->doGrep(allFiles[fileIdx], fileIdx, CountOnly, total_CountOnly, UTF_16);
        
        count_mutex.lock();
        fileIdx = fileCount;
        fileCount++;
        count_mutex.unlock();
    }

    pthread_exit(NULL);
}


// Returns true if the command line argument shouldn't be passed to icGrep or Grep.
bool isArgUnwantedForAll(char *argument) {
    std::vector<std::string> unwantedFlags = {"-gs"};
    for (unsigned i = 0; i < unwantedFlags.size(); ++i){
        if (strcmp(argument, unwantedFlags[i].c_str()) == 0) {
            return true;
        }
    }
    return false;
}
// Filters out the command line strings that shouldn't be passed on to Grep
bool isArgUnwantedForGrep(char *argument) {
#ifdef FUTURE
    std::vector<std::string> unwantedFlags = {"-n", "-P", "-G", "-E"};
#else
    std::vector<std::string> unwantedFlags = {"-n"};
#endif

    for (unsigned i = 0; i < unwantedFlags.size(); ++i){
        if (strcmp(argument, unwantedFlags[i].c_str()) == 0) {
            return true;
        }
    }

    for (unsigned i = 0; i < inputFiles.size(); ++i){    // filter out input content files.
        if (strcmp(argument, inputFiles[i].c_str()) == 0) {
            return true;
        }
    }

    return false;
}
// Filters out the command line strings that shouldn't be passed on to IcGrep
bool isArgUnwantedForIcGrep(char *argument) {
    bool isUnwanted = false;
    std::vector<std::string> unwantedFlags = {"-c"};

    for (unsigned i = 0; i < unwantedFlags.size(); ++i){
        if (strcmp(argument, unwantedFlags[i].c_str()) == 0) {
            isUnwanted = true;
        }
    }

    return isUnwanted;
}

/*
* Constructs a shell command that calls icgrep and then pipes the output to grep.
* Then executs this shell command using the "system()" function.
* This allows the output to be colored since all output is piped to grep.
*/ 
void pipeIcGrepOutputToGrep(int argc, char *argv[]) {
    std::string icGrepArguments = "";
    std::string grepArguments = "";

    // Construct the shell arguments for icgrep and grep 
    // by filtering out the command line arguments passed into this process.
    for (int i = 1; i < argc; i++) {
        if (!isArgUnwantedForAll(argv[i])) {

            if (!isArgUnwantedForIcGrep(argv[i])) {
                // Wrap everything in quotes since the arguments passed into this program had them stripped by bash.
                icGrepArguments.append("\"");       
                icGrepArguments.append(argv[i]);
                icGrepArguments.append("\" ");
            }

            if (!isArgUnwantedForGrep(argv[i])) {
                grepArguments.append("\"");
                grepArguments.append(argv[i]);
                grepArguments.append("\" ");
            }
        }
    }

#ifdef FUTURE
    switch (RegexpSyntax) {
        case re::RE_Syntax::BRE:
            grepArguments.append("\"-G\" ");
            break;
        case re::RE_Syntax::ERE:
            grepArguments.append("\"-E\" ");
            break;
        case re::RE_Syntax::PCRE:
            grepArguments.append("\"-P\" ");
            break;
        default:
            //TODO: handle fix string
            break;
    }
#endif

    std::string systemCall = argv[0];
    systemCall.append(" ");
    systemCall.append(icGrepArguments);
    systemCall.append(" ");
#ifdef FUTURE
    systemCall.append(" | grep --color=always ");
#else
    systemCall.append(" | grep --color=always -P ");
#endif
    systemCall.append(grepArguments);

    system(systemCall.c_str());
}


// This is a stub, to be expanded later.
bool excludeDirectory(boost::filesystem::path dirpath) { return dirpath.filename() == ".svn";}

std::vector<std::string> getFullFileList(cl::list<std::string> & inputFiles) {
    using namespace boost::filesystem;
    symlink_option follow_symlink = FollowSubdirectorySymlinks ? symlink_option::recurse : symlink_option::none;
    std::vector<std::string> expanded_paths;
    boost::system::error_code errc;
    if (FollowSubdirectorySymlinks) {
        EnterDirectoriesRecursively = true;
    }
    for (auto & f : inputFiles) {
        path p(f);
        if (EnterDirectoriesRecursively && is_directory(p)) {
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
                        if (excludeDirectory(e)) di.no_push();
                    }
                    else expanded_paths.push_back(e.string());
                    di.increment(errc);
                    if (errc) {
                        expanded_paths.push_back(e.string()); 
                    }
                }
            }
        }
        else expanded_paths.push_back(p.string());
    }
    return expanded_paths;
}


int main(int argc, char *argv[]) {
    llvm::install_fatal_error_handler(&icgrep_error_handler);
#ifndef USE_LLVM_3_6
    cl::HideUnrelatedOptions(ArrayRef<const cl::OptionCategory *>{&LegacyGrepOptions, &EnhancedGrepOptions, re::re_toolchain_flags(), pablo::pablo_toolchain_flags(), codegen::codegen_flags()});
#endif
    cl::ParseCommandLineOptions(argc, argv);
#ifdef FUTURE
    if (RegexpSyntax == re::RE_Syntax::FixedStrings) {
        llvm::report_fatal_error("Sorry, FixedStrings syntax is not fully supported\n.");
    }
#endif
    re::RE * re_ast = get_icgrep_RE();
    std::string module_name = "grepcode:" + sha1sum(allREs) + ":" + std::to_string(globalFlags);

    if (GrepSupport) {  // Calls icgrep again on command line and passes output to grep.
        pipeIcGrepOutputToGrep(argc, argv);
        return 0;   // icgrep is called again, so we need to end this process.
    }
    
    GrepEngine grepEngine;
    grepEngine.grepCodeGen(module_name, re_ast, CountOnly, UTF_16);
    //std::cerr << "grepCodeGen complete";

    releaseSlabAllocatorMemory();
    
    allFiles = getFullFileList(inputFiles);
    
    initResult(allFiles);
    for (unsigned i=0; i < allFiles.size(); ++i){
        total_CountOnly.push_back(0);
    }

    if (Threads <= 1) {

        #ifdef PRINT_TIMING_INFORMATION
        // PAPI_RES_STL, PAPI_STL_CCY, PAPI_FUL_CCY, PAPI_MEM_WCY
        // PAPI_RES_STL, PAPI_BR_MSP, PAPI_LST_INS, PAPI_L1_TCM
        papi::PapiCounter<4> papiCounters({PAPI_RES_STL, PAPI_STL_CCY, PAPI_FUL_CCY, PAPI_MEM_WCY});
        #endif
        for (unsigned i = 0; i != allFiles.size(); ++i) {
            #ifdef PRINT_TIMING_INFORMATION
            papiCounters.start();
            const timestamp_t execution_start = read_cycle_counter();
            #endif
            grepEngine.doGrep(allFiles[i], i, CountOnly, total_CountOnly, UTF_16);
            #ifdef PRINT_TIMING_INFORMATION
            const timestamp_t execution_end = read_cycle_counter();
            papiCounters.stop();
            std::cerr << "EXECUTION TIME: " << allFiles[i] << ":" << "CYCLES|" << (execution_end - execution_start) << papiCounters << std::endl;
            #endif
        }        
    } else if (Threads > 1) {
        const unsigned numOfThreads = Threads; // <- convert the command line value into an integer to allow stack allocation
        pthread_t threads[numOfThreads];

        for(unsigned long i = 0; i < numOfThreads; ++i){
            const int rc = pthread_create(&threads[i], NULL, DoGrep, (void *)&grepEngine);
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
    
    PrintResult(CountOnly, total_CountOnly);
    
    return 0;
}
