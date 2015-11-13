/*
 *  Copyright (c) 2015 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#include <string>
#include <iostream>
#include <fstream>

#include "basis_bits.h"
#include "toolchain.h"
#include "utf_encoding.h"
#include "pablo/pablo_compiler.h"
#include <llvm/IR/Function.h>
#include <llvm/IR/Type.h>
#include <llvm/IR/Module.h>
#include <llvm/ExecutionEngine/ExecutionEngine.h>
#include <llvm/ExecutionEngine/MCJIT.h>
#include <llvm/IRReader/IRReader.h>
#include <llvm/Support/SourceMgr.h>
#include <llvm/Support/CommandLine.h>
#include <llvm/Support/TargetSelect.h>
#include <llvm/Support/Host.h>

#include <re/re_re.h>
#include <re/parsefailure.h>
#include <re/re_parser.h>
#include <re/re_any.h>
#include <re/re_alt.h>
#include <pablo/function.h>

#include "do_grep.h"

static cl::OptionCategory aRegexSourceOptions("Regular Expression Options",
                                       "These options control the regular expression source.");

static cl::OptionCategory bGrepOutputOptions("Output Options",
                                      "These options control the output.");

static cl::list<std::string> inputFiles(cl::Positional, cl::desc("<regex> <input file ...>"), cl::OneOrMore);

static cl::opt<bool> CountOnly("c", cl::desc("Count and display the matching lines per file only."), cl::cat(bGrepOutputOptions));
static cl::alias CountOnlyLong("count", cl::desc("Alias for -c"), cl::aliasopt(CountOnly));
static cl::opt<bool> NormalizeLineBreaks("normalize-line-breaks", cl::desc("Normalize line breaks to std::endl."), cl::init(false),  cl::cat(bGrepOutputOptions));

static cl::opt<bool> ShowFileNames("H", cl::desc("Show the file name with each matching line."), cl::cat(bGrepOutputOptions));
static cl::alias ShowFileNamesLong("with-filename", cl::desc("Alias for -H"), cl::aliasopt(ShowFileNames));

static cl::opt<bool> CaseInsensitive("i", cl::desc("Ignore case distinctions in the pattern and the file."), cl::cat(aRegexSourceOptions));
static cl::opt<bool> ShowLineNumbers("n", cl::desc("Show the line number with each matching line."), cl::cat(bGrepOutputOptions));
static cl::alias ShowLineNumbersLong("line-number", cl::desc("Alias for -n"), cl::aliasopt(ShowLineNumbers));

static cl::list<std::string> regexVector("e", cl::desc("Regular expression"), cl::ZeroOrMore, cl::cat(aRegexSourceOptions));
static cl::opt<std::string> RegexFilename("f", cl::desc("Take regular expressions (one per line) from a file"), cl::value_desc("regex file"), cl::init(""), cl::cat(aRegexSourceOptions));
static cl::opt<std::string> IRFileName("precompiled", cl::desc("Use precompiled regular expression"), cl::value_desc("LLVM IR file"), cl::init(""), cl::cat(aRegexSourceOptions));



static unsigned firstInputFile = 1;  // Normal case when first positional arg is a regex.

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
    
    re::ModeFlagSet globalFlags = 0;
    if (CaseInsensitive) globalFlags |= re::CASE_INSENSITIVE_MODE_FLAG;

  
    std::vector<re::RE *> REs;
    re::RE * re_ast = nullptr;
    for (unsigned i = 0; i < regexVector.size(); i++) {
//        try
//        {
//            re_ast = re::RE_Parser::parse(regexVector[i], globalFlags);
//        }
//        catch (ParseFailure failure)
//        {
//            std::cerr << "Regex parsing failure: " << failure.what() << std::endl;
//            std::cerr << regexVector[i] << std::endl;
//            exit(1);
//        }
//        catch (UCD::UnicodePropertyExpressionError e)
//        {
//            std::cerr << "Unicode error: " << e.what() << std::endl;
//            std::cerr << regexVector[i] << std::endl;
//            exit(1);
//        }
        re_ast = re::RE_Parser::parse(regexVector[i], globalFlags);
        REs.push_back(re_ast);
    }
    if (REs.size() > 1) {
        re_ast = re::makeAlt(REs.begin(), REs.end());
    }
    
    return re_ast;
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
    
    Encoding encoding(Encoding::Type::UTF_8, 8);

    llvm::Function * icgrep_IR = nullptr;
    
    if (IRFileName == "") {        
        re::RE * re_ast = get_icgrep_RE();
        re_ast = regular_expression_passes(encoding, re_ast);
        
        pablo::PabloFunction * function = re2pablo_compiler(encoding, re_ast);

        pablo_function_passes(function);
        pablo::PabloCompiler pablo_compiler(VectorType::get(IntegerType::get(getGlobalContext(), 64), BLOCK_SIZE/64));
        try {
            icgrep_IR = pablo_compiler.compile(function);
            delete function;
            releaseSlabAllocatorMemory();
        } catch (std::runtime_error e) {
            delete function;
            releaseSlabAllocatorMemory();
            std::cerr << "Runtime error: " << e.what() << std::endl;
            exit(1);
        }
    } else {
        firstInputFile = 0;  // No regexp arguments; first positional argument is a file to process.
        SMDiagnostic ParseErr;
        Module * M = parseIRFile(IRFileName, ParseErr, getGlobalContext()).release();
        if (!M) {
            throw std::runtime_error("Error in Parsing IR File " + IRFileName);
        }
        icgrep_IR = M->getFunction("process_block");
    }
    
    llvm::ExecutionEngine * engine = JIT_to_ExecutionEngine(icgrep_IR);
    
    icgrep_Linking(icgrep_IR->getParent(), engine);
    
    // Ensure everything is ready to go.
    engine->finalizeObject();
    
    // TODO getPointerToFunction() is deprecated. Investigate getFunctionAddress(string name) instead.
    void * icgrep_init_carry_ptr = engine->getPointerToFunction(icgrep_IR->getParent()->getFunction("process_block_initialize_carries"));
    void * icgrep_MCptr = engine->getPointerToFunction(icgrep_IR);
    
    if (icgrep_MCptr) {
        GrepExecutor grepEngine(icgrep_init_carry_ptr, icgrep_MCptr);
        grepEngine.setCountOnlyOption(CountOnly);
        grepEngine.setNormalizeLineBreaksOption(NormalizeLineBreaks);
        grepEngine.setShowLineNumberOption(ShowLineNumbers);
        if (inputFiles.size() > (firstInputFile + 1) || ShowFileNames) {
            grepEngine.setShowFileNameOption();
        }
        for (unsigned i = firstInputFile; i != inputFiles.size(); ++i) {
            grepEngine.doGrep(inputFiles[i]);
        }
    }
    
    //engine->freeMachineCodeForFunction(icgrep_IR); // Removed in LLVM 3.6. MC will be automatically freed in destructors.
    delete engine;

    return 0;
}
