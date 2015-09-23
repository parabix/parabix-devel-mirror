/*
 *  Copyright (c) 2015 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#include <string>
#include <iostream>
#include <fstream>

#include "icgrep.h"
#include "utf_encoding.h"
#include "pablo/pablo_compiler.h"
#include <llvm/IR/Function.h>
#include <llvm/IR/Module.h>
#include <llvm/ExecutionEngine/ExecutionEngine.h>
#include <llvm/ExecutionEngine/MCJIT.h>
#include <llvm/IRReader/IRReader.h>
#include <llvm/Support/SourceMgr.h>
#include <llvm/Support/CommandLine.h>
#include <llvm/Support/TargetSelect.h>
#include <llvm/Support/Host.h>

#ifndef DISABLE_PREGENERATED_UCD_FUNCTIONS
#include <UCD/precompiled_properties.h>
#endif
#include <re/re_cc.h>
#include <re/re_nullable.h>
#include <re/re_simplifier.h>
#include <re/re_alt.h>
#include <re/parsefailure.h>
#include <re/re_parser.h>
#include <re/re_compiler.h>
#include <utf8_encoder.h>
#include <cc/cc_compiler.h>
#include <cc/cc_namemap.hpp>
#include <pablo/pablo_compiler.h>
#include <pablo/optimizers/pablo_simplifier.hpp>
#include <pablo/optimizers/pablo_codesinking.hpp>
#ifdef ENABLE_MULTIPLEXING
#include <pablo/optimizers/pablo_automultiplexing.hpp>
#include <pablo/optimizers/pablo_bddminimization.h>
#include <pablo/optimizers/booleanreassociationpass.h>
#endif
#include <pablo/function.h>
#include <re/printer_re.h>
#include <pablo/printer_pablos.h>

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

static cl::OptionCategory cRegexOutputOptions("Regex Dump Options",
                                      "These options control printing of intermediate regular expression structures.");

static cl::OptionCategory dPabloDumpOptions("Pablo Dump Options",
                                      "These options control printing of intermediate Pablo code.");

static cl::opt<bool> PrintAllREs("print-REs", cl::init(false), cl::desc("print regular expression passes"), cl::cat(cRegexOutputOptions));
static cl::opt<bool> PrintParsedREs("print-parsed-REs", cl::init(false), cl::desc("print out parsed regular expressions"), cl::cat(cRegexOutputOptions));
static cl::opt<bool> PrintStrippedREs("print-stripped-REs", cl::init(false), cl::desc("print out REs with nullable prefixes/suffixes removed"), cl::cat(cRegexOutputOptions));
static cl::opt<bool> PrintNamedREs("print-named-REs", cl::init(false), cl::desc("print out named REs"), cl::cat(cRegexOutputOptions));
static cl::opt<bool> PrintUTF8REs("print-utf8-REs", cl::init(false), cl::desc("print out UTF-8 REs"), cl::cat(cRegexOutputOptions));
static cl::opt<bool> PrintSimplifiedREs("print-simplified-REs", cl::init(false), cl::desc("print out final simplified REs"), cl::cat(cRegexOutputOptions));
static cl::opt<bool> PrintCompiledCCcode("print-CC-pablo", cl::init(false), cl::desc("print Pablo output from character class compiler"), cl::cat(dPabloDumpOptions));
static cl::opt<bool> PrintCompiledREcode("print-RE-pablo", cl::init(false), cl::desc("print Pablo output from the regular expression compiler"), cl::cat(dPabloDumpOptions));
static cl::opt<bool> PrintOptimizedREcode("print-pablo", cl::init(false), cl::desc("print final optimized Pablo code"), cl::cat(dPabloDumpOptions));

static cl::OptionCategory cPabloOptimizationsOptions("Pablo Optimizations", "These options control Pablo optimization passes.");

static cl::opt<bool> DisablePabloCSE("disable-CSE", cl::init(false),
                                      cl::desc("Disable Pablo common subexpression elimination/dead code elimination"),
                                      cl::cat(cPabloOptimizationsOptions));
static cl::opt<bool> PabloSinkingPass("sinking", cl::init(false),
                                      cl::desc("Moves all instructions into the innermost legal If-scope so that they are only executed when needed."),
                                      cl::cat(cPabloOptimizationsOptions));

#ifdef ENABLE_MULTIPLEXING
static cl::opt<bool> EnableMultiplexing("multiplexing", cl::init(false),
    cl::desc("combine Advances whose inputs are mutual exclusive into the fewest number of advances possible (expensive)."),
    cl::cat(cPabloOptimizationsOptions));

static cl::opt<bool> EnableReassociation("reassoc", cl::init(false),
    cl::desc("perform reassocation and distribution law optimization."),
    cl::cat(cPabloOptimizationsOptions));
#endif

static cl::opt<bool> UseAVX2("use-AVX2", cl::init(false), cl::desc("execute with AVX2 instruction set."), cl::cat(cRegexOutputOptions));

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
        try
        {
            re_ast = re::RE_Parser::parse(regexVector[i], globalFlags);
        }
        catch (ParseFailure failure)
        {
            std::cerr << "Regex parsing failure: " << failure.what() << std::endl;
            std::cerr << regexVector[i] << std::endl;
            exit(1);
        }
        catch (UCD::UnicodePropertyExpressionError e)
        {
            std::cerr << "Unicode error: " << e.what() << std::endl;
            std::cerr << regexVector[i] << std::endl;
            exit(1);
        }
        REs.push_back(re_ast);
    }
    if (REs.size() > 1) {
        re_ast = re::makeAlt(REs.begin(), REs.end());
    }
    
    if (PrintAllREs || PrintParsedREs) {
        std::cerr << "Parser:" << std::endl << Printer_RE::PrintRE(re_ast) << std::endl;
    }
    return re_ast;
}

re::RE * regular_expression_passes(const Encoding encoding, re::RE * re_ast)  {
    //Optimization passes to simplify the AST.
    re_ast = re::RE_Nullable::removeNullablePrefix(re_ast);
    if (PrintAllREs || PrintStrippedREs) {
        std::cerr << "RemoveNullablePrefix:" << std::endl << Printer_RE::PrintRE(re_ast) << std::endl;
    }
    re_ast = re::RE_Nullable::removeNullableSuffix(re_ast);
    if (PrintAllREs || PrintStrippedREs) {
        std::cerr << "RemoveNullableSuffix:" << std::endl << Printer_RE::PrintRE(re_ast) << std::endl;
    }
    
    cc::CC_NameMap nameMap;
    re_ast = nameMap.process(re_ast, re::UnicodeClass);
    
    // std::cerr << "-----------------------------" << std::endl;
    
    if (PrintAllREs || PrintNamedREs) {
        std::cerr << "Namer:" << std::endl << Printer_RE::PrintRE(re_ast) << std::endl;
        std::cerr << "NameMap:\n" << nameMap.printMap() << std::endl;
    }
    
    //Add the UTF encoding.
    if (encoding.getType() == Encoding::Type::UTF_8) {
        re_ast = cc::UTF8_Encoder::toUTF8(nameMap, re_ast);
        if (PrintAllREs || PrintUTF8REs) {
            //Print to the terminal the AST that was generated by the utf8 encoder.
            std::cerr << "UTF8-encoder:" << std::endl << Printer_RE::PrintRE(re_ast) << std::endl;
            std::cerr << "NameMap:\n" << nameMap.printMap() << std::endl;
        }
    }
    
    re_ast = re::RE_Simplifier::simplify(re_ast);
    if (PrintAllREs || PrintSimplifiedREs) {
        //Print to the terminal the AST that was generated by the simplifier.
        std::cerr << "Simplifier:" << std::endl << Printer_RE::PrintRE(re_ast) << std::endl;
    }
    return re_ast;
}
    
pablo::PabloFunction * re2pablo_compiler(const Encoding encoding, re::RE * re_ast) {
    
    pablo::PabloFunction * function = pablo::PabloFunction::Create("process_block", 8, 2);
    
    cc::CC_Compiler cc_compiler(*function, encoding);
    
    cc_compiler.compileByteClasses(re_ast);
    
    if (PrintCompiledCCcode) {
        //Print to the terminal the AST that was generated by the character class compiler.
        llvm::raw_os_ostream cerr(std::cerr);
        cerr << "CC AST:" << "\n";
        PabloPrinter::print(function->getEntryBlock().statements(), cerr);
    }
    
    re::RE_Compiler re_compiler(*function, cc_compiler);
    re_compiler.initializeRequiredStreams();
    re_compiler.finalizeMatchResult(re_compiler.compile(re_ast));

    if (PrintCompiledREcode) {
        //Print to the terminal the AST that was generated by the pararallel bit-stream compiler.
        llvm::raw_os_ostream cerr(std::cerr);
        cerr << "Initial Pablo AST:\n";
        PabloPrinter::print(function->getEntryBlock().statements(), cerr);
    }
    return function;
}

void pablo_function_passes(pablo::PabloFunction * function) {
    // Scan through the pablo code and perform DCE and CSE
    if (!DisablePabloCSE) {
        pablo::Simplifier::optimize(*function);
    }
    if (PabloSinkingPass) {
        pablo::CodeSinking::optimize(*function);
    }
#ifdef ENABLE_MULTIPLEXING    
    if (EnableMultiplexing) {
        pablo::BDDMinimizationPass::optimize(*function);
        pablo::AutoMultiplexing::optimize(*function);        
    }    
    if (EnableReassociation) {
        pablo::BooleanReassociationPass::optimize(*function);
    }
#endif
    if (PrintOptimizedREcode) {
        //Print to the terminal the AST that was generated by the pararallel bit-stream compiler.
        llvm::raw_os_ostream cerr(std::cerr);
        cerr << "Final Pablo AST:\n";
        PabloPrinter::print(function->getEntryBlock().statements(), cerr);
    }
}



ExecutionEngine * JIT_to_ExecutionEngine (llvm::Function * f) {

    InitializeNativeTarget();
    InitializeNativeTargetAsmPrinter();
    InitializeNativeTargetAsmParser();

    std::string errMessage;
    EngineBuilder builder(std::move(std::unique_ptr<Module>(f->getParent())));
    builder.setErrorStr(&errMessage);
    builder.setMCPU(sys::getHostCPUName());
    builder.setOptLevel(CodeGenOpt::Level::None);
#if (BLOCK_SIZE == 256)
    if(UseAVX2){
            std::vector<std::string> attrs;
            attrs.push_back("avx2");
            builder.setMAttrs(attrs);
    }
#endif
    //builder.setOptLevel(mMaxWhileDepth ? CodeGenOpt::Level::Less : CodeGenOpt::Level::None);
    ExecutionEngine * engine = builder.create();
    if (engine == nullptr) {
        throw std::runtime_error("Could not create ExecutionEngine: " + errMessage);
    }
    //engine->addGlobalMapping(cast<GlobalValue>(mPrintRegisterFunction), (void *)&wrapped_print_register);
    // engine->addGlobalMapping(externalFunction, proto->getFunctionPtr());

    return engine;
}


extern "C" {
  void wrapped_print_register(char * regName, BitBlock bit_block) {
      print_register<BitBlock>(regName, bit_block);
  }
}

void icgrep_Linking(Module * m, ExecutionEngine * e) {
    Module::FunctionListType & fns = m->getFunctionList();
    for (Module::FunctionListType::iterator it = fns.begin(), it_end = fns.end(); it != it_end; ++it) {
        std::string fnName = it->getName().str();
        if (fnName == "process_block") continue;
        if (fnName == "wrapped_print_register") {
            e->addGlobalMapping(cast<GlobalValue>(it), (void *)&wrapped_print_register);
        }
#ifndef DISABLE_PREGENERATED_UCD_FUNCTIONS
        else {
            const UCD::ExternalProperty & ep = UCD::resolveExternalProperty(fnName);
            e->addGlobalMapping(cast<GlobalValue>(it), std::get<0>(ep));
        }
#endif
    }
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
        pablo::PabloCompiler pablo_compiler;
        try {
            icgrep_IR = pablo_compiler.compile(function);
            releaseSlabAllocatorMemory();
        }
        catch (std::runtime_error e) {
            releaseSlabAllocatorMemory();
            std::cerr << "Runtime error: " << e.what() << std::endl;
            exit(1);
        }
    }
    else {
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
    
    void * icgrep_MCptr = engine->getPointerToFunction(icgrep_IR);
    
    if (icgrep_MCptr) {
        GrepExecutor grepEngine(icgrep_MCptr);
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
    
    //engine->freeMachineCodeForFunction(icgrep_IR); // This function only prints a "not supported" message. Reevaluate with LLVM 3.6.
    delete engine;

    return 0;
}

