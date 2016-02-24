/*
 *  Copyright (c) 2015 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#include <string>
#include <iostream>
#include <fstream>

#include "utf_encoding.h"
#include "pablo/pablo_compiler.h"
#include <llvm/IR/Function.h>
#include <llvm/IR/Module.h>
#include <llvm/ExecutionEngine/ExecutionEngine.h>
#include <llvm/ExecutionEngine/MCJIT.h>
#include <llvm/IRReader/IRReader.h>
#include <llvm/Support/CommandLine.h>
#include <llvm/CodeGen/CommandFlags.h>
#include <llvm/Support/SourceMgr.h>
#include <llvm/Support/TargetSelect.h>
#include <llvm/Support/Host.h>

#include <IDISA/idisa_avx_builder.h>
#include <IDISA/idisa_sse_builder.h>
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
#include <pablo/pablo_compiler.h>
#include <pablo/optimizers/pablo_simplifier.hpp>
#include <pablo/optimizers/codemotionpass.h>
#include <pablo/passes/flattenassociativedfg.h>
#include <pablo/passes/factorizedfg.h>
#ifdef ENABLE_MULTIPLEXING
#include <pablo/optimizers/pablo_automultiplexing.hpp>
#include <pablo/optimizers/pablo_bddminimization.h>
#include <pablo/optimizers/distributivepass.h>
#include <pablo/optimizers/schedulingprepass.h>
#endif
#include <pablo/function.h>
#include <pablo/analysis/pabloverifier.hpp>
#include <re/printer_re.h>
#include <pablo/printer_pablos.h>

#include "do_grep.h"

using namespace pablo;

static cl::OptionCategory bGrepOutputOptions("Output Options",
                                      "These options control the output.");

static cl::opt<bool> CountOnly("c", cl::desc("Count and display the matching lines per file only."), cl::cat(bGrepOutputOptions));
static cl::alias CountOnlyLong("count", cl::desc("Alias for -c"), cl::aliasopt(CountOnly));
static cl::opt<bool> NormalizeLineBreaks("normalize-line-breaks", cl::desc("Normalize line breaks to std::endl."), cl::init(false),  cl::cat(bGrepOutputOptions));

static cl::opt<bool> ShowFileNames("H", cl::desc("Show the file name with each matching line."), cl::cat(bGrepOutputOptions));
static cl::alias ShowFileNamesLong("with-filename", cl::desc("Alias for -H"), cl::aliasopt(ShowFileNames));

static cl::opt<bool> ShowLineNumbers("n", cl::desc("Show the line number with each matching line."), cl::cat(bGrepOutputOptions));
static cl::alias ShowLineNumbersLong("line-number", cl::desc("Alias for -n"), cl::aliasopt(ShowLineNumbers));


static cl::OptionCategory cRegexOutputOptions("Regex Dump Options",
                                              "These options control printing of intermediate regular expression structures.");
static cl::opt<bool> PrintAllREs("print-REs", cl::init(false), cl::desc("print regular expression passes"), cl::cat(cRegexOutputOptions));
static cl::opt<bool> PrintParsedREs("print-parsed-REs", cl::init(false), cl::desc("print out parsed regular expressions"), cl::cat(cRegexOutputOptions));
static cl::opt<bool> PrintStrippedREs("print-stripped-REs", cl::init(false), cl::desc("print out REs with nullable prefixes/suffixes removed"), cl::cat(cRegexOutputOptions));
static cl::opt<bool> PrintNamedREs("print-named-REs", cl::init(false), cl::desc("print out named REs"), cl::cat(cRegexOutputOptions));
static cl::opt<bool> PrintUTF8REs("print-utf8-REs", cl::init(false), cl::desc("print out UTF-8 REs"), cl::cat(cRegexOutputOptions));
static cl::opt<bool> PrintSimplifiedREs("print-simplified-REs", cl::init(false), cl::desc("print out final simplified REs"), cl::cat(cRegexOutputOptions));
static cl::OptionCategory dPabloDumpOptions("Pablo Dump Options",
                                            "These options control printing of intermediate Pablo code.");

static cl::opt<bool> PrintOptimizedREcode("print-pablo", cl::init(false), cl::desc("print final optimized Pablo code"), cl::cat(dPabloDumpOptions));
static cl::opt<bool> PrintCompiledCCcode("print-CC-pablo", cl::init(false), cl::desc("print Pablo output from character class compiler"), cl::cat(dPabloDumpOptions));
static cl::opt<bool> PrintCompiledREcode("print-RE-pablo", cl::init(false), cl::desc("print Pablo output from the regular expression compiler"), cl::cat(dPabloDumpOptions));

static cl::OptionCategory cPabloOptimizationsOptions("Pablo Optimizations", "These options control Pablo optimization passes.");

static cl::opt<bool> DisablePabloCSE("disable-CSE", cl::init(false),
                                     cl::desc("Disable Pablo common subexpression elimination/dead code elimination"),
                                     cl::cat(cPabloOptimizationsOptions));
static cl::opt<bool> PabloSinkingPass("sinking", cl::init(false),
                                      cl::desc("Moves all instructions into the innermost legal If-scope so that they are only executed when needed."),
                                      cl::cat(cPabloOptimizationsOptions));

#ifdef ENABLE_MULTIPLEXING
static cl::opt<bool> PrintUnloweredCode("print-unlowered-pablo", cl::init(false), cl::desc("print Pablo output prior to lowering. "), cl::cat(dPabloDumpOptions));

static cl::opt<bool> EnableMultiplexing("multiplexing", cl::init(false),
                                        cl::desc("combine Advances whose inputs are mutual exclusive into the fewest number of advances possible (expensive)."),
                                        cl::cat(cPabloOptimizationsOptions));

static cl::opt<bool> EnableLowering("lowering", cl::init(false),
                                         cl::desc("coalesce associative functions prior to optimization passes."),
                                         cl::cat(cPabloOptimizationsOptions));
static cl::opt<bool> EnablePreDistribution("pre-dist", cl::init(false),
                                         cl::desc("apply distribution law optimization."),
                                         cl::cat(cPabloOptimizationsOptions));
static cl::opt<bool> EnablePostDistribution("post-dist", cl::init(false),
                                         cl::desc("apply distribution law optimization."),
                                         cl::cat(cPabloOptimizationsOptions));
#endif

static cl::opt<bool> DisableAVX2("disable-AVX2", cl::init(false), cl::desc("disable AVX2 instruction set."), cl::cat(cPabloOptimizationsOptions));

re::RE * regular_expression_passes(const Encoding encoding, re::RE * re_ast)  {
    if (PrintAllREs || PrintParsedREs) {
        std::cerr << "Parser:" << std::endl << Printer_RE::PrintRE(re_ast) << std::endl;
    }

    //Optimization passes to simplify the AST.
    re_ast = re::RE_Nullable::removeNullablePrefix(re_ast);
    if (PrintAllREs || PrintStrippedREs) {
        std::cerr << "RemoveNullablePrefix:" << std::endl << Printer_RE::PrintRE(re_ast) << std::endl;
    }
    re_ast = re::RE_Nullable::removeNullableSuffix(re_ast);
    if (PrintAllREs || PrintStrippedREs) {
        std::cerr << "RemoveNullableSuffix:" << std::endl << Printer_RE::PrintRE(re_ast) << std::endl;
    }

    re_ast = re::RE_Simplifier::simplify(re_ast);
    if (PrintAllREs || PrintSimplifiedREs) {
        //Print to the terminal the AST that was generated by the simplifier.
        std::cerr << "Simplifier:" << std::endl << Printer_RE::PrintRE(re_ast) << std::endl;
    }
    return re_ast;
}
    
PabloFunction * re2pablo_compiler(const Encoding encoding, re::RE * re_ast) {
    PabloFunction * function = PabloFunction::Create("process_block", 8, 2);
    cc::CC_Compiler cc_compiler(*function, encoding);
    re::RE_Compiler re_compiler(*function, cc_compiler);
    re_compiler.initializeRequiredStreams();
    re_compiler.compileUnicodeNames(re_ast);
    re_compiler.finalizeMatchResult(re_compiler.compile(re_ast));

    if (PrintCompiledREcode) {
        //Print to the terminal the AST that was generated by the pararallel bit-stream compiler.
        llvm::raw_os_ostream cerr(std::cerr);
        cerr << "Initial Pablo AST:\n";
        PabloPrinter::print(*function, cerr);
    }
    #ifndef NDEBUG
    PabloVerifier::verify(*function, "creation");
    #endif
    return function;
}

void pablo_function_passes(PabloFunction * function) {
    // Scan through the pablo code and perform DCE and CSE
    if (!DisablePabloCSE) {
        Simplifier::optimize(*function);
    }
#ifdef ENABLE_MULTIPLEXING
    if (EnableLowering || EnablePreDistribution || EnablePostDistribution || EnableMultiplexing) {
        FlattenAssociativeDFG::transform(*function);
    }
#endif
    if (PabloSinkingPass) {
        CodeMotionPass::optimize(*function);
    }
#ifdef ENABLE_MULTIPLEXING    
    if (EnablePreDistribution) {
        DistributivePass::optimize(*function);
    }
    if (EnableMultiplexing) {
        MultiplexingPass::optimize(*function);
    }
    if (EnablePostDistribution) {
        DistributivePass::optimize(*function);
    }
    SchedulingPrePass::optimize(*function);
    if (PrintUnloweredCode) {
        //Print to the terminal the AST that was generated by the pararallel bit-stream compiler.
        llvm::raw_os_ostream cerr(std::cerr);
        cerr << "Unlowered Pablo AST:\n";
        PabloPrinter::print(*function, cerr);
    }
    if (EnableLowering || EnablePreDistribution || EnablePostDistribution || EnableMultiplexing) {
        FactorizeDFG::transform(*function);
    }
#endif
    if (PrintOptimizedREcode) {
        PabloVerifier::verify(*function, "post-optimization");
        //Print to the terminal the AST that was generated by the pararallel bit-stream compiler.
        llvm::raw_os_ostream cerr(std::cerr);
        cerr << "Final Pablo AST:\n";
        PabloPrinter::print(*function, cerr);
    }
}

// Dynamic AVX2 confirmation
#if (BLOCK_SIZE == 256)
#define ISPC_LLVM_VERSION ISPC_LLVM_3_6
#include "ispc.cpp"
#endif


IDISA::IDISA_Builder * GetNativeIDISA_Builder(Module * mod, Type * bitBlockType) {

#if (BLOCK_SIZE == 256)
    if ((strncmp(lGetSystemISA(), "avx2", 4) == 0)) {
        return new IDISA::IDISA_AVX2_Builder(mod, bitBlockType);
        //std::cerr << "IDISA_AVX2_Builder selected\n";
    }
    else{
        return new IDISA::IDISA_SSE2_Builder(mod, bitBlockType);
        //std::cerr << "Generic IDISA_Builder selected\n";
    }
#else    
    return new IDISA::IDISA_SSE2_Builder(mod, bitBlockType);
#endif
}



ExecutionEngine * JIT_to_ExecutionEngine (Module * m) {

    InitializeNativeTarget();
    InitializeNativeTargetAsmPrinter();
    InitializeNativeTargetAsmParser();

    std::string errMessage;
    EngineBuilder builder(std::move(std::unique_ptr<Module>(m)));
    builder.setErrorStr(&errMessage);
    builder.setMCPU(sys::getHostCPUName());
    builder.setOptLevel(CodeGenOpt::Level::None);

#if (BLOCK_SIZE == 256)
    if (!DisableAVX2 && (strncmp(lGetSystemISA(), "avx2", 4) == 0)) {
            std::vector<std::string> attrs;
            attrs.push_back("avx2");
            builder.setMAttrs(attrs);
    //std::cerr << "+avx2 set" << std::endl;
    }
#endif
    //builder.setOptLevel(mMaxWhileDepth ? CodeGenOpt::Level::Less : CodeGenOpt::Level::None);
    ExecutionEngine * engine = builder.create();
    if (engine == nullptr) {
        throw std::runtime_error("Could not create ExecutionEngine: " + errMessage);
    }

    return engine;
}

int total_count = 0;

extern "C" {
    void wrapped_report_match(uint64_t lineNum, uint64_t line_start, uint64_t line_end, const char * buffer, int filesize, char * filename) {
        if(CountOnly){
            total_count++;
            return;
        }

        llvm::raw_os_ostream out(std::cout);
        if (ShowFileNames) {
            out << filename << ':';
        }
        if (ShowLineNumbers) {
            out << lineNum << ":";
        }

        if ((buffer[line_start] == 0xA) && (line_start != line_end)) {
            // The line "starts" on the LF of a CRLF.  Really the end of the last line.
            line_start++;
        }
        if (line_end == filesize) {
            // The match position is at end-of-file.   We have a final unterminated line.
            out.write(&buffer[line_start], line_end - line_start);
            if (NormalizeLineBreaks) {
                out << '\n';  // terminate it
            }
            return;
        }
        unsigned char end_byte = (unsigned char)buffer[line_end]; 
        if (NormalizeLineBreaks) {
            if (end_byte == 0x85) {
                // Line terminated with NEL, on the second byte.  Back up 1.
                line_end--;
            } else if (end_byte > 0xD) {
                // Line terminated with PS or LS, on the third byte.  Back up 2.
                line_end -= 2;
            }
            out.write(&buffer[line_start], line_end - line_start);
            out << '\n';
        }
        else{   
            if (end_byte == 0x0D) {
                // Check for line_end on first byte of CRLF;  note that we don't
                // want to access past the end of buffer.
                if ((line_end + 1 < filesize) && (buffer[line_end + 1] == 0x0A)) {
                    // Found CRLF; preserve both bytes.
                    line_end++;
                }
            }
            out.write(&buffer[line_start], line_end - line_start + 1);
        }
    }
}


void PrintTotalCount(){
    if(CountOnly){
        std::cout << total_count << std::endl;
    }
}

re::CC * parsedCodePointSet;

extern "C" {
    void insert_codepoints(uint64_t lineNum, uint64_t line_start, uint64_t line_end, const char * buffer) {
       re::codepoint_t c = 0;
        ssize_t line_pos = line_start;
        while (isxdigit(buffer[line_pos])) {
            if (isdigit(buffer[line_pos])) {
                c = (c << 4) | (buffer[line_pos] - '0');
            }
            else {
                c = (c << 4) | (tolower(buffer[line_pos]) - 'a' + 10);
            }
            line_pos++;
        }
        assert(((line_pos - line_start) >= 4) && ((line_pos - line_start) <= 6)); // UCD format 4 to 6 hex digits.       
        parsedCodePointSet->insert(c);
    }
}

void setParsedCodePointSet(){
    parsedCodePointSet = re::makeCC();
}

re::CC * getParsedCodePointSet(){
    return parsedCodePointSet;
}

// extern "C" {
//   void wrapped_print_register(char * regName, BitBlock bit_block) {
//       print_register<BitBlock>(regName, bit_block);
//   }
// }

void icgrep_Linking(Module * m, ExecutionEngine * e) {
    Module::FunctionListType & fns = m->getFunctionList();
    for (Module::FunctionListType::iterator it = fns.begin(), it_end = fns.end(); it != it_end; ++it) {
        std::string fnName = it->getName().str();
        if (fnName == "s2p_block") continue;
        if (fnName == "process_block") continue;
        if (fnName == "process_block_initialize_carries") continue;
        
        // if (fnName == "wrapped_print_register") {
        //     e->addGlobalMapping(cast<GlobalValue>(it), (void *)&wrapped_print_register);
        // }
        if (fnName == "wrapped_report_match") {
            e->addGlobalMapping(cast<GlobalValue>(it), (void *)&wrapped_report_match);
        }
        if (fnName == "insert_codepoints") {
            e->addGlobalMapping(cast<GlobalValue>(it), (void *)&insert_codepoints);
        }
#ifndef DISABLE_PREGENERATED_UCD_FUNCTIONS
        else {
            const UCD::ExternalProperty & ep = UCD::resolveExternalProperty(fnName);
            e->addGlobalMapping(cast<GlobalValue>(it), std::get<0>(ep));
        }
#endif
    }
}

