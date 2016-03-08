/*
 *  Copyright (c) 2015 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#include <string>
#include <iostream>
#include <fstream>

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
#include <IDISA/idisa_i64_builder.h>
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
#include <object_cache.h>
// Dynamic processor detection
#define ISPC_LLVM_VERSION ISPC_LLVM_3_6
#include "ispc.cpp"

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

static cl::OptionCategory dPabloDumpOptions("Pablo Dump Options", "These options control printing of intermediate Pablo code.");

static cl::opt<bool> PrintOptimizedREcode("print-pablo", cl::init(false), cl::desc("print final optimized Pablo code"), cl::cat(dPabloDumpOptions));
static cl::opt<bool> PrintCompiledCCcode("print-CC-pablo", cl::init(false), cl::desc("print Pablo output from character class compiler"), cl::cat(dPabloDumpOptions));
static cl::opt<bool> PrintCompiledREcode("print-RE-pablo", cl::init(false), cl::desc("print Pablo output from the regular expression compiler"), cl::cat(dPabloDumpOptions));
static cl::opt<std::string> PabloOutputFilename("print-pablo-output", cl::init(""), cl::desc("output Pablo filename"), cl::cat(dPabloDumpOptions));

static cl::OptionCategory cMachineCodeOptimization("Machine Code Optimizations", "These options control back-end compilier optimization levels.");


static cl::opt<char> OptLevel("O", cl::desc("Optimization level. [-O0, -O1, -O2, or -O3] (default = '-O0')"),
                              cl::cat(cMachineCodeOptimization), cl::Prefix, cl::ZeroOrMore, cl::init('0'));

static cl::OptionCategory cPabloOptimizationsOptions("Pablo Optimizations", "These options control Pablo optimization passes.");

static cl::opt<bool> DisableSimplification("disable-simplification", cl::init(false),
                                     cl::desc("Disable Pablo Simplification pass (not recommended)"),
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
                                         cl::desc("apply distribution law optimization prior to multiplexing."),
                                         cl::cat(cPabloOptimizationsOptions));

static cl::opt<bool> EnablePostDistribution("post-dist", cl::init(false),
                                         cl::desc("apply distribution law optimization after multiplexing."),
                                         cl::cat(cPabloOptimizationsOptions));

static cl::opt<bool> EnablePrePassScheduling("pre-pass-scheduling", cl::init(false),
                                         cl::desc("apply pre-pass scheduling prior to LLVM IR generation."),
                                         cl::cat(cPabloOptimizationsOptions));
#endif

static cl::OptionCategory dCodeGenOptions("Code Generation Options", "These options control code generation.");

static cl::opt<bool> DisableAVX2("disable-AVX2", cl::init(false), cl::desc("disable AVX2 instruction set."), cl::cat(dCodeGenOptions));

static cl::opt<int> BlockSize("BlockSize", cl::init(0), cl::desc("specify a block size (defaults to widest SIMD register width in bits)."), cl::cat(dCodeGenOptions));


static cl::OptionCategory cObjectCache("Object Caching", "These options control back-end object caching behaviours.");

static cl::opt<bool> EnableObjectCache("enable-object-cache", cl::init(false), cl::desc("Enable object caching"), cl::cat(cObjectCache));

static cl::opt<std::string> ObjectCacheDir("object-cache-dir", cl::init(""), cl::desc("Path to the object cache diretory"), cl::cat(cObjectCache));

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

#ifdef PRINT_TIMING_INFORMATION
#define READ_CYCLE_COUNTER(name) name = read_cycle_counter();
#else
#define READ_CYCLE_COUNTER(name)
#endif

#ifdef PRINT_TIMING_INFORMATION
unsigned COUNT_STATEMENTS(const PabloFunction * const entry) {
    std::stack<const Statement *> scope;
    unsigned statements = 0;
    // Scan through and collect all the advances, calls, scanthrus and matchstars ...
    for (const Statement * stmt = entry->getEntryBlock()->front(); ; ) {
        while ( stmt ) {
            ++statements;
            if (LLVM_UNLIKELY(isa<If>(stmt) || isa<While>(stmt))) {
                // Set the next statement to be the first statement of the inner scope and push the
                // next statement of the current statement into the scope stack.
                const PabloBlock * const nested = isa<If>(stmt) ? cast<If>(stmt)->getBody() : cast<While>(stmt)->getBody();
                scope.push(stmt->getNextNode());
                stmt = nested->front();
                assert (stmt);
                continue;
            }
            stmt = stmt->getNextNode();
        }
        if (scope.empty()) {
            break;
        }
        stmt = scope.top();
        scope.pop();
    }
    return statements;
}

unsigned COUNT_ADVANCES(const PabloFunction * const entry) {

    std::stack<const Statement *> scope;
    unsigned advances = 0;

    // Scan through and collect all the advances, calls, scanthrus and matchstars ...
    for (const Statement * stmt = entry->getEntryBlock()->front(); ; ) {
        while ( stmt ) {
            if (isa<Advance>(stmt)) {
                ++advances;
            }
            else if (LLVM_UNLIKELY(isa<If>(stmt) || isa<While>(stmt))) {
                // Set the next statement to be the first statement of the inner scope and push the
                // next statement of the current statement into the scope stack.
                const PabloBlock * const nested = isa<If>(stmt) ? cast<If>(stmt)->getBody() : cast<While>(stmt)->getBody();
                scope.push(stmt->getNextNode());
                stmt = nested->front();
                assert (stmt);
                continue;
            }
            stmt = stmt->getNextNode();
        }
        if (scope.empty()) {
            break;
        }
        stmt = scope.top();
        scope.pop();
    }
    return advances;
}

using DistributionMap = boost::container::flat_map<unsigned, unsigned>;

DistributionMap SUMMARIZE_VARIADIC_DISTRIBUTION(const PabloFunction * const entry) {
    std::stack<const Statement *> scope;
    DistributionMap distribution;
    // Scan through and collect all the advances, calls, scanthrus and matchstars ...
    for (const Statement * stmt = entry->getEntryBlock()->front(); ; ) {
        while ( stmt ) {
            if (isa<Variadic>(stmt)) {
                auto f = distribution.find(stmt->getNumOperands());
                if (f == distribution.end()) {
                    distribution.emplace(stmt->getNumOperands(), 1);
                } else {
                    f->second += 1;
                }
            }
            else if (LLVM_UNLIKELY(isa<If>(stmt) || isa<While>(stmt))) {
                // Set the next statement to be the first statement of the inner scope and push the
                // next statement of the current statement into the scope stack.
                const PabloBlock * const nested = isa<If>(stmt) ? cast<If>(stmt)->getBody() : cast<While>(stmt)->getBody();
                scope.push(stmt->getNextNode());
                stmt = nested->front();
                assert (stmt);
                continue;
            }
            stmt = stmt->getNextNode();
        }
        if (scope.empty()) {
            break;
        }
        stmt = scope.top();
        scope.pop();
    }
    return distribution;
}
#endif

void pablo_function_passes(PabloFunction * function) {
    // Scan through the pablo code and perform DCE and CSE

#ifdef PRINT_TIMING_INFORMATION
    timestamp_t simplification_start = 0, simplification_end = 0;
    timestamp_t coalescing_start = 0, coalescing_end = 0;
    timestamp_t sinking_start = 0, sinking_end = 0;
    timestamp_t pre_distribution_start = 0, pre_distribution_end = 0;
    timestamp_t multiplexing_start = 0, multiplexing_end = 0;
    timestamp_t post_distribution_start = 0, post_distribution_end = 0;
    timestamp_t lowering_start = 0, lowering_end = 0;
    timestamp_t scheduling_start = 0, scheduling_end = 0;
    DistributionMap distribution;
    const timestamp_t optimization_start = read_cycle_counter();
#endif
    if (!DisableSimplification) {
        READ_CYCLE_COUNTER(simplification_start);
        Simplifier::optimize(*function);
        READ_CYCLE_COUNTER(simplification_end);
    }
#ifdef ENABLE_MULTIPLEXING
    if (EnableLowering || EnablePreDistribution || EnablePostDistribution) {
        READ_CYCLE_COUNTER(coalescing_start);
        CanonicalizeDFG::transform(*function);
        READ_CYCLE_COUNTER(coalescing_end);
    }
    if (EnablePreDistribution) {
        READ_CYCLE_COUNTER(pre_distribution_start);
        DistributivePass::optimize(*function);
        READ_CYCLE_COUNTER(pre_distribution_end);
    }
    if (EnableMultiplexing) {
        READ_CYCLE_COUNTER(multiplexing_start);
        MultiplexingPass::optimize(*function);
        READ_CYCLE_COUNTER(multiplexing_end);
        if (EnableLowering || EnablePreDistribution || EnablePostDistribution) {
            CanonicalizeDFG::transform(*function);
        }
    }
    if (EnablePostDistribution) {
        READ_CYCLE_COUNTER(post_distribution_start);
        DistributivePass::optimize(*function);
        READ_CYCLE_COUNTER(post_distribution_end);
    }
#endif
    if (PabloSinkingPass) {
        READ_CYCLE_COUNTER(sinking_start);
        CodeMotionPass::optimize(*function);
        READ_CYCLE_COUNTER(sinking_end);
    }
#ifdef ENABLE_MULTIPLEXING
    if (PrintUnloweredCode) {
        //Print to the terminal the AST that was generated by the pararallel bit-stream compiler.
        llvm::raw_os_ostream cerr(std::cerr);
        cerr << "Unlowered Pablo AST:\n";
        PabloPrinter::print(*function, cerr);
    }
    #ifdef PRINT_TIMING_INFORMATION
    distribution = SUMMARIZE_VARIADIC_DISTRIBUTION(function);
    #endif
    if (EnableLowering || EnablePreDistribution || EnablePostDistribution) {
        READ_CYCLE_COUNTER(lowering_start);
        FactorizeDFG::transform(*function);
        READ_CYCLE_COUNTER(lowering_end);
    }
    if (EnablePrePassScheduling) {
        READ_CYCLE_COUNTER(scheduling_start);
        SchedulingPrePass::optimize(*function);
        READ_CYCLE_COUNTER(scheduling_end);
    }
#endif
#ifdef PRINT_TIMING_INFORMATION
    const timestamp_t optimization_end = read_cycle_counter();
#endif
    if (PrintOptimizedREcode) {
        if (PabloOutputFilename.empty()) {
            //Print to the terminal the AST that was generated by the pararallel bit-stream compiler.
            llvm::raw_os_ostream cerr(std::cerr);
            cerr << "Final Pablo AST:\n";
            PabloPrinter::print(*function, cerr);
        } else {
            std::error_code error;
            llvm::raw_fd_ostream out(PabloOutputFilename, error, sys::fs::OpenFlags::F_None);
            PabloPrinter::print(*function, out);
        }
    }
#ifdef PRINT_TIMING_INFORMATION
    std::cerr << "PABLO OPTIMIZATION TIME: " << (optimization_end - optimization_start) << std::endl;
    std::cerr << "  SIMPLIFICATION TIME: " << (simplification_end - simplification_start) << std::endl;
    std::cerr << "  COALESCING TIME: " << (coalescing_end - coalescing_start) << std::endl;
    std::cerr << "  SINKING TIME: " << (sinking_end - sinking_start) << std::endl;
    std::cerr << "  PRE-DISTRIBUTION TIME: " << (pre_distribution_end - pre_distribution_start) << std::endl;
    std::cerr << "  MULTIPLEXING TIME: " << (multiplexing_end - multiplexing_start) << std::endl;
    std::cerr << "  MULTIPLEXING SEED: " << MultiplexingPass::SEED << std::endl;
    std::cerr << "  MULTIPLEXING NODES USED: " << MultiplexingPass::NODES_USED << std::endl;
    std::cerr << "  MULTIPLEXING NODES ALLOCATED: " << MultiplexingPass::NODES_ALLOCATED << std::endl;
    std::cerr << "  LOWERING TIME: " << (lowering_end - lowering_start) << std::endl;
    std::cerr << "  POST-DISTRIBUTION TIME: " << (post_distribution_end - post_distribution_start) << std::endl;
    std::cerr << "  SCHEDULING TIME: " << (scheduling_end - scheduling_start) << std::endl;
    std::cerr << "PABLO STATEMENTS: " << COUNT_STATEMENTS(function) << std::endl;
    std::cerr << "PABLO ADVANCES: " << COUNT_ADVANCES(function) << std::endl;
    std::cerr << "PRE-LOWERING VARIADIC DISTRIBUTION: ";
    bool join = false;
    for (auto dist : distribution) {
        if (join) {
            std::cerr << ';';
        }
        std::cerr << dist.first << '|' << dist.second;
        join = true;
    }
    std::cerr << std::endl;
#endif
}


IDISA::IDISA_Builder * GetIDISA_Builder(Module * mod) {
    bool hasAVX2 = (strncmp(lGetSystemISA(), "avx2", 4) == 0);
    
    unsigned theBlockSize = BlockSize;  // from command line
    
    if (theBlockSize == 0) {  // No BlockSize override: use processor SIMD width
        theBlockSize = hasAVX2 ? 256 : 128;
    }
    Type * bitBlockType = VectorType::get(IntegerType::get(getGlobalContext(), 64), theBlockSize/64);
    
    int blockSize = bitBlockType->isIntegerTy() ? cast<IntegerType>(bitBlockType)->getIntegerBitWidth() : cast<VectorType>(bitBlockType)->getBitWidth();
    if (blockSize >= 256) {
        if (hasAVX2) {
            return new IDISA::IDISA_AVX2_Builder(mod, bitBlockType);
        }
        else{
            return new IDISA::IDISA_SSE2_Builder(mod, bitBlockType);
        }
    }
    else if (blockSize == 64)
        return new IDISA::IDISA_I64_Builder(mod, bitBlockType);  
    return new IDISA::IDISA_SSE2_Builder(mod, bitBlockType);
}



ExecutionEngine * JIT_to_ExecutionEngine (Module * m) {

    InitializeNativeTarget();
    InitializeNativeTargetAsmPrinter();
    InitializeNativeTargetAsmParser();

    std::string errMessage;
    EngineBuilder builder(std::move(std::unique_ptr<Module>(m)));
    builder.setErrorStr(&errMessage);
    builder.setMCPU(sys::getHostCPUName());
    CodeGenOpt::Level optLevel = CodeGenOpt::Level::None;
    switch (OptLevel) {
        case '0': optLevel = CodeGenOpt::None; break;
        case '1': optLevel = CodeGenOpt::Less; break;
        case '2': optLevel = CodeGenOpt::Default; break;
        case '3': optLevel = CodeGenOpt::Aggressive; break;
        default: errs() << OptLevel << " is an invalid optimization level.\n";
    }
    builder.setOptLevel(optLevel);

    if (!DisableAVX2 && (strncmp(lGetSystemISA(), "avx2", 4) == 0)) {
            std::vector<std::string> attrs;
            attrs.push_back("avx2");
            builder.setMAttrs(attrs);
    }

    //builder.setOptLevel(mMaxWhileDepth ? CodeGenOpt::Level::Less : CodeGenOpt::Level::None);
    ExecutionEngine * engine = builder.create();
    ICGrepObjectCache * cache = nullptr;
    if (engine == nullptr) {
        throw std::runtime_error("Could not create ExecutionEngine: " + errMessage);
    }

    if (EnableObjectCache) {
        // TODO a better default cache diretory (HOME expansion?)
        if (ObjectCacheDir.empty())
            cache = new ICGrepObjectCache(".icgrep-cache/");
        else
            cache = new ICGrepObjectCache(ObjectCacheDir);
        engine->setObjectCache(cache);
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
