/*
 *  Copyright (c) 2015 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#include <compiler.h>
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
#endif
#include "UCD/precompiled_gc.h"
#include "UCD/precompiled_sc.h"
#include "UCD/precompiled_scx.h"
#include "UCD/precompiled_blk.h"
#include "UCD/precompiled_derivedcoreproperties.h"
#include "UCD/precompiled_proplist.h"
#include <llvm/Support/CommandLine.h>
#include <pablo/function.h>
#include <re/printer_re.h>
#include <pablo/printer_pablos.h>
#include <iostream>

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


static cl::OptionCategory cPabloOptimizationsOptions("Pablo Optimizations",
                                              "These options control Pablo optimization passes.");

static cl::opt<bool> DisablePabloCSE("disable-CSE", cl::init(false),
                                      cl::desc("Disable Pablo common subexpression elimination/dead code elimination"),
                                      cl::cat(cPabloOptimizationsOptions));
static cl::opt<bool> PabloSinkingPass("sinking", cl::init(false),
                                      cl::desc("Moves all instructions into the innermost legal If-scope so that they are only executed when needed."),
                                      cl::cat(cPabloOptimizationsOptions));
#ifdef ENABLE_MULTIPLEXING
static cl::opt<bool> EnableMultiplexing("enable-multiplexing", cl::init(false),
                                      cl::desc("combine Advances whose inputs are mutual exclusive into the fewest number of advances possible (expensive)."),
                                      cl::cat(cPabloOptimizationsOptions));
#endif

using namespace re;
using namespace cc;
using namespace pablo;

namespace icgrep {

CompiledPabloFunction compile(const Encoding encoding, const std::vector<std::string> regexps, const ModeFlagSet initialFlags) {
    std::vector<RE *> REs;
    RE * re_ast = nullptr;
    for (int i = 0; i < regexps.size(); i++) {
        try
        {
            re_ast = RE_Parser::parse(regexps[i], initialFlags);
        }
        catch (ParseFailure failure)
        {
            std::cerr << "Regex parsing failure: " << failure.what() << std::endl;
            std::cerr << regexps[i] << std::endl;
            exit(1);
        }
        REs.push_back(re_ast);
    }
    if (REs.size() > 1) {
        re_ast = makeAlt(REs.begin(), REs.end());
    }

    if (PrintAllREs || PrintParsedREs) {
        std::cerr << "Parser:" << std::endl << Printer_RE::PrintRE(re_ast) << std::endl;
    }

    //Optimization passes to simplify the AST.
    re_ast = RE_Nullable::removeNullablePrefix(re_ast);
    if (PrintAllREs || PrintStrippedREs) {
        std::cerr << "RemoveNullablePrefix:" << std::endl << Printer_RE::PrintRE(re_ast) << std::endl;
    }
    re_ast = RE_Nullable::removeNullableSuffix(re_ast);
    if (PrintAllREs || PrintStrippedREs) {
        std::cerr << "RemoveNullableSuffix:" << std::endl << Printer_RE::PrintRE(re_ast) << std::endl;
    }

    CC_NameMap nameMap;
    re_ast = nameMap.process(re_ast, UnicodeClass);

    // std::cerr << "-----------------------------" << std::endl;

    if (PrintAllREs || PrintNamedREs) {
        std::cerr << "Namer:" << std::endl << Printer_RE::PrintRE(re_ast) << std::endl;
        std::cerr << "NameMap:\n" << nameMap.printMap() << std::endl;
    }

    //Add the UTF encoding.
    if (encoding.getType() == Encoding::Type::UTF_8) {
        re_ast = UTF8_Encoder::toUTF8(nameMap, re_ast);
        if (PrintAllREs || PrintUTF8REs) {
            //Print to the terminal the AST that was generated by the utf8 encoder.
            std::cerr << "UTF8-encoder:" << std::endl << Printer_RE::PrintRE(re_ast) << std::endl;
            std::cerr << "NameMap:\n" << nameMap.printMap() << std::endl;
        }
    }
    
    re_ast = RE_Simplifier::simplify(re_ast);
    if (PrintAllREs || PrintSimplifiedREs) {
      //Print to the terminal the AST that was generated by the simplifier.
      std::cerr << "Simplifier:" << std::endl << Printer_RE::PrintRE(re_ast) << std::endl;
    }

    PabloFunction function = PabloFunction::Create("process_block", 8, 2);

    CC_Compiler cc_compiler(function, encoding);
    
    cc_compiler.compileByteClasses(re_ast);
    
    if (PrintCompiledCCcode) {
      //Print to the terminal the AST that was generated by the character class compiler.
      llvm::raw_os_ostream cerr(std::cerr);
      cerr << "CC AST:" << "\n";
      PabloPrinter::print(function.getEntryBlock().statements(), cerr);
    }
    
    RE_Compiler re_compiler(function, cc_compiler);
    re_compiler.initializeRequiredStreams();
    re_compiler.finalizeMatchResult(re_compiler.compile(re_ast));

    if (PrintCompiledREcode) {
        //Print to the terminal the AST that was generated by the pararallel bit-stream compiler.
        llvm::raw_os_ostream cerr(std::cerr);
        cerr << "Initial Pablo AST:\n";
        PabloPrinter::print(function.getEntryBlock().statements(), cerr);
    }

    // Scan through the pablo code and perform DCE and CSE
    if (!DisablePabloCSE) {
        Simplifier::optimize(function);
    }
    if (PabloSinkingPass) {
        CodeSinking::optimize(function);
    }
    #ifdef ENABLE_MULTIPLEXING
    if (EnableMultiplexing) {
        AutoMultiplexing::optimize(function);
    }
    #endif
    if (PrintOptimizedREcode) {
      //Print to the terminal the AST that was generated by the pararallel bit-stream compiler.
      llvm::raw_os_ostream cerr(std::cerr);
      cerr << "Final Pablo AST:\n";
      PabloPrinter::print(function.getEntryBlock().statements(), cerr);
    }

    PabloCompiler pablo_compiler;
    if (UsePregeneratedUnicode()) {
        install_property_gc_fn_ptrs(pablo_compiler);
        install_property_sc_fn_ptrs(pablo_compiler);
        install_property_scx_fn_ptrs(pablo_compiler);
        install_property_blk_fn_ptrs(pablo_compiler);
        install_property_DerivedCoreProperties_fn_ptrs(pablo_compiler);
        install_property_PropList_fn_ptrs(pablo_compiler);
    }
    try {
        CompiledPabloFunction retVal = pablo_compiler.compile(function);
        releaseSlabAllocatorMemory();
        return retVal;
    }
    catch (std::runtime_error e) {
        releaseSlabAllocatorMemory();
        std::cerr << "Runtime error: " << e.what() << std::endl;
        exit(1);
    }
}

}
