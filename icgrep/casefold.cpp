/*
 *  Copyright (c) 2015 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#include <string>
#include <iostream>
#include <fstream>
#include <sstream>

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
#include <llvm/Support/raw_ostream.h>

#include <re/re_cc.h>
#include <cc/cc_compiler.h>
#include <pablo/function.h>
#include <IDISA/idisa_builder.h>
#include <IDISA/idisa_target.h>
#include <kernels/casefold_pipeline.h>

// Dynamic processor detection
#define ISPC_LLVM_VERSION ISPC_LLVM_3_6
#include <util/ispc.cpp>

#include <utf_encoding.h>

// mmap system
#include <boost/filesystem.hpp>
#include <boost/iostreams/device/mapped_file.hpp>
using namespace boost::iostreams;
using namespace boost::filesystem;

#include <fcntl.h>

static cl::list<std::string> inputFiles(cl::Positional, cl::desc("<input file ...>"), cl::OneOrMore);


static cl::OptionCategory cMachineCodeOptimization("Machine Code Optimizations", "These options control back-end compilier optimization levels.");

static cl::opt<char> OptLevel("O", cl::desc("Optimization level. [-O0, -O1, -O2, or -O3] (default = '-O0')"),
                              cl::cat(cMachineCodeOptimization), cl::Prefix, cl::ZeroOrMore, cl::init('0'));


//
//  Functions taken from toolchain.cpp and modified for casefold 
//  JIT_t_ExecutionEngine : remove object cache
//  icgrep_Linking:   unneeded?
//  all others: definitely unneeded
//

ExecutionEngine * JIT_to_ExecutionEngine (Module * m) {

    InitializeNativeTarget();
    InitializeNativeTargetAsmPrinter();
    InitializeNativeTargetAsmParser();

    PassRegistry * Registry = PassRegistry::getPassRegistry();
    initializeCore(*Registry);
    initializeCodeGen(*Registry);
    initializeLowerIntrinsicsPass(*Registry);

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

    if ((strncmp(lGetSystemISA(), "avx2", 4) == 0)) {
            std::vector<std::string> attrs;
            attrs.push_back("avx2");
            builder.setMAttrs(attrs);
    }

    // builder.selectTarget();

    //builder.setOptLevel(mMaxWhileDepth ? CodeGenOpt::Level::Less : CodeGenOpt::Level::None);
    ExecutionEngine * engine = builder.create();
    if (engine == nullptr) {
        throw std::runtime_error("Could not create ExecutionEngine: " + errMessage);
    }
    return engine;
}


//
//  Generate Pablo code for case folding.
//

pablo::PabloFunction * casefold2pablo(const Encoding encoding) {
    pablo::PabloFunction * function = pablo::PabloFunction::Create("casefold_block", 8, 8);
    cc::CC_Compiler cc_compiler(*function, encoding);
    pablo::PabloBuilder pBuilder(cc_compiler.getBuilder().getPabloBlock(), cc_compiler.getBuilder());
    const std::vector<pablo::Var *> basis_bits = cc_compiler.getBasisBits();
    
    pablo::PabloAST * alpha = cc_compiler.compileCC(re::makeCC(0x41, 0x5A));  // ASCII A-Z
    
    function->setResult(0, pBuilder.createAssign("b0", basis_bits[0]));
    function->setResult(1, pBuilder.createAssign("b1", basis_bits[1]));
    function->setResult(2, pBuilder.createAssign("b2", pBuilder.createXor(basis_bits[2], alpha)));
    function->setResult(3, pBuilder.createAssign("b3", basis_bits[3]));
    function->setResult(4, pBuilder.createAssign("b4", basis_bits[4]));
    function->setResult(5, pBuilder.createAssign("b5", basis_bits[5]));
    function->setResult(6, pBuilder.createAssign("b6", basis_bits[6]));
    function->setResult(7, pBuilder.createAssign("b7", basis_bits[7]));
    return function;
}



typedef void (*casefoldFunctionType)(char * byte_data, size_t filesize);

casefoldFunctionType caseFoldCodeGen(void) {
                            
    Module * M = new Module("casefold", getGlobalContext());
    
    IDISA::IDISA_Builder * idb = GetIDISA_Builder(M);

    kernel::PipelineBuilder pipelineBuilder(M, idb);

    Encoding encoding(Encoding::Type::UTF_8, 8);
    
    pablo::PabloFunction * function = casefold2pablo(encoding);
    

    pipelineBuilder.CreateKernels(function);

    pipelineBuilder.ExecuteKernels();

    //std::cerr << "ExecuteKernels(); done\n";
    llvm::Function * main_IR = M->getFunction("Main");
    ExecutionEngine * mEngine = JIT_to_ExecutionEngine(M);
    
    mEngine->finalizeObject();
    //std::cerr << "finalizeObject(); done\n";

    delete idb;

    return reinterpret_cast<casefoldFunctionType>(mEngine->getPointerToFunction(main_IR));
}

void doCaseFold(casefoldFunctionType fn_ptr, const std::string & fileName) {
    std::string mFileName = fileName;
    size_t mFileSize;
    char * mFileBuffer;
    
    const path file(mFileName);
    if (exists(file)) {
        if (is_directory(file)) {
            return;
        }
    } else {
        std::cerr << "Error: cannot open " << mFileName << " for processing. Skipped.\n";
        return;
    }
    
    mFileSize = file_size(file);
    mapped_file mFile;
    if (mFileSize == 0) {
        mFileBuffer = nullptr;
    }
    else {
        try {
            mFile.open(mFileName, mapped_file::priv, mFileSize, 0);
        } catch (std::ios_base::failure e) {
            std::cerr << "Error: Boost mmap of " << mFileName << ": " << e.what() << std::endl;
            return;
        }
        mFileBuffer = mFile.data();
    }
    //std::cerr << "mFileSize =" << mFileSize << "\n";
    //std::cerr << "fn_ptr =" << std::hex << reinterpret_cast<intptr_t>(fn_ptr) << "\n";

    fn_ptr(mFileBuffer, mFileSize);

    mFile.close();
    
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

    casefoldFunctionType fn_ptr = caseFoldCodeGen();

    for (unsigned i = 0; i != inputFiles.size(); ++i) {
        doCaseFold(fn_ptr, inputFiles[i]);
    }

    return 0;
}

                       
