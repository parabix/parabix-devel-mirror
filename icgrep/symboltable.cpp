/*
 *  Copyright (c) 2015 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#include <string>
#include <iostream>
#include <fstream>

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
#include <llvm/Support/Debug.h>
#include <llvm/Support/TargetSelect.h>
#include <llvm/Support/Host.h>
#include <llvm/IR/Verifier.h>
#include <IDISA/idisa_target.h>
#include <kernels/symboltablepipeline.h>

// mmap system
#include <boost/filesystem.hpp>
#include <boost/iostreams/device/mapped_file.hpp>
using namespace boost::iostreams;
using namespace boost::filesystem;

typedef void (*SymTableFunctionType)(char * byte_data, size_t filesize);

static cl::list<std::string> files(cl::Positional, cl::desc("<input file ...>"), cl::ZeroOrMore);

void process(const std::string & fileName, SymTableFunctionType function) {
    const path filePath(fileName);
    if (exists(filePath) && !is_directory(filePath)) {
        size_t fileSize = file_size(filePath);
        if (fileSize > 0) {
            mapped_file file;
            file.open(fileName, mapped_file::priv, fileSize, 0);
            char * data = file.data();
            assert (data);
            function(data, fileSize);
            file.close();
        }
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

    Module * M = new Module("symboltable", getGlobalContext());

    IDISA::IDISA_Builder * idb = GetIDISA_Builder(M);

    kernel::SymbolTableBuilder pipelineBuilder(M, idb);

    pipelineBuilder.createKernels();

    llvm::Function * main_IR = pipelineBuilder.ExecuteKernels();
    if (LLVM_UNLIKELY(main_IR == nullptr)) {
        throw std::runtime_error("No LLVM function created!");
    }
    llvm::ExecutionEngine * engine = JIT_to_ExecutionEngine(M);
    verifyModule(*M, &dbgs());
    engine->finalizeObject();

    SymTableFunctionType function = reinterpret_cast<SymTableFunctionType>(engine->getPointerToFunction(main_IR));
    if (LLVM_UNLIKELY(function == nullptr)) {
        throw std::runtime_error("filed to compile LLVM function!");
    }

    for (std::string file : files) {
        process(file, function);
    }

    delete engine;
    delete idb;
    return 0;
}
