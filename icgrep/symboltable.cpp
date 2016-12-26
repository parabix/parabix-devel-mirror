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
#include <IR_Gen/idisa_target.h>
#include <kernels/symboltablepipeline.h>

// mmap system
#include <boost/filesystem.hpp>
#include <boost/iostreams/device/mapped_file.hpp>

typedef void (*SymTableFunctionType)(char * byte_data, size_t filesize);

static cl::list<std::string> files(cl::Positional, cl::desc("<input file ...>"), cl::ZeroOrMore);

void process(const std::string & fileName, SymTableFunctionType function) {
    const boost::filesystem::path filePath(fileName);
    if (exists(filePath) && !is_directory(filePath)) {
        size_t fileSize = file_size(filePath);
        if (fileSize > 0) {
            boost::iostreams::mapped_file file;
            file.open(fileName, boost::iostreams::mapped_file::priv, fileSize, 0);
            char * data = file.data();
            assert (data);
            function(data, fileSize);
            file.close();
        }
    }
}


int main(int argc, char *argv[]) {

    LLVMContext TheContext;

    Module * M = new Module("symboltable", TheContext);

    IDISA::IDISA_Builder * idb = IDISA::GetIDISA_Builder(M);

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
