/*
 *  Copyright (c) 2016 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#include <string>
#include <iostream>
#include <fstream>
#include <llvm/IR/Function.h>
#include <llvm/IR/Module.h>
#include <llvm/ExecutionEngine/ExecutionEngine.h>
#include "llvm/Linker/Linker.h"
#include <llvm/Support/CommandLine.h>
#include <cc/cc_compiler.h>
#include <IR_Gen/idisa_builder.h>
#include <IR_Gen/idisa_target.h>
#include <kernels/streamset.h>
#include <kernels/mmap_kernel.h>
#include <kernels/cc_kernel.h>
#include <kernels/cc_scan_kernel.h>
#include <kernels/pipeline.h>
#include <boost/filesystem.hpp>
#include <toolchain.h>
#include <boost/iostreams/device/mapped_file.hpp>

using namespace llvm;
using namespace kernel;
using namespace parabix;


std::vector<size_t> LFPositions;
extern "C" {
void wrapped_report_pos(size_t match_pos, int dist) {
        LFPositions.push_back(match_pos);
    }

}

void preprocess_Linking(Module * m, ExecutionEngine * e) {
    Module::FunctionListType & fns = m->getFunctionList();
    for (Module::FunctionListType::iterator it = fns.begin(), it_end = fns.end(); it != it_end; ++it) {
        std::string fnName = it->getName().str();
        if (fnName == "wrapped_report_pos") {
            e->addGlobalMapping(cast<GlobalValue>(it), (void *)&wrapped_report_pos);
        }
    }
}


Function * preprocessPipeline(Module * m, IDISA::IDISA_Builder * iBuilder){
    Type * const size_ty = iBuilder->getSizeTy();
    Type * const inputType = PointerType::get(ArrayType::get(iBuilder->getBitBlockType(), 1), 0);

    Function * const mainFn = cast<Function>(m->getOrInsertFunction("LF", iBuilder->getVoidTy(), inputType, size_ty, inputType, nullptr));
    mainFn->setCallingConv(CallingConv::C);
    iBuilder->SetInsertPoint(BasicBlock::Create(m->getContext(), "entry", mainFn, 0));
    Function::arg_iterator args = mainFn->arg_begin();
    Value * const inputStream = &*(args++);
    inputStream->setName("input");
    Value * const fileSize = &*(args++);
    fileSize->setName("fileSize");
    Value * const lineBreak = &*(args++);
    lineBreak->setName("lineBreak");

    const unsigned segmentSize = codegen::SegmentSize;
    unsigned bufferSegments = codegen::BufferSegments;

    ExternalFileBuffer ByteStream(iBuilder, iBuilder->getStreamSetTy(1, 8));
    ByteStream.setStreamSetBuffer(inputStream, fileSize);
    ExternalFileBuffer MatchResults(iBuilder, iBuilder->getStreamSetTy(1, 1), segmentSize * bufferSegments);
    MatchResults.setStreamSetBuffer(lineBreak, fileSize);

    kernel::MMapSourceKernel mmapK(iBuilder, segmentSize); 
    mmapK.generateKernel({}, {&ByteStream});
    mmapK.setInitialArguments({fileSize});

    kernel::DirectCharacterClassKernelBuilder linefeedK(iBuilder, "linefeed", {re::makeCC(0x0A)}, 1);
    linefeedK.generateKernel({&ByteStream}, {&MatchResults});
    
    kernel::CCScanKernel scanMatchK(iBuilder, 1);
    scanMatchK.generateKernel({&MatchResults}, {}); 
    
    generatePipelineLoop(iBuilder, {&mmapK, &linefeedK, &scanMatchK});
    iBuilder->CreateRetVoid();

    return mainFn;

}

typedef void (*preprocessFunctionType)(char * byte_data, size_t filesize, char * lb);

preprocessFunctionType preprocessCodeGen() {
                            
    LLVMContext TheContext;
    Module * M = new Module("preprocess", TheContext);
    IDISA::IDISA_Builder * idb = IDISA::GetIDISA_Builder(M);

    llvm::Function * main_IR = preprocessPipeline(M, idb);

    ExecutionEngine * preprocessEngine = JIT_to_ExecutionEngine(M);
    ApplyObjectCache(preprocessEngine);

    preprocess_Linking(M, preprocessEngine);
    
    preprocessEngine->finalizeObject();

    delete idb;
    return reinterpret_cast<preprocessFunctionType>(preprocessEngine->getPointerToFunction(main_IR));
}

std::vector<size_t> preprocess(char * fileBuffer, size_t fileSize, char * LineBreak) {
    preprocessFunctionType preprocess_ptr = preprocessCodeGen();
    preprocess_ptr(fileBuffer, fileSize, LineBreak);
    return LFPositions;
}
