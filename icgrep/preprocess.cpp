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
#include <kernels/toolchain.h>
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

void preprocessPipeline(ParabixDriver & pxDriver){
    IDISA::IDISA_Builder * iBuilder = pxDriver.getIDISA_Builder();
    Module * m = iBuilder->getModule();
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
    ByteStream.setStreamSetBuffer(inputStream);
    ExternalFileBuffer MatchResults(iBuilder, iBuilder->getStreamSetTy(1, 1), segmentSize * bufferSegments);
    MatchResults.setStreamSetBuffer(lineBreak);

    kernel::MMapSourceKernel mmapK(iBuilder, segmentSize); 
    mmapK.setInitialArguments({fileSize});
    pxDriver.addKernelCall(mmapK, {}, {&ByteStream});

    kernel::DirectCharacterClassKernelBuilder linefeedK(iBuilder, "linefeed", {re::makeCC(0x0A)}, 1);
    pxDriver.addKernelCall(linefeedK, {&ByteStream}, {&MatchResults});
    
    kernel::CCScanKernel scanMatchK(iBuilder, 1);
    pxDriver.addKernelCall(scanMatchK, {&MatchResults}, {}); 
    
    pxDriver.generatePipelineIR();
    iBuilder->CreateRetVoid();

    pxDriver.addExternalLink(scanMatchK, "wrapped_report_pos", &wrapped_report_pos);
    pxDriver.linkAndFinalize();

}

typedef void (*preprocessFunctionType)(char * byte_data, size_t filesize, char * lb);

preprocessFunctionType preprocessCodeGen() {
                            
    LLVMContext TheContext;
    Module * M = new Module("preprocess", TheContext);
    IDISA::IDISA_Builder * idb = IDISA::GetIDISA_Builder(M);
    ParabixDriver pxDriver(idb);

    preprocessPipeline(pxDriver);
    
    preprocessFunctionType main = reinterpret_cast<preprocessFunctionType>(pxDriver.getPointerToMain());

    delete idb;
    return main;
}

std::vector<size_t> preprocess(char * fileBuffer, size_t fileSize, char * LineBreak) {
    preprocessFunctionType preprocess_ptr = preprocessCodeGen();
    preprocess_ptr(fileBuffer, fileSize, LineBreak);
    return LFPositions;
}
