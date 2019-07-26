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
#include <kernels/core/streamset.h>
#include <kernels/source_kernel.h>
#include <kernels/cc_kernel.h>
#include <kernels/cc_scan_kernel.h>
#include <boost/filesystem.hpp>
#include <boost/iostreams/device/mapped_file.hpp>

using namespace llvm;
using namespace kernel;

namespace re {class RE;}


std::vector<size_t> LFPositions;
extern "C" {
void wrapped_report_pos(size_t match_pos, int dist) {
        LFPositions.push_back(match_pos);
    }

}

void preprocessPipeline(CPUEngineInstance & pxDriver){
    auto & iBuilder = pxDriver.getBuilder();
    Module * m = iBuilder->getModule();
    Type * const size_ty = iBuilder->getSizeTy();
    Type * const inputType = iBuilder->getInt8PtrTy();
    FunctionType * mainTy = FunctionType::get(iBuilder->getVoidTy(), {inputType, size_ty, inputType}, false);
    Function * const mainFn = Function::Create(mainTy, Function::InternalLinkage, "Main", m);
    mainFn->setCallingConv(CallingConv::C);
    iBuilder->SetInsertPoint(BasicBlock::Create(m->getContext(), "entry", mainFn, 0));
    Function::arg_iterator args = mainFn->arg_begin();
    Value * const inputStream = &*(args++);
    inputStream->setName("input");
    Value * const fileSize = &*(args++);
    fileSize->setName("fileSize");

    const unsigned segmentSize = codegen::SegmentSize;
    unsigned bufferSegments = codegen::BufferSegments;

    StreamSetBuffer * ByteStream = pxDriver.addBuffer<ExternalBuffer>(iBuilder, iBuilder->getStreamSetTy(1, 8));
    kernel::Kernel * sourceK = pxDriver.addKernelInstance<kernel::MemorySourceKernel>(iBuilder, iBuilder->getInt8PtrTy());
    sourceK->setInitialArguments({inputStream, fileSize});
    pxDriver.makeKernelCall(sourceK, {}, {ByteStream});

    StreamSetBuffer * MatchResults = pxDriver.addBuffer<StaticBuffer>(iBuilder, iBuilder->getStreamSetTy(1, 1), segmentSize * bufferSegments);
    kernel::Kernel * linefeedK = pxDriver.addKernelInstance<kernel::DirectCharacterClassKernelBuilder>(iBuilder, "linefeed", std::vector<re::CC *>{re::makeCC(0x0A)}, 1);
    pxDriver.makeKernelCall(linefeedK, {ByteStream}, {MatchResults});

    kernel::Kernel * scanMatchK = pxDriver.addKernelInstance<kernel::CCScanKernel>(iBuilder, 1);
    pxDriver.makeKernelCall(scanMatchK, {MatchResults}, {});

    pxDriver.generatePipelineIR();
    iBuilder->CreateRetVoid();

    pxDriver.LinkFunction(*scanMatchK, "wrapped_report_pos", wrapped_report_pos);
    pxDriver.finalizeObject();

}

typedef void (*preprocessFunctionType)(char * byte_data, size_t filesize);

std::vector<size_t> preprocess(char * fileBuffer, size_t fileSize) {
    CPUEngineInstance pxDriver("preprocess");
    preprocessPipeline(pxDriver);
    auto main = reinterpret_cast<preprocessFunctionType>(pxDriver.getMain());
    main(fileBuffer, fileSize);
    return LFPositions;
}
