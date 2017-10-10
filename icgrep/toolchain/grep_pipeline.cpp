/*
 *  Copyright (c) 2017 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#include "grep_pipeline.h"
#include <llvm/IR/Module.h>
#include <boost/filesystem.hpp>
#include <kernels/grep_kernel.h>
#include <kernels/linebreak_kernel.h>
#include <kernels/source_kernel.h>
#include <kernels/s2p_kernel.h>
#include <kernels/scanmatchgen.h>
#include <kernels/streamset.h>
#include <kernels/kernel_builder.h>
#include <re/re_toolchain.h>
#include <toolchain/toolchain.h>
#include <re/re_name_resolve.h>    
#include <re/re_collect_unicodesets.h>
#include <re/re_multiplex.h>
#include <toolchain/cpudriver.h>
#include <llvm/Support/raw_ostream.h>

using namespace parabix;
using namespace llvm;

namespace grep {

    
void accumulate_match_wrapper(intptr_t accum_addr, const size_t lineNum, size_t line_start, size_t line_end) {
    reinterpret_cast<MatchAccumulator *>(accum_addr)->accumulate_match(lineNum, line_start, line_end);
}

void grepBuffer(re::RE * pattern, const char * search_buffer, size_t bufferLength, MatchAccumulator * accum) {
    const unsigned segmentSize = 8;

    ParabixDriver pxDriver("codepointEngine");
    auto & idb = pxDriver.getBuilder();
    Module * M = idb->getModule();
    
    Function * mainFunc = cast<Function>(M->getOrInsertFunction("Main", idb->getVoidTy(), idb->getInt8PtrTy(), idb->getSizeTy(), nullptr));
    mainFunc->setCallingConv(CallingConv::C);
    auto args = mainFunc->arg_begin();
    Value * const buffer = &*(args++);
    buffer->setName("buffer");
    Value * length = &*(args++);
    length->setName("length");
    
    idb->SetInsertPoint(BasicBlock::Create(M->getContext(), "entry", mainFunc, 0));
    
    StreamSetBuffer * ByteStream = pxDriver.addBuffer(make_unique<SourceBuffer>(idb, idb->getStreamSetTy(1, 8)));
    kernel::Kernel * sourceK = pxDriver.addKernelInstance(make_unique<kernel::MemorySourceKernel>(idb, idb->getInt8PtrTy(), segmentSize));
    sourceK->setInitialArguments({buffer, length});
    pxDriver.makeKernelCall(sourceK, {}, {ByteStream});
    
    StreamSetBuffer * BasisBits = pxDriver.addBuffer(make_unique<CircularBuffer>(idb, idb->getStreamSetTy(8, 1), segmentSize));
    
    kernel::Kernel * s2pk = pxDriver.addKernelInstance(make_unique<kernel::S2PKernel>(idb));
    pxDriver.makeKernelCall(s2pk, {ByteStream}, {BasisBits});
    
    kernel::Kernel * linebreakK = pxDriver.addKernelInstance(make_unique<kernel::LineBreakKernelBuilder>(idb, 8));
    StreamSetBuffer * LineBreakStream = pxDriver.addBuffer(make_unique<CircularBuffer>(idb, idb->getStreamSetTy(1, 1), segmentSize));
    pxDriver.makeKernelCall(linebreakK, {BasisBits}, {LineBreakStream});
    
    kernel::Kernel * requiredStreamsK = pxDriver.addKernelInstance(make_unique<kernel::RequiredStreams_UTF8>(idb));
    StreamSetBuffer * RequiredStreams = pxDriver.addBuffer(make_unique<CircularBuffer>(idb, idb->getStreamSetTy(4, 1), segmentSize));
    pxDriver.makeKernelCall(requiredStreamsK, {BasisBits}, {RequiredStreams});
    
    StreamSetBuffer * MatchResults = pxDriver.addBuffer(make_unique<CircularBuffer>(idb, idb->getStreamSetTy(1, 1), segmentSize));
    kernel::Kernel * icgrepK = pxDriver.addKernelInstance(make_unique<kernel::ICGrepKernel>(idb, pattern));
    pxDriver.makeKernelCall(icgrepK, {BasisBits, LineBreakStream, RequiredStreams}, {MatchResults});
    
    StreamSetBuffer * MatchedLines = pxDriver.addBuffer(make_unique<CircularBuffer>(idb, idb->getStreamSetTy(1, 1), segmentSize));
    kernel::Kernel * matchedLinesK = pxDriver.addKernelInstance(make_unique<kernel::MatchedLinesKernel>(idb));
    pxDriver.makeKernelCall(matchedLinesK, {MatchResults, LineBreakStream}, {MatchedLines});
    
    kernel::Kernel * scanMatchK = pxDriver.addKernelInstance(make_unique<kernel::ScanMatchKernel>(idb, GrepType::CallBack, 8));
    scanMatchK->setInitialArguments({ConstantInt::get(idb->getIntAddrTy(), reinterpret_cast<intptr_t>(accum))});
    pxDriver.makeKernelCall(scanMatchK, {MatchedLines, LineBreakStream, ByteStream}, {});
    pxDriver.LinkFunction(*scanMatchK, "accumulate_match_wrapper", &accumulate_match_wrapper);
    pxDriver.generatePipelineIR();
    pxDriver.deallocateBuffers();
    idb->CreateRetVoid();
    pxDriver.finalizeObject();
    
    typedef void (*GrepFunctionType)(const char * buffer, const size_t length);
    auto f = reinterpret_cast<GrepFunctionType>(pxDriver.getMain());
    f(search_buffer, bufferLength);
}
}