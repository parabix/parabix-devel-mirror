/*
 *  Copyright (c) 2016 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 */


#include <toolchain.h>
#include "pipeline.h"
#include "utf_encoding.h"

#include <kernels/scanmatchgen.h>
#include <kernels/s2p_kernel.h>

#include <pablo/function.h>
#include <pablo/pablo_kernel.h>
#include <pablo/pablo_toolchain.h>

#include <llvm/IR/Intrinsics.h>
#include "llvm/Support/SourceMgr.h"
#include "llvm/IRReader/IRReader.h"
#include "llvm/Linker/Linker.h"



using namespace pablo;
using namespace kernel;

PipelineBuilder::PipelineBuilder(Module * m, IDISA::IDISA_Builder * b)
: mMod(m)
, iBuilder(b)
, mBitBlockType(b->getBitBlockType())
, mBlockSize(b->getBitBlockWidth()) {

}

PipelineBuilder::~PipelineBuilder() {
}

inline Value * generatePopcount(IDISA::IDISA_Builder * iBuilder, Value * bits) {
    Value * ctpopFunc = Intrinsic::getDeclaration(iBuilder->getModule(), Intrinsic::ctpop, bits->getType());
    return iBuilder->CreateCall(ctpopFunc, {bits});
}

inline Value * Cal_Count(Value * match_ptr, IDISA::IDISA_Builder * iBuilder) {
    Value * matches = iBuilder->CreateLoad(match_ptr, false, "match");
    return generatePopcount(iBuilder, matches);
}

Function * PipelineBuilder::ExecuteKernels(PabloFunction * function, bool isNameExpression, bool CountOnly, bool UTF_16) {
    
    s2pKernel  s2pk(iBuilder);
    scanMatchKernel scanMatchK(iBuilder, 64, false);

    s2pk.generateKernel();
    scanMatchK.generateKernel();
    
    //std::unique_ptr<Module> s2pM = s2pk.createKernelModule();
    //std::unique_ptr<Module> scanMatchM = scanMatchK.createKernelModule();
    
    //s2pk.addKernelDeclarations(mMod);
    //scanMatchK.addKernelDeclarations(mMod);

    pablo_function_passes(function);
    PabloKernel  icgrepK(iBuilder, "icgrep", function, {"matchedLineCount"});
    icgrepK.generateKernel();

    //std::unique_ptr<Module> icgrepM = icgrepK.createKernelModule();
    //icgrepK.addKernelDeclarations(mMod);
    
    Type * const int64ty = iBuilder->getInt64Ty();
    Type * const int8PtrTy = iBuilder->getInt8PtrTy();
    Type * const inputType = PointerType::get(ArrayType::get(ArrayType::get(mBitBlockType, (UTF_16 ? 16 : 8)), 1), 0);
    Type * const resultTy = CountOnly ? int64ty : iBuilder->getVoidTy();
    Function * const main = cast<Function>(mMod->getOrInsertFunction("Main", resultTy, inputType, int64ty, int64ty, nullptr));
    main->setCallingConv(CallingConv::C);
    Function::arg_iterator args = main->arg_begin();

    Value * const inputStream = &*(args++);
    inputStream->setName("input");
    Value * const bufferSize = &*(args++);
    bufferSize->setName("bufferSize");
    Value * const fileIdx = &*(args++);
    fileIdx->setName("fileIdx");

    iBuilder->SetInsertPoint(BasicBlock::Create(mMod->getContext(), "entry", main,0));
    BasicBlock * entryBlock = iBuilder->GetInsertBlock();
    BasicBlock * fullCondBlock = BasicBlock::Create(mMod->getContext(), "fullCond", main, 0);
    BasicBlock * fullBodyBlock = BasicBlock::Create(mMod->getContext(), "fullBody", main, 0);
    BasicBlock * finalBlock = BasicBlock::Create(mMod->getContext(), "final", main, 0);

    
    const unsigned segmentSize = 1;// or codegen::SegmentSize
    
    StreamSetBuffer ByteStream(iBuilder, StreamSetType(1, (UTF_16 ? 16 : 8)), 0);
    StreamSetBuffer BasisBits(iBuilder, StreamSetType((UTF_16 ? 16 : 8), 1), segmentSize);
    StreamSetBuffer MatchResults(iBuilder, StreamSetType(2, 1), segmentSize);
    
    ByteStream.setStreamSetBuffer(inputStream);
    BasisBits.allocateBuffer();
    MatchResults.allocateBuffer();

    Value * initialBufferSize = bufferSize;
    Value * initialBlockNo = iBuilder->getInt64(0);
    BasicBlock * initialBlock = entryBlock;
    
    Value * s2pInstance = s2pk.createInstance({});
    Value * icgrepInstance = icgrepK.createInstance({});
    Value * scanMatchInstance = nullptr;
    if (!CountOnly) {
        scanMatchInstance = scanMatchK.createInstance({iBuilder->CreateBitCast(inputStream, int8PtrTy), bufferSize, fileIdx});
    }
    iBuilder->CreateBr(fullCondBlock);
    
    iBuilder->SetInsertPoint(fullCondBlock);
    PHINode * remainingBytes = iBuilder->CreatePHI(int64ty, 2, "remainingBytes");
    remainingBytes->addIncoming(initialBufferSize, initialBlock);
    PHINode * blockNo = iBuilder->CreatePHI(int64ty, 2, "blockNo");
    blockNo->addIncoming(initialBlockNo, initialBlock);
    
    Constant * const step = ConstantInt::get(int64ty, mBlockSize * (UTF_16 ? 2 : 1));
    Value * fullCondTest = iBuilder->CreateICmpULT(remainingBytes, step);
    iBuilder->CreateCondBr(fullCondTest, finalBlock, fullBodyBlock);

    // Full Block Pipeline loop
    iBuilder->SetInsertPoint(fullBodyBlock);
    
    Value * byteStreamPtr = ByteStream.getBlockPointer(blockNo);
    Value * basisBitsPtr = BasisBits.getBlockPointer(blockNo);
    Value * matchResultsPtr = MatchResults.getBlockPointer(blockNo);
    s2pk.createDoBlockCall(s2pInstance, {byteStreamPtr, basisBitsPtr});
    icgrepK.createDoBlockCall(icgrepInstance, {basisBitsPtr, matchResultsPtr});
    if (!CountOnly) {

        scanMatchK.createDoBlockCall(scanMatchInstance, {matchResultsPtr});
    }
    remainingBytes->addIncoming(iBuilder->CreateSub(remainingBytes, step), fullBodyBlock);
    blockNo->addIncoming(iBuilder->CreateAdd(blockNo, iBuilder->getInt64(1)), fullBodyBlock);
    iBuilder->CreateBr(fullCondBlock);

    iBuilder->SetInsertPoint(finalBlock);
    byteStreamPtr = ByteStream.getBlockPointer(blockNo);
    basisBitsPtr = BasisBits.getBlockPointer(blockNo);
    matchResultsPtr = MatchResults.getBlockPointer(blockNo);
    s2pk.createFinalBlockCall(s2pInstance, remainingBytes, {byteStreamPtr, basisBitsPtr});
    icgrepK.createFinalBlockCall(icgrepInstance, remainingBytes, {basisBitsPtr, matchResultsPtr});
    if (CountOnly) {
        Value * matchCount = icgrepK.createGetAccumulatorCall(icgrepInstance, "matchedLineCount");
        iBuilder->CreateRet(matchCount);
    }
    else {
        scanMatchK.createFinalBlockCall(scanMatchInstance, remainingBytes, {matchResultsPtr});
        iBuilder->CreateRetVoid();
    }
    
    //Linker L(*mMod);
    //L.linkInModule(std::move(s2pM));
    //L.linkInModule(std::move(scanMatchM));
    //L.linkInModule(std::move(icgrepM));


    return main;
}
