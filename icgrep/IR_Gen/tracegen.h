/*
 *  Copyright (c) 2017 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 */
#ifndef TRACEGEN_H
#define TRACEGEN_H

#include "idisa_builder.h"
#include <string>
#include <llvm/IR/Module.h>
#include <llvm/IR/Constants.h>
#include <llvm/IR/Intrinsics.h>
#include <llvm/IR/Function.h>



class TraceTool {
public:
    TraceTool(IDISA::IDISA_Builder * b, unsigned log2TraceBufSize = 16);
    
    unsigned declareTraceVar(std::string traceVarName);
    
    unsigned newTraceVar(std::string traceName);
    void addTraceEntry(unsigned traceVar, llvm::Value * traceVal);
    void createDumpTrace();
    
private:
    IDISA::IDISA_Builder * iBuilder;
    unsigned mLog2TraceBufSize;
    unsigned mTraceVarCount;
    std::vector<llvm::Value *> mTraceFormatString;
    llvm::Value * mTraceBufferPtr;
    llvm::Value * mTraceIndexPtr;
    llvm::Constant * mTraceIndexMask;
};

using namespace llvm;

TraceTool::TraceTool(IDISA::IDISA_Builder * b, unsigned log2TraceBufSize) :
    iBuilder(b),
    mLog2TraceBufSize(log2TraceBufSize),
    mTraceVarCount(0) {

    Type * entryType = StructType::get(iBuilder->getInt8Ty()->getPointerTo(), iBuilder->getSizeTy(), nullptr);
    Type * bufferType = ArrayType::get(entryType, 1 << mLog2TraceBufSize);
    mTraceBufferPtr = iBuilder->CreateAlloca(bufferType);
    mTraceIndexPtr = iBuilder->CreateAlloca(iBuilder->getInt32Ty());
    iBuilder->CreateStore(ConstantInt::getNullValue(iBuilder->getInt32Ty()), mTraceIndexPtr);
    mTraceIndexMask = ConstantInt::get(iBuilder->getInt32Ty(), (1 << mLog2TraceBufSize) - 1);
}

unsigned TraceTool::newTraceVar(std::string traceName) {
    std::string formatString = traceName + " = %" PRIx64 "\n";
    mTraceFormatString.push_back(iBuilder->CreateGlobalStringPtr(formatString.c_str()));
    return mTraceVarCount++;
}

void TraceTool::addTraceEntry(unsigned traceVar, llvm::Value * traceVal) {
    
    Value * traceIndex = iBuilder->CreateLoad(mTraceIndexPtr);
    Value * entryVarPtr = iBuilder->CreateGEP(mTraceBufferPtr, {iBuilder->getInt32(0), traceIndex, iBuilder->getInt32(0)});
    iBuilder->CreateStore(mTraceFormatString[traceVar], entryVarPtr);
    Value * entryValPtr = iBuilder->CreateGEP(mTraceBufferPtr, {iBuilder->getInt32(0), traceIndex, iBuilder->getInt32(1)});
    iBuilder->CreateStore(iBuilder->CreateZExt(traceVal, iBuilder->getSizeTy()), entryValPtr);
    iBuilder->CreateStore(iBuilder->CreateAnd(mTraceIndexMask, iBuilder->CreateAdd(traceIndex, iBuilder->getInt32(1))), mTraceIndexPtr);
}

void TraceTool::createDumpTrace() {
    Constant * traceBufSize = ConstantInt::get(iBuilder->getInt32Ty(), 1<<mLog2TraceBufSize);
    Function * printF = iBuilder->GetPrintf();
    BasicBlock * DumpEntryBlock = iBuilder->GetInsertBlock();
    Function * currentFn = DumpEntryBlock->getParent();
    BasicBlock * DumpTraceLoop = BasicBlock::Create(iBuilder->getContext(), "DumpTraceLoop", currentFn, 0);
    BasicBlock * DumpTraceExit = BasicBlock::Create(iBuilder->getContext(), "DumpTraceExit", currentFn, 0);
    
    Value * lastTraceIndex = iBuilder->CreateLoad(mTraceIndexPtr);
    Value * truncated = iBuilder->CreateICmpUGT(lastTraceIndex, traceBufSize);
    Value * firstDumpIndex = iBuilder->CreateSelect(truncated, iBuilder->CreateSub(lastTraceIndex, traceBufSize), ConstantInt::getNullValue(iBuilder->getInt32Ty()));
    
    iBuilder->CreateBr(DumpTraceLoop);
    iBuilder->SetInsertPoint(DumpTraceLoop);
    PHINode * loopIndex = iBuilder->CreatePHI(iBuilder->getInt32Ty(), 2);
    loopIndex->addIncoming(firstDumpIndex, DumpEntryBlock);
    
    Value * entryVarPtr = iBuilder->CreateGEP(mTraceBufferPtr, {iBuilder->getInt32(0), loopIndex, iBuilder->getInt32(0)});
    Value * formatString = iBuilder->CreateLoad(entryVarPtr);
    Value * entryValPtr = iBuilder->CreateGEP(mTraceBufferPtr, {iBuilder->getInt32(0), loopIndex, iBuilder->getInt32(1)});
    Value * entryVal = iBuilder->CreateLoad(entryValPtr);
    iBuilder->CreateCall(printF, {formatString, entryVal});
    
    Value * nextIndex = iBuilder->CreateAnd(iBuilder->CreateAdd(loopIndex, iBuilder->getInt32(1)), mTraceIndexMask);
    loopIndex->addIncoming(nextIndex, DumpTraceLoop);
    Value * atLastTraceIndex = iBuilder->CreateICmpEQ(loopIndex, lastTraceIndex);
    iBuilder->CreateCondBr(atLastTraceIndex, DumpTraceExit, DumpTraceLoop);
    iBuilder->SetInsertPoint(DumpTraceExit);
}

#endif
