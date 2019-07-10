#include "tracegen.h"

#include <llvm/IR/Module.h>
#include <llvm/IR/Constants.h>
#include <llvm/IR/Intrinsics.h>
#include <llvm/IR/Function.h>

using namespace llvm;

TraceTool::TraceTool(IDISA::IDISA_Builder * b, unsigned log2TraceBufSize)
: iBuilder(b)
, mLog2TraceBufSize(log2TraceBufSize)
, mTraceVarCount(0) {

    llvm::Type * entryType = llvm::StructType::get(iBuilder->getInt8Ty()->getPointerTo(), iBuilder->getSizeTy());
    llvm::Type * bufferType = llvm::ArrayType::get(entryType, 1 << mLog2TraceBufSize);
    mTraceBufferPtr = iBuilder->CreateAlloca(bufferType);
    mTraceIndexPtr = iBuilder->CreateAlloca(iBuilder->getInt32Ty());
    iBuilder->CreateStore(llvm::ConstantInt::getNullValue(iBuilder->getInt32Ty()), mTraceIndexPtr);
    mTraceIndexMask = llvm::ConstantInt::get(iBuilder->getInt32Ty(), (1 << mLog2TraceBufSize) - 1);
}

unsigned TraceTool::newTraceVar(std::string traceName) {
    std::string formatString = traceName + " = %" PRIx64 "\n";
    mTraceFormatString.push_back(iBuilder->GetString(formatString.c_str()));
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
    Value * atLastTraceIndex = iBuilder->CreateICmpEQ(nextIndex, lastTraceIndex);
    iBuilder->CreateCondBr(atLastTraceIndex, DumpTraceExit, DumpTraceLoop);
    iBuilder->SetInsertPoint(DumpTraceExit);
}
