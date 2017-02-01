/*
 *  Copyright (c) 2016 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 */
#include "stdout_kernel.h"
#include <llvm/IR/Module.h>
#include <IR_Gen/idisa_builder.h>
#include <kernels/streamset.h>
#include <stdio.h>
// #include <llvm/IR/Type.h>
namespace llvm { class Type; }

using namespace llvm;

namespace kernel {
            
// Rather than using doBlock logic to write one block at a time, this custom
// doSegment method, writes the entire segment with a single write call.
void StdOutKernel::generateDoSegmentMethod(Value *doFinal, const std::vector<Value *> &producerPos) {

    PointerType * i8PtrTy = iBuilder->getInt8PtrTy();

    Constant * blockItems = iBuilder->getSize(iBuilder->getBitBlockWidth());
    Constant * itemBytes = iBuilder->getSize(mCodeUnitWidth/8);
    
    Value * processed = getProcessedItemCount("codeUnitBuffer");
    Value * itemsToDo = iBuilder->CreateSub(producerPos[0], processed);
    
    Value * blockNo = getBlockNo();
    Value * byteOffset = iBuilder->CreateMul(iBuilder->CreateURem(processed, blockItems), itemBytes);
    Value * bytePtr = getStreamView(i8PtrTy, "codeUnitBuffer", blockNo, byteOffset);
    iBuilder->CreateWriteCall(iBuilder->getInt32(1), bytePtr, iBuilder->CreateMul(itemsToDo, itemBytes));

    processed = iBuilder->CreateAdd(processed, itemsToDo);
    setProcessedItemCount("codeUnitBuffer", processed);
    setBlockNo(iBuilder->CreateUDiv(processed, blockItems));

}

StdOutKernel::StdOutKernel(IDISA::IDISA_Builder * iBuilder, unsigned codeUnitWidth)
: SegmentOrientedKernel(iBuilder, "stdout", {Binding{iBuilder->getStreamSetTy(1, codeUnitWidth), "codeUnitBuffer"}}, {}, {}, {}, {})
, mCodeUnitWidth(codeUnitWidth) {
    setNoTerminateAttribute(true);
}

void FileSink::generateInitMethod() {
    BasicBlock * setTerminationOnFailure = CreateBasicBlock("setTerminationOnFailure");
    BasicBlock * fileSinkInitExit = CreateBasicBlock("fileSinkInitExit");
    Value * handle = iBuilder->CreateFOpenCall(getScalarField("fileName"), iBuilder->CreateGlobalStringPtr("w"));
    setScalarField("IOstreamPtr", handle);
    Value * failure = iBuilder->CreateICmpEQ(iBuilder->CreatePtrToInt(handle, iBuilder->getSizeTy()), iBuilder->getSize(0));
    iBuilder->CreateCondBr(failure, setTerminationOnFailure, fileSinkInitExit);
    iBuilder->SetInsertPoint(setTerminationOnFailure);
    setTerminationSignal();
    iBuilder->CreateBr(fileSinkInitExit);
    iBuilder->SetInsertPoint(fileSinkInitExit);
}

void FileSink::generateDoSegmentMethod(Value *doFinal, const std::vector<Value *> &producerPos) {
    PointerType * i8PtrTy = iBuilder->getInt8PtrTy();
    
    BasicBlock * closeFile = CreateBasicBlock("closeFile");
    BasicBlock * fileOutExit = CreateBasicBlock("fileOutExit");
    Constant * blockItems = iBuilder->getSize(iBuilder->getBitBlockWidth());
    Constant * itemBytes = iBuilder->getSize(mCodeUnitWidth/8);
    
    Value * processed = getProcessedItemCount("codeUnitBuffer");
    Value * itemsToDo = iBuilder->CreateSub(producerPos[0], processed);
    Value * IOstreamPtr = getScalarField("IOstreamPtr");
    
    Value * blockNo = getBlockNo();
    Value * byteOffset = iBuilder->CreateMul(iBuilder->CreateURem(processed, blockItems), itemBytes);
    Value * bytePtr = getStreamView(i8PtrTy, "codeUnitBuffer", blockNo, byteOffset);
    iBuilder->CreateFWriteCall(bytePtr, itemsToDo, itemBytes, IOstreamPtr);
    
    processed = iBuilder->CreateAdd(processed, itemsToDo);
    setProcessedItemCount("codeUnitBuffer", processed);
    setBlockNo(iBuilder->CreateUDiv(processed, blockItems));
    iBuilder->CreateCondBr(doFinal, closeFile, fileOutExit);
    
    iBuilder->SetInsertPoint(closeFile);
    iBuilder->CreateFCloseCall(IOstreamPtr);
    iBuilder->CreateBr(fileOutExit);
    
    iBuilder->SetInsertPoint(fileOutExit);
}

FileSink::FileSink(IDISA::IDISA_Builder * iBuilder, unsigned codeUnitWidth)
: SegmentOrientedKernel(iBuilder, "filesink", {Binding{iBuilder->getStreamSetTy(1, codeUnitWidth), "codeUnitBuffer"}}, {},
                {Binding{iBuilder->getInt8PtrTy(), "fileName"}}, {}, {Binding{iBuilder->getFILEptrTy(), "IOstreamPtr"}})
, mCodeUnitWidth(codeUnitWidth) {
}

}



