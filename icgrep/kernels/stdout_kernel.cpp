/*
 *  Copyright (c) 2016 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 */
#include "stdout_kernel.h"
#include <llvm/IR/Module.h>
#include <IR_Gen/idisa_builder.h>
#include <kernels/streamset.h>
namespace llvm { class Type; }

using namespace llvm;
using namespace parabix;

namespace kernel {

// Rather than using doBlock logic to write one block at a time, this custom
// doSegment method attempts to write the entire segment with a single write call.
// However, if the segment spans two memory areas (e.g., because of wraparound),
// then two write calls are made.
void StdOutKernel::generateDoSegmentMethod(Value *doFinal, const std::vector<Value *> &producerPos) {
    PointerType * i8PtrTy = iBuilder->getInt8PtrTy();

    Constant * blockItems = iBuilder->getSize(iBuilder->getBitBlockWidth() - 1);
    Constant * itemBytes = iBuilder->getSize(mCodeUnitWidth / 8);
    Value * processed = getProcessedItemCount("codeUnitBuffer");
    Value * itemsToDo = iBuilder->CreateSub(producerPos[0], processed);
    // There may be two memory areas if we are at the physical end of a circular buffer.
    const auto b  = getInputStreamSetBuffer("codeUnitBuffer");
    Value * wraparound = nullptr;
    if (isa<CircularBuffer>(b) || isa<CircularCopybackBuffer>(b)) {
        Value * instance = getStreamSetBufferPtr("codeUnitBuffer");
        Value * accessible = b->getLinearlyAccessibleItems(instance, processed);
        wraparound = iBuilder->CreateICmpULT(accessible, itemsToDo);
        itemsToDo = iBuilder->CreateSelect(wraparound, accessible, itemsToDo);
    }
    
    Value * byteOffset = iBuilder->CreateMul(iBuilder->CreateAnd(processed, blockItems), itemBytes);
    Value * bytePtr = iBuilder->CreatePointerCast(getInputStreamBlockPtr("codeUnitBuffer", iBuilder->getInt32(0)), i8PtrTy);
    bytePtr = iBuilder->CreateGEP(bytePtr, byteOffset);

    iBuilder->CreateWriteCall(iBuilder->getInt32(1), bytePtr, iBuilder->CreateMul(itemsToDo, itemBytes));

    processed = iBuilder->CreateAdd(processed, itemsToDo);
    setProcessedItemCount("codeUnitBuffer", processed);
    
    // Now we may process the second area (if required).
    if (isa<CircularBuffer>(b) || isa<CircularCopybackBuffer>(b)) {
        BasicBlock * wrapAroundWrite = CreateBasicBlock("wrapAroundWrite");
        BasicBlock * stdoutExit = CreateBasicBlock("stdoutExit");
        iBuilder->CreateCondBr(wraparound, wrapAroundWrite, stdoutExit);
        iBuilder->SetInsertPoint(wrapAroundWrite);
        
        // Calculate from the updated value of processed;
        byteOffset = iBuilder->CreateMul(iBuilder->CreateAnd(processed, blockItems), itemBytes);
        Value * bytePtr = iBuilder->CreatePointerCast(getInputStreamBlockPtr("codeUnitBuffer", iBuilder->getInt32(0)), i8PtrTy);
        bytePtr = iBuilder->CreateGEP(bytePtr, byteOffset);

        itemsToDo = iBuilder->CreateSub(producerPos[0], processed);
        iBuilder->CreateWriteCall(iBuilder->getInt32(1), bytePtr, iBuilder->CreateMul(itemsToDo, itemBytes));
        processed = iBuilder->CreateAdd(processed, itemsToDo);
        setProcessedItemCount("codeUnitBuffer", producerPos[0]);
        iBuilder->CreateBr(stdoutExit);
        iBuilder->SetInsertPoint(stdoutExit);
    }
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

    Value * IOstreamPtr = getScalarField("IOstreamPtr");
    Value * processed = getProcessedItemCount("codeUnitBuffer");
    Value * itemsToDo = iBuilder->CreateSub(producerPos[0], processed);
    // There may be two memory areas if we are at the physical end of a circular buffer.
    const auto b  = getInputStreamSetBuffer("codeUnitBuffer");
    Value * wraparound = nullptr;
    if (isa<CircularBuffer>(b) || isa<CircularCopybackBuffer>(b)) {
        Value * instance = getStreamSetBufferPtr("codeUnitBuffer");
        Value * accessible = b->getLinearlyAccessibleItems(instance, processed);
        wraparound = iBuilder->CreateICmpULT(accessible, itemsToDo);
        itemsToDo = iBuilder->CreateSelect(wraparound, accessible, itemsToDo);
    }
    
    Value * byteOffset = iBuilder->CreateMul(iBuilder->CreateURem(processed, blockItems), itemBytes);
    Value * bytePtr = iBuilder->CreatePointerCast(getInputStreamBlockPtr("codeUnitBuffer", iBuilder->getInt32(0)), i8PtrTy);
    bytePtr = iBuilder->CreateGEP(bytePtr, byteOffset);

    iBuilder->CreateFWriteCall(bytePtr, itemsToDo, itemBytes, IOstreamPtr);

    
    processed = iBuilder->CreateAdd(processed, itemsToDo);
    setProcessedItemCount("codeUnitBuffer", processed);
    
    // Now we may process the second area (if required).
    if (isa<CircularBuffer>(b) || isa<CircularCopybackBuffer>(b)) {
        BasicBlock * wrapAroundWrite = CreateBasicBlock("wrapAroundWrite");
        BasicBlock * checkFinal = CreateBasicBlock("checkFinal");
        iBuilder->CreateCondBr(wraparound, wrapAroundWrite, checkFinal);
        iBuilder->SetInsertPoint(wrapAroundWrite);
        
        // Calculate from the updated value of processed;
        byteOffset = iBuilder->CreateMul(iBuilder->CreateURem(processed, blockItems), itemBytes);
        Value * bytePtr = iBuilder->CreatePointerCast(getInputStreamBlockPtr("codeUnitBuffer", iBuilder->getInt32(0)), i8PtrTy);
        bytePtr = iBuilder->CreateGEP(bytePtr, byteOffset);
        itemsToDo = iBuilder->CreateSub(producerPos[0], processed);
        iBuilder->CreateFWriteCall(bytePtr, itemsToDo, itemBytes, IOstreamPtr);
        processed = iBuilder->CreateAdd(processed, itemsToDo);
        setProcessedItemCount("codeUnitBuffer", producerPos[0]);
        iBuilder->CreateBr(checkFinal);
        iBuilder->SetInsertPoint(checkFinal);
    }
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




