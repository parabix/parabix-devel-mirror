/*
 *  Copyright (c) 2016 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 */
#include "stdout_kernel.h"
#include <llvm/IR/Module.h>
#include <IR_Gen/idisa_builder.h>

using namespace llvm;

namespace kernel {

// The doBlock method is deprecated.   But in case it is used, just call doSegment with
// 1 as the number of blocks to do.
void StdOutKernel::generateDoBlockMethod() const {
    auto savePoint = iBuilder->saveIP();
    Module * m = iBuilder->getModule();
    Function * doBlockFunction = m->getFunction(mKernelName + doBlock_suffix);
    Function * doSegmentFunction = m->getFunction(mKernelName + doSegment_suffix);
    iBuilder->SetInsertPoint(BasicBlock::Create(iBuilder->getContext(), "entry", doBlockFunction, 0));
    Value * self = getParameter(doBlockFunction, "self");
    iBuilder->CreateCall(doSegmentFunction, {self, iBuilder->getSize(1)});
    iBuilder->CreateRetVoid();
    iBuilder->restoreIP(savePoint);
}
            
// Rather than using doBlock logic to write one block at a time, this custom
// doSegment method, writes the entire segment with a single write call.
void StdOutKernel::generateDoSegmentMethod() const {
    auto savePoint = iBuilder->saveIP();
    Module * m = iBuilder->getModule();
    Function * doSegmentFunction = m->getFunction(mKernelName + doSegment_suffix);
    Type * i8PtrTy = iBuilder->getInt8PtrTy();
    
    iBuilder->SetInsertPoint(BasicBlock::Create(iBuilder->getContext(), "entry", doSegmentFunction, 0));
    Constant * blockItems = iBuilder->getSize(iBuilder->getBitBlockWidth());
    Constant * itemBytes = iBuilder->getSize(mCodeUnitWidth/8);
    
    Function::arg_iterator args = doSegmentFunction->arg_begin();
    Value * self = &*(args++);
    Value * doFinal = &*(args++);
    Value * producerPos = &*(args++);
    ////iBuilder->CallPrintInt("blocksToDo", blocksToDo);
    Value * streamStructPtr = getStreamSetStructPtr(self, "codeUnitBuffer");
    //iBuilder->CallPrintInt("streamStructPtr", iBuilder->CreatePtrToInt(streamStructPtr, iBuilder->getInt64Ty()));

    //iBuilder->CallPrintInt("producerPos", producerPos);
    Value * processed = getProcessedItemCount(self, "codeUnitBuffer");
    Value * itemsToDo = iBuilder->CreateSub(producerPos, processed);
    
    Value * blockNo = getScalarField(self, blockNoScalar);
    Value * byteOffset = iBuilder->CreateMul(iBuilder->CreateURem(processed, blockItems), itemBytes);
    Value * bytePtr = getStreamView(i8PtrTy, self, "codeUnitBuffer", blockNo, byteOffset);
    iBuilder->CreateWriteCall(iBuilder->getInt32(1), bytePtr, iBuilder->CreateMul(itemsToDo, itemBytes));

    processed = iBuilder->CreateAdd(processed, itemsToDo);
    setProcessedItemCount(self, "codeUnitBuffer", processed);
    setScalarField(self, blockNoScalar, iBuilder->CreateUDiv(processed, blockItems));
    mStreamSetInputBuffers[0]->setConsumerPos(streamStructPtr, processed);

    iBuilder->CreateRetVoid();
    iBuilder->restoreIP(savePoint);
}

void StdOutKernel::generateFinalBlockMethod() const {
    auto savePoint = iBuilder->saveIP();
    Module * m = iBuilder->getModule();
    Function * finalBlockFunction = m->getFunction(mKernelName + finalBlock_suffix);
    Type * i8PtrTy = iBuilder->getInt8PtrTy();    
    iBuilder->SetInsertPoint(BasicBlock::Create(iBuilder->getContext(), "fb_flush", finalBlockFunction, 0));
    Constant * blockItems = iBuilder->getSize(iBuilder->getBitBlockWidth());
    Constant * itemBytes = iBuilder->getSize(mCodeUnitWidth/8);
    Value * self = getParameter(finalBlockFunction, "self");
    Value * streamStructPtr = getStreamSetStructPtr(self, "codeUnitBuffer");
    LoadInst * producerPos = iBuilder->CreateAtomicLoadAcquire(mStreamSetInputBuffers[0]->getProducerPosPtr(streamStructPtr));
    Value * processed = getProcessedItemCount(self, "codeUnitBuffer");
    Value * itemsAvail = iBuilder->CreateSub(producerPos, processed);
    Value * blockNo = getScalarField(self, blockNoScalar);
    Value * byteOffset = iBuilder->CreateMul(iBuilder->CreateURem(processed, blockItems), itemBytes);
    Value * bytePtr = getStreamView(i8PtrTy, self, "codeUnitBuffer", blockNo, byteOffset);
    iBuilder->CreateWriteCall(iBuilder->getInt32(1), bytePtr, iBuilder->CreateMul(itemsAvail, itemBytes));
    setProcessedItemCount(self, "codeUnitBuffer", producerPos);
    mStreamSetInputBuffers[0]->setConsumerPos(streamStructPtr, producerPos);
    setTerminationSignal(self);
    iBuilder->CreateRetVoid();
    iBuilder->restoreIP(savePoint);
}
}
