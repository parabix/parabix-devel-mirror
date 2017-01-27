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
    /* unused Value * doFinal = &*(args++);*/ args++;
    Value * producerPos = &*(args++);
    Value * processed = getProcessedItemCount(self, "codeUnitBuffer");
    Value * itemsToDo = iBuilder->CreateSub(producerPos, processed);
    
    Value * blockNo = getScalarField(self, blockNoScalar);
    Value * byteOffset = iBuilder->CreateMul(iBuilder->CreateURem(processed, blockItems), itemBytes);
    Value * bytePtr = getStreamView(i8PtrTy, self, "codeUnitBuffer", blockNo, byteOffset);
    iBuilder->CreateWriteCall(iBuilder->getInt32(1), bytePtr, iBuilder->CreateMul(itemsToDo, itemBytes));

    processed = iBuilder->CreateAdd(processed, itemsToDo);
    setProcessedItemCount(self, "codeUnitBuffer", processed);
    setScalarField(self, blockNoScalar, iBuilder->CreateUDiv(processed, blockItems));

    iBuilder->CreateRetVoid();
    iBuilder->restoreIP(savePoint);
}

StdOutKernel::StdOutKernel(IDISA::IDISA_Builder * iBuilder, unsigned codeUnitWidth)
: SegmentOrientedKernel(iBuilder, "stdout", {Binding{iBuilder->getStreamSetTy(1, codeUnitWidth), "codeUnitBuffer"}}, {}, {}, {}, {})
, mCodeUnitWidth(codeUnitWidth) {
    setNoTerminateAttribute(true);
}

void FileSink::generateInitMethod() const {
    auto savePoint = iBuilder->saveIP();
    Module * const m = iBuilder->getModule();
    Function * initFunction = m->getFunction(mKernelName + init_suffix);
    iBuilder->SetInsertPoint(BasicBlock::Create(iBuilder->getContext(), "Init_entry", initFunction, 0));    
    BasicBlock * setTerminationOnFailure = BasicBlock::Create(iBuilder->getContext(), "setTerminationOnFailure", initFunction, 0);
    BasicBlock * fileSinkInitExit = BasicBlock::Create(iBuilder->getContext(), "fileSinkInitExit", initFunction, 0);
    Function::arg_iterator args = initFunction->arg_begin();
    Value * self = &*(args++);
    iBuilder->CreateStore(ConstantAggregateZero::get(mKernelStateType), self);
    for (auto binding : mScalarInputs) {
        Value * param = &*(args++);
        Value * ptr = iBuilder->CreateGEP(self, {iBuilder->getInt32(0), getScalarIndex(binding.name)});
        iBuilder->CreateStore(param, ptr);
    }
    Value * handle = iBuilder->CreateFOpenCall(getScalarField(self, "fileName"), iBuilder->CreateGlobalStringPtr("w"));
    setScalarField(self, "IOstreamPtr", handle);
    Value * failure = iBuilder->CreateICmpEQ(iBuilder->CreatePtrToInt(handle, iBuilder->getSizeTy()), iBuilder->getSize(0));
    iBuilder->CreateCondBr(failure, setTerminationOnFailure, fileSinkInitExit);
    iBuilder->SetInsertPoint(setTerminationOnFailure);
    setTerminationSignal(self);
    iBuilder->CreateBr(fileSinkInitExit);
    iBuilder->SetInsertPoint(fileSinkInitExit);
    iBuilder->CreateRetVoid();
    iBuilder->restoreIP(savePoint);
}
    
    

void FileSink::generateDoSegmentMethod() const {
    auto savePoint = iBuilder->saveIP();
    Module * m = iBuilder->getModule();
    Function * doSegmentFunction = m->getFunction(mKernelName + doSegment_suffix);
    Type * i8PtrTy = iBuilder->getInt8PtrTy();
    
    iBuilder->SetInsertPoint(BasicBlock::Create(iBuilder->getContext(), "entry", doSegmentFunction, 0));
    BasicBlock * closeFile = BasicBlock::Create(iBuilder->getContext(), "closeFile", doSegmentFunction, 0);
    BasicBlock * fileOutExit = BasicBlock::Create(iBuilder->getContext(), "fileOutExit", doSegmentFunction, 0);
    Constant * blockItems = iBuilder->getSize(iBuilder->getBitBlockWidth());
    Constant * itemBytes = iBuilder->getSize(mCodeUnitWidth/8);
    
    Function::arg_iterator args = doSegmentFunction->arg_begin();
    Value * self = &*(args++);
    Value * doFinal = &*(args++);
    Value * producerPos = &*(args++);
    Value * processed = getProcessedItemCount(self, "codeUnitBuffer");
    Value * itemsToDo = iBuilder->CreateSub(producerPos, processed);
    Value * IOstreamPtr = getScalarField(self, "IOstreamPtr");
    
    Value * blockNo = getScalarField(self, blockNoScalar);
    Value * byteOffset = iBuilder->CreateMul(iBuilder->CreateURem(processed, blockItems), itemBytes);
    Value * bytePtr = getStreamView(i8PtrTy, self, "codeUnitBuffer", blockNo, byteOffset);
    iBuilder->CreateFWriteCall(bytePtr, itemsToDo, itemBytes, IOstreamPtr);
    
    processed = iBuilder->CreateAdd(processed, itemsToDo);
    setProcessedItemCount(self, "codeUnitBuffer", processed);
    setScalarField(self, blockNoScalar, iBuilder->CreateUDiv(processed, blockItems));
    iBuilder->CreateCondBr(doFinal, closeFile, fileOutExit);
    
    iBuilder->SetInsertPoint(closeFile);
    iBuilder->CreateFCloseCall(IOstreamPtr);
    iBuilder->CreateBr(fileOutExit);
    
    iBuilder->SetInsertPoint(fileOutExit);
    iBuilder->CreateRetVoid();
    iBuilder->restoreIP(savePoint);

}

FileSink::FileSink(IDISA::IDISA_Builder * iBuilder, unsigned codeUnitWidth)
: SegmentOrientedKernel(iBuilder, "filesink", {Binding{iBuilder->getStreamSetTy(1, codeUnitWidth), "codeUnitBuffer"}}, {},
                {Binding{iBuilder->getInt8PtrTy(), "fileName"}}, {}, {Binding{iBuilder->getFILEptrTy(), "IOstreamPtr"}})
, mCodeUnitWidth(codeUnitWidth) {
}

}



