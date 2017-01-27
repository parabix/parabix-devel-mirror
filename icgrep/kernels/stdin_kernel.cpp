/*
 *  Copyright (c) 2017 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 */
#include <kernels/stdin_kernel.h>
#include <llvm/IR/Module.h>
#include <kernels/kernel.h>
#include <IR_Gen/idisa_builder.h>

using namespace llvm;

namespace kernel {
    
void StdInKernel::generateDoSegmentMethod() const {
    auto savePoint = iBuilder->saveIP();
    Module * m = iBuilder->getModule();
    Function * doSegmentFunction = m->getFunction(mKernelName + doSegment_suffix);
    Type * i8PtrTy = iBuilder->getInt8PtrTy();
    
    iBuilder->SetInsertPoint(BasicBlock::Create(iBuilder->getContext(), "entry", doSegmentFunction, 0));
    BasicBlock * setTermination = BasicBlock::Create(iBuilder->getContext(), "setTermination", doSegmentFunction, 0);
    BasicBlock * stdInExit = BasicBlock::Create(iBuilder->getContext(), "stdInExit", doSegmentFunction, 0);
    Constant * blockItems = ConstantInt::get(iBuilder->getSizeTy(), iBuilder->getBitBlockWidth());
    Constant * itemBytes = ConstantInt::get(iBuilder->getSizeTy(), mCodeUnitWidth/8);
    Constant * segmentBytes = ConstantInt::get(iBuilder->getSizeTy(),
                                               mSegmentBlocks * iBuilder->getBitBlockWidth() * mCodeUnitWidth/8);
    Constant * stdin_fileno = iBuilder->getInt32(STDIN_FILENO);
    Function::arg_iterator args = doSegmentFunction->arg_begin();
    Value * self = &*(args++);
    
    Value * produced = getProducedItemCount(self, "codeUnitBuffer");
    Value * blockNo = iBuilder->CreateUDiv(produced, blockItems);
    Value * byteOffset = iBuilder->CreateMul(iBuilder->CreateURem(produced, blockItems), itemBytes);
    Value * bytePtr = getStreamView(i8PtrTy, self, "codeUnitBuffer", blockNo, byteOffset);
    
    Value * nRead = iBuilder->CreateReadCall(stdin_fileno, bytePtr, segmentBytes);
    Value * bytesRead = iBuilder->CreateSelect(iBuilder->CreateICmpSLT(nRead, iBuilder->getSize(0)), iBuilder->getSize(0), nRead);
    produced = iBuilder->CreateAdd(produced, iBuilder->CreateUDiv(bytesRead, itemBytes));
    setProducedItemCount(self, "codeUnitBuffer", produced);
    Value * lessThanFullSegment = iBuilder->CreateICmpULT(bytesRead, segmentBytes);
    iBuilder->CreateCondBr(lessThanFullSegment, setTermination, stdInExit);
    iBuilder->SetInsertPoint(setTermination);
    setTerminationSignal(self);
    iBuilder->CreateBr(stdInExit);
    
    iBuilder->SetInsertPoint(stdInExit);
    iBuilder->CreateRetVoid();
    iBuilder->restoreIP(savePoint);
    
}

StdInKernel::StdInKernel(IDISA::IDISA_Builder * iBuilder, unsigned blocksPerSegment, unsigned codeUnitWidth)
: SegmentOrientedKernel(iBuilder, "stdin_source", {}, {Binding{iBuilder->getStreamSetTy(1, codeUnitWidth), "codeUnitBuffer"}}, {}, {}, {})
, mSegmentBlocks(blocksPerSegment)
, mCodeUnitWidth(codeUnitWidth) {
    
}

void FileSource::generateInitMethod() const {
    auto savePoint = iBuilder->saveIP();
    Module * const m = iBuilder->getModule();
    Function * initFunction = m->getFunction(mKernelName + init_suffix);
    iBuilder->SetInsertPoint(BasicBlock::Create(iBuilder->getContext(), "Init_entry", initFunction, 0));
    BasicBlock * setTerminationOnFailure = BasicBlock::Create(iBuilder->getContext(), "setTerminationOnFailure", initFunction, 0);
    BasicBlock * fileSourceInitExit = BasicBlock::Create(iBuilder->getContext(), "fileSourceInitExit", initFunction, 0);
    Function::arg_iterator args = initFunction->arg_begin();
    Value * self = &*(args++);
    iBuilder->CreateStore(ConstantAggregateZero::get(mKernelStateType), self);
    for (auto binding : mScalarInputs) {
        Value * param = &*(args++);
        Value * ptr = iBuilder->CreateGEP(self, {iBuilder->getInt32(0), getScalarIndex(binding.name)});
        iBuilder->CreateStore(param, ptr);
    }
    Value * handle = iBuilder->CreateFOpenCall(getScalarField(self, "fileName"), iBuilder->CreateGlobalStringPtr("r"));
    setScalarField(self, "IOstreamPtr", handle);
    Value * failure = iBuilder->CreateICmpEQ(iBuilder->CreatePtrToInt(handle, iBuilder->getSizeTy()), iBuilder->getSize(0));
    iBuilder->CreateCondBr(failure, setTerminationOnFailure, fileSourceInitExit);
    iBuilder->SetInsertPoint(setTerminationOnFailure);
    setTerminationSignal(self);
    iBuilder->CreateBr(fileSourceInitExit);
    iBuilder->SetInsertPoint(fileSourceInitExit);
    iBuilder->CreateRetVoid();
    iBuilder->restoreIP(savePoint);
}
    
void FileSource::generateDoSegmentMethod() const {
    auto savePoint = iBuilder->saveIP();
    Module * m = iBuilder->getModule();
    Function * doSegmentFunction = m->getFunction(mKernelName + doSegment_suffix);
    Type * i8PtrTy = iBuilder->getInt8PtrTy();
    
    iBuilder->SetInsertPoint(BasicBlock::Create(iBuilder->getContext(), "entry", doSegmentFunction, 0));
    BasicBlock * closeFile = BasicBlock::Create(iBuilder->getContext(), "closeFile", doSegmentFunction, 0);
    BasicBlock * fileSourceExit = BasicBlock::Create(iBuilder->getContext(), "fileSourceExit", doSegmentFunction, 0);
    Constant * blockItems = iBuilder->getSize(iBuilder->getBitBlockWidth());
    Constant * itemBytes = iBuilder->getSize(mCodeUnitWidth/8);
    
    Function::arg_iterator args = doSegmentFunction->arg_begin();
    Value * self = &*(args++);
    // ignore: Value * doFinal = &*(args++);
    Value * produced = getProducedItemCount(self, "codeUnitBuffer");
    Value * blockNo = iBuilder->CreateUDiv(produced, blockItems);
    Value * byteOffset = iBuilder->CreateMul(iBuilder->CreateURem(produced, blockItems), itemBytes);
    Value * bytePtr = getStreamView(i8PtrTy, self, "codeUnitBuffer", blockNo, byteOffset);
    Value * IOstreamPtr = getScalarField(self, "IOstreamPtr");
    Value * itemsToDo = iBuilder->getSize(mSegmentBlocks * iBuilder->getBitBlockWidth());
    Value * nRead = iBuilder->CreateFReadCall(bytePtr, itemsToDo, itemBytes, IOstreamPtr);
    produced = iBuilder->CreateAdd(produced, nRead);
    setProducedItemCount(self, "codeUnitBuffer", produced);
    Value * lessThanFullSegment = iBuilder->CreateICmpULT(nRead, itemsToDo);
    iBuilder->CreateCondBr(lessThanFullSegment, closeFile, fileSourceExit);

    iBuilder->SetInsertPoint(closeFile);
    iBuilder->CreateFCloseCall(IOstreamPtr);
    setTerminationSignal(self);
    iBuilder->CreateBr(fileSourceExit);
    
    iBuilder->SetInsertPoint(fileSourceExit);
    iBuilder->CreateRetVoid();
    iBuilder->restoreIP(savePoint);
    
}
    
FileSource::FileSource(IDISA::IDISA_Builder * iBuilder, unsigned blocksPerSegment, unsigned codeUnitWidth)
: SegmentOrientedKernel(iBuilder, "filesink", {Binding{iBuilder->getStreamSetTy(1, codeUnitWidth), "codeUnitBuffer"}}, {},
                {Binding{iBuilder->getInt8PtrTy(), "fileName"}}, {}, {Binding{iBuilder->getFILEptrTy(), "IOstreamPtr"}})
, mSegmentBlocks(blocksPerSegment)
, mCodeUnitWidth(codeUnitWidth) {
}

}
