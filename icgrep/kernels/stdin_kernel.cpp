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
    
void StdInKernel::generateDoSegmentMethod(Value *doFinal, const std::vector<Value *> &producerPos) {

    BasicBlock * setTermination = CreateBasicBlock("setTermination");
    BasicBlock * stdInExit = CreateBasicBlock("stdInExit");
//    ConstantInt * blockItems = iBuilder->getSize(iBuilder->getBitBlockWidth());
    ConstantInt * itemBytes = iBuilder->getSize(mCodeUnitWidth/8);
    ConstantInt * segmentBytes = iBuilder->getSize(mSegmentBlocks * iBuilder->getBitBlockWidth() * mCodeUnitWidth/8);
    ConstantInt * stdin_fileno = iBuilder->getInt32(STDIN_FILENO);
    Value * produced = getProducedItemCount("codeUnitBuffer");
//    Value * byteOffset = iBuilder->CreateMul(iBuilder->CreateURem(produced, blockItems), itemBytes);
//    Value * bytePtr = getRawItemPointer("codeUnitBuffer", iBuilder->getInt32(0), produced);
    Value * bytePtr = getOutputStream("codeUnitBuffer", iBuilder->getInt32(0));
    bytePtr = iBuilder->CreatePointerCast(bytePtr, iBuilder->getInt8PtrTy());


    
    Value * nRead = iBuilder->CreateReadCall(stdin_fileno, bytePtr, segmentBytes);
    Value * bytesRead = iBuilder->CreateSelect(iBuilder->CreateICmpSLT(nRead, iBuilder->getSize(0)), iBuilder->getSize(0), nRead);
    produced = iBuilder->CreateAdd(produced, iBuilder->CreateUDiv(bytesRead, itemBytes));
    setProducedItemCount("codeUnitBuffer", produced);
    Value * lessThanFullSegment = iBuilder->CreateICmpULT(bytesRead, segmentBytes);
    iBuilder->CreateCondBr(lessThanFullSegment, setTermination, stdInExit);
    iBuilder->SetInsertPoint(setTermination);
    setTerminationSignal();
    iBuilder->CreateBr(stdInExit);
    
    iBuilder->SetInsertPoint(stdInExit);

    
}

StdInKernel::StdInKernel(IDISA::IDISA_Builder * iBuilder, unsigned blocksPerSegment, unsigned codeUnitWidth)
: SegmentOrientedKernel(iBuilder, "stdin_source", {}, {Binding{iBuilder->getStreamSetTy(1, codeUnitWidth), "codeUnitBuffer"}}, {}, {}, {})
, mSegmentBlocks(blocksPerSegment)
, mCodeUnitWidth(codeUnitWidth) {
    
}

void FileSource::generateInitMethod() {
    BasicBlock * setTerminationOnFailure = CreateBasicBlock("setTerminationOnFailure");
    BasicBlock * fileSourceInitExit = CreateBasicBlock("fileSourceInitExit");
    Value * handle = iBuilder->CreateFOpenCall(getScalarField("fileName"), iBuilder->CreateGlobalStringPtr("r"));
    setScalarField("IOstreamPtr", handle);
    Value * failure = iBuilder->CreateICmpEQ(iBuilder->CreatePtrToInt(handle, iBuilder->getSizeTy()), iBuilder->getSize(0));
    iBuilder->CreateCondBr(failure, setTerminationOnFailure, fileSourceInitExit);
    iBuilder->SetInsertPoint(setTerminationOnFailure);
    setTerminationSignal();
    iBuilder->CreateBr(fileSourceInitExit);
    iBuilder->SetInsertPoint(fileSourceInitExit);
}
    
void FileSource::generateDoSegmentMethod(Value * doFinal, const std::vector<Value *> & producerPos) {

    BasicBlock * closeFile = CreateBasicBlock("closeFile");
    BasicBlock * fileSourceExit = CreateBasicBlock("fileSourceExit");
    Constant * itemBytes = iBuilder->getSize(mCodeUnitWidth/8);
    
    Value * produced = getProducedItemCount("codeUnitBuffer");
    Value * bytePtr = getOutputStream("codeUnitBuffer", iBuilder->getInt32(0));
    bytePtr = iBuilder->CreatePointerCast(bytePtr, iBuilder->getInt8PtrTy());

    Value * IOstreamPtr = getScalarField("IOstreamPtr");
    Value * itemsToDo = iBuilder->getSize(mSegmentBlocks * iBuilder->getBitBlockWidth());
    Value * nRead = iBuilder->CreateFReadCall(bytePtr, itemsToDo, itemBytes, IOstreamPtr);
    produced = iBuilder->CreateAdd(produced, nRead);
    setProducedItemCount("codeUnitBuffer", produced);
    Value * lessThanFullSegment = iBuilder->CreateICmpULT(nRead, itemsToDo);
    iBuilder->CreateCondBr(lessThanFullSegment, closeFile, fileSourceExit);

    iBuilder->SetInsertPoint(closeFile);
    iBuilder->CreateFCloseCall(IOstreamPtr);
    setTerminationSignal();
    iBuilder->CreateBr(fileSourceExit);
    
    iBuilder->SetInsertPoint(fileSourceExit);
    
}
    
FileSource::FileSource(IDISA::IDISA_Builder * iBuilder, unsigned blocksPerSegment, unsigned codeUnitWidth)
: SegmentOrientedKernel(iBuilder, "filesink", {Binding{iBuilder->getStreamSetTy(1, codeUnitWidth), "codeUnitBuffer"}}, {},
                {Binding{iBuilder->getInt8PtrTy(), "fileName"}}, {}, {Binding{iBuilder->getFILEptrTy(), "IOstreamPtr"}})
, mSegmentBlocks(blocksPerSegment)
, mCodeUnitWidth(codeUnitWidth) {
}

}
