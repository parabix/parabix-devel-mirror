/*
 *  Copyright (c) 2017 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 */
#include <kernels/stdin_kernel.h>
#include <llvm/IR/Module.h>
#include <kernels/kernel.h>
#include <IR_Gen/idisa_builder.h>

using namespace llvm;

inline static unsigned ceil_log2(const unsigned v) {
    assert ("log2(0) is undefined!" && v != 0);
    return 32 - __builtin_clz(v - 1);
}

namespace kernel {

void StdInKernel::generateDoSegmentMethod(Value * /* doFinal */, const std::vector<Value *> & /* producerPos */) {

    BasicBlock * setTermination = CreateBasicBlock("setTermination");
    BasicBlock * stdInExit = CreateBasicBlock("stdInExit");
    ConstantInt * segmentItems = iBuilder->getSize(mSegmentBlocks * iBuilder->getBitBlockWidth() / mCodeUnitWidth);
    ConstantInt * segmentItems2 = iBuilder->getSize(2 * mSegmentBlocks * iBuilder->getBitBlockWidth() / mCodeUnitWidth);
    // on the first segment, we buffer twice the data necessary to ensure that we can safely check for a non-LF line break
    Value * itemsRead = getProducedItemCount("codeUnitBuffer");
    Value * isFirst = iBuilder->CreateICmpEQ(itemsRead, iBuilder->getSize(0));
    Value * itemsToRead = iBuilder->CreateSelect(isFirst, segmentItems2, segmentItems);

    Value * segmentBytes = reserveItemCount("codeUnitBuffer", itemsToRead);
    Value * bytePtr =  getOutputStreamBlockPtr("codeUnitBuffer", iBuilder->getInt32(0));
    bytePtr = iBuilder->CreatePointerCast(bytePtr, iBuilder->getInt8PtrTy());
    Value * bytesRead = iBuilder->CreateReadCall(iBuilder->getInt32(STDIN_FILENO), bytePtr, segmentBytes);
    itemsRead = iBuilder->CreateAdd(itemsRead, iBuilder->CreateUDiv(bytesRead, iBuilder->getSize(mCodeUnitWidth / 8)));

    iBuilder->CreateCondBr(iBuilder->CreateICmpEQ(bytesRead, iBuilder->getSize(0)), setTermination, stdInExit);
    iBuilder->SetInsertPoint(setTermination);
    setTerminationSignal();
    iBuilder->CreateBr(stdInExit);
    iBuilder->SetInsertPoint(stdInExit);

    setProducedItemCount("codeUnitBuffer", itemsRead);
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
    
void FileSource::generateDoSegmentMethod(Value * /* doFinal */, const std::vector<Value *> & /* producerPos */) {

    BasicBlock * closeFile = CreateBasicBlock("closeFile");
    BasicBlock * fileSourceExit = CreateBasicBlock("fileSourceExit");
    Constant * itemBytes = iBuilder->getSize(mCodeUnitWidth/8);
    
    Value * produced = getProducedItemCount("codeUnitBuffer");
    Value * bytePtr = getOutputStreamBlockPtr("codeUnitBuffer", iBuilder->getInt32(0));
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
