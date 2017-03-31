/*
 *  Copyright (c) 2017 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 */
#include <kernels/stdin_kernel.h>
#include <llvm/IR/Module.h>
#include <kernels/kernel.h>
#include <IR_Gen/idisa_builder.h>

using namespace llvm;

inline static size_t round_up_to_nearest(const size_t x, const size_t y) {
    return (((x - 1) | (y - 1)) + 1);
}

namespace kernel {

void StdInKernel::generateDoSegmentMethod(Value * /* doFinal */, const std::vector<Value *> & /* producerPos */) {

    BasicBlock * const entryBlock = iBuilder->GetInsertBlock();
    BasicBlock * const readBlock = CreateBasicBlock("ReadMoreData");
    BasicBlock * const setTermination = CreateBasicBlock("SetTermination");
    BasicBlock * const stdInExit = CreateBasicBlock("StdInExit");

    ConstantInt * const segmentSize = iBuilder->getSize(mSegmentBlocks * iBuilder->getBitBlockWidth());
    Value * bufferedSize = getScalarField("BufferedSize");
    Value * const itemsAlreadyRead = getProducedItemCount("codeUnitBuffer");
    Value * const bytesAlreadyRead = iBuilder->CreateMul(itemsAlreadyRead, iBuilder->getSize(mCodeUnitWidth / 8));
    Value * unreadSize = iBuilder->CreateSub(bufferedSize, bytesAlreadyRead);
    Value * const exaustedBuffer = iBuilder->CreateICmpULT(unreadSize, segmentSize);
    iBuilder->CreateUnlikelyCondBr(exaustedBuffer, readBlock, stdInExit);

    iBuilder->SetInsertPoint(readBlock);
    // how many pages are required to have enough data for the segment plus one overflow block?
    const auto PageAlignedSegmentSize = round_up_to_nearest((mSegmentBlocks + 1) * iBuilder->getBitBlockWidth() * (mCodeUnitWidth / 8), getpagesize());
    ConstantInt * const bytesToRead = iBuilder->getSize(PageAlignedSegmentSize);
    reserveBytes("codeUnitBuffer", bytesToRead);
    BasicBlock * const readExit = iBuilder->GetInsertBlock();

    Value * const ptr = getRawOutputPointer("codeUnitBuffer", iBuilder->getInt32(0), bufferedSize);
    Value * const bytePtr = iBuilder->CreatePointerCast(ptr, iBuilder->getInt8PtrTy());
    Value * const bytesRead = iBuilder->CreateReadCall(iBuilder->getInt32(STDIN_FILENO), bytePtr, bytesToRead);

    unreadSize = iBuilder->CreateAdd(unreadSize, bytesRead);
    bufferedSize = iBuilder->CreateAdd(bufferedSize, bytesRead);
    setScalarField("BufferedSize", bufferedSize);
    iBuilder->CreateUnlikelyCondBr(iBuilder->CreateICmpULT(unreadSize, segmentSize), setTermination, stdInExit);

    iBuilder->SetInsertPoint(setTermination);
    Value * const itemsRemaining = iBuilder->CreateUDiv(unreadSize, iBuilder->getSize(mCodeUnitWidth / 8));
    setTerminationSignal();
    iBuilder->CreateBr(stdInExit);

    stdInExit->moveAfter(iBuilder->GetInsertBlock());

    iBuilder->SetInsertPoint(stdInExit);
    PHINode * const produced = iBuilder->CreatePHI(itemsAlreadyRead->getType(), 3);

    produced->addIncoming(segmentSize, entryBlock);
    produced->addIncoming(segmentSize, readExit);
    produced->addIncoming(itemsRemaining, setTermination);
    Value * const itemsRead = iBuilder->CreateAdd(itemsAlreadyRead, produced);

    setProducedItemCount("codeUnitBuffer", itemsRead);
}

StdInKernel::StdInKernel(IDISA::IDISA_Builder * iBuilder, unsigned blocksPerSegment, unsigned codeUnitWidth)
: SegmentOrientedKernel(iBuilder, "stdin_source", {}, {Binding{iBuilder->getStreamSetTy(1, codeUnitWidth), "codeUnitBuffer"}}, {}, {}, {Binding{iBuilder->getSizeTy(), "BufferedSize"}})
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
