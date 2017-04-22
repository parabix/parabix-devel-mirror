/*
 *  Copyright (c) 2017 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 */
#include <kernels/stdin_kernel.h>
#include <llvm/IR/Module.h>
#include <kernels/kernel.h>
#include <IR_Gen/idisa_builder.h>
#include <llvm/Support/raw_ostream.h>

using namespace llvm;

inline static size_t round_up_to_nearest(const size_t x, const size_t y) {
    return (((x - 1) | (y - 1)) + 1);
}

namespace kernel {

void StdInKernel::generateDoSegmentMethod() {

    BasicBlock * const entryBlock = iBuilder->GetInsertBlock();
    BasicBlock * const readBlock = CreateBasicBlock("ReadMoreData");
    BasicBlock * const setTermination = CreateBasicBlock("SetTermination");
    BasicBlock * const stdInExit = CreateBasicBlock("StdInExit");

    ConstantInt * const segmentSize = iBuilder->getSize(mSegmentBlocks * iBuilder->getBitBlockWidth());
    Value * bufferedSize = getBufferedSize("InputStream");
    Value * const itemsAlreadyRead = getProducedItemCount("InputStream");
    Value * const bytesAlreadyRead = iBuilder->CreateMul(itemsAlreadyRead, iBuilder->getSize(mCodeUnitWidth / 8));
    Value * unreadSize = iBuilder->CreateSub(bufferedSize, bytesAlreadyRead);

    Value * const exaustedBuffer = iBuilder->CreateICmpULT(unreadSize, segmentSize);
    iBuilder->CreateUnlikelyCondBr(exaustedBuffer, readBlock, stdInExit);

    iBuilder->SetInsertPoint(readBlock);

//    Value * consumed = getConsumedItemCount("InputStream");
//    Value * remaining = iBuilder->CreateSub(itemsAlreadyRead, consumed);

    // how many pages are required to have enough data for the segment plus one overflow block?
    const auto PageAlignedSegmentSize = round_up_to_nearest((mSegmentBlocks + 1) * iBuilder->getBitBlockWidth() * (mCodeUnitWidth / 8), getpagesize());
    ConstantInt * const bytesToRead = iBuilder->getSize(PageAlignedSegmentSize);
    reserveBytes("InputStream", bytesToRead);
    BasicBlock * const readExit = iBuilder->GetInsertBlock();


    Value * const ptr = getRawOutputPointer("InputStream", iBuilder->getInt32(0), bufferedSize);
    Value * const bytePtr = iBuilder->CreatePointerCast(ptr, iBuilder->getInt8PtrTy());
    Value * const bytesRead = iBuilder->CreateReadCall(iBuilder->getInt32(STDIN_FILENO), bytePtr, bytesToRead);

    unreadSize = iBuilder->CreateAdd(unreadSize, bytesRead);
    bufferedSize = iBuilder->CreateAdd(bufferedSize, bytesRead);
    setBufferedSize("InputStream", bufferedSize);

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

    setProducedItemCount("InputStream", itemsRead);
}

StdInKernel::StdInKernel(IDISA::IDISA_Builder * iBuilder, unsigned blocksPerSegment, unsigned codeUnitWidth)
: SegmentOrientedKernel(iBuilder, "Parabix:stdin_source", {}, {Binding{iBuilder->getStreamSetTy(1, codeUnitWidth), "InputStream"}}, {}, {}, {})
, mSegmentBlocks(blocksPerSegment)
, mCodeUnitWidth(codeUnitWidth) {
    
}

void FileSourceKernel::generateDoSegmentMethod() {

    BasicBlock * entryBlock = iBuilder->GetInsertBlock();
    BasicBlock * setTermination = CreateBasicBlock("setTermination");
    BasicBlock * mmapSourceExit = CreateBasicBlock("mmapSourceExit");
    ConstantInt * segmentItems = iBuilder->getSize(mSegmentBlocks * iBuilder->getBitBlockWidth());
    Value * fileItems = getScalarField("fileSize");
    if (mCodeUnitWidth > 8) {
        fileItems = iBuilder->CreateUDiv(fileItems, iBuilder->getSize(mCodeUnitWidth / 8));
    }
    Value * produced = getProducedItemCount("sourceBuffer");
    produced = iBuilder->CreateAdd(produced, segmentItems);
    Value * lessThanFullSegment = iBuilder->CreateICmpULT(fileItems, produced);
    iBuilder->CreateCondBr(lessThanFullSegment, setTermination, mmapSourceExit);
    iBuilder->SetInsertPoint(setTermination);
    setTerminationSignal();
    iBuilder->CreateBr(mmapSourceExit);

    iBuilder->SetInsertPoint(mmapSourceExit);

    PHINode * itemsRead = iBuilder->CreatePHI(produced->getType(), 2);
    itemsRead->addIncoming(produced, entryBlock);
    itemsRead->addIncoming(fileItems, setTermination);
    setProducedItemCount("sourceBuffer", itemsRead);
}

void FileSourceKernel::generateInitializeMethod() {
    setBaseAddress("sourceBuffer", getScalarField("fileSource"));
    setBufferedSize("sourceBuffer", getScalarField("fileSize"));
}

FileSourceKernel::FileSourceKernel(IDISA::IDISA_Builder * iBuilder, Type * fileSourceTy, unsigned blocksPerSegment, unsigned codeUnitWidth)
: SegmentOrientedKernel(iBuilder, "Parabix:file_source",
    {},
    {Binding{iBuilder->getStreamSetTy(1, codeUnitWidth), "sourceBuffer"}},
    {Binding{fileSourceTy, "fileSource"}, Binding{iBuilder->getSizeTy(), "fileSize"}}, {}, {})
, mSegmentBlocks(blocksPerSegment)
, mCodeUnitWidth(codeUnitWidth) {

}



}
