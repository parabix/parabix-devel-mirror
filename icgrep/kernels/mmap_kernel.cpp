/*
 *  Copyright (c) 2017 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 */
#include "mmap_kernel.h"
#include <llvm/IR/Module.h>
#include <IR_Gen/idisa_builder.h>
#include <kernels/streamset.h>

using namespace llvm;

namespace kernel {

void MMapSourceKernel::generateInitializeMethod() {
    BasicBlock * emptyFile = CreateBasicBlock("EmptyFile");
    BasicBlock * nonEmptyFile = CreateBasicBlock("NonEmptyFile");
    BasicBlock * exit = CreateBasicBlock("Exit");

    Value * fd = getScalarField("fileDescriptor");
    Value * fileSize = iBuilder->CreateFileSize(fd);
    if (mCodeUnitWidth > 8) {
        fileSize = iBuilder->CreateUDiv(fileSize, iBuilder->getSize(mCodeUnitWidth / 8));
    }
    Value * const isEmpty = iBuilder->CreateICmpEQ(fileSize, ConstantInt::getNullValue(fileSize->getType()));
    iBuilder->CreateUnlikelyCondBr(isEmpty, emptyFile, nonEmptyFile);
    // we cannot mmap a 0 length file; just create a 1-page sized fake file buffer for simplicity
    iBuilder->SetInsertPoint(emptyFile);
    Constant * pageSize = ConstantInt::get(fileSize->getType(), getpagesize());
    Value * fakeFileBuffer = iBuilder->CreateAnonymousMMap(pageSize);
    iBuilder->CreateBr(exit);

    iBuilder->SetInsertPoint(nonEmptyFile);
    Value * fileBackedBuffer = iBuilder->CreateFileSourceMMap(fd, fileSize);
    iBuilder->CreateBr(exit);

    iBuilder->SetInsertPoint(exit);
    PHINode * buffer = iBuilder->CreatePHI(fileBackedBuffer->getType(), 2);
    buffer->addIncoming(fakeFileBuffer, emptyFile);
    buffer->addIncoming(fileBackedBuffer, nonEmptyFile);
    PHINode * size = iBuilder->CreatePHI(fileSize->getType(), 2);
    size->addIncoming(pageSize, emptyFile);
    size->addIncoming(fileSize, nonEmptyFile);

    setBaseAddress("sourceBuffer", buffer);
    setBufferedSize("sourceBuffer", size);
    setScalarField("readableBuffer", buffer);
    setScalarField("fileSize", fileSize);
    iBuilder->CreateMAdvise(buffer, fileSize, CBuilder::ADVICE_WILLNEED);

}

void MMapSourceKernel::generateDoSegmentMethod() {

    BasicBlock * dropPages = CreateBasicBlock("dropPages");
    BasicBlock * produceData = CreateBasicBlock("produceData");
    BasicBlock * setTermination = CreateBasicBlock("setTermination");
    BasicBlock * mmapSourceExit = CreateBasicBlock("mmapSourceExit");

    // instruct the OS that it can safely drop any fully consumed pages
    Value * consumed = getConsumedItemCount("sourceBuffer");
    Type * const consumedTy = consumed->getType();
    Type * const voidPtrTy = iBuilder->getVoidPtrTy();

    // multiply the consumed count by the code unit size then mask off any partial pages
    if (mCodeUnitWidth > 8) {
        consumed = iBuilder->CreateMul(consumed, iBuilder->getSize(mCodeUnitWidth / 8));
    }
    const auto pageSize = getpagesize();
    if (LLVM_LIKELY((pageSize & (pageSize - 1)) == 0)) {
        consumed = iBuilder->CreateAnd(consumed, ConstantExpr::getNot(ConstantInt::get(consumedTy, pageSize - 1)));
    } else {
        consumed = iBuilder->CreateSub(consumed, iBuilder->CreateURem(consumed, ConstantInt::get(consumedTy, pageSize)));
    }
    Value * sourceBuffer = getBaseAddress("sourceBuffer");
    sourceBuffer = iBuilder->CreatePtrToInt(sourceBuffer, consumedTy);
    Value * consumedBuffer = iBuilder->CreateAdd(sourceBuffer, consumed);
    Value * readableBuffer = getScalarField("readableBuffer");
    readableBuffer = iBuilder->CreatePtrToInt(readableBuffer, consumedTy);
    Value * unnecessaryBytes = iBuilder->CreateSub(consumedBuffer, readableBuffer);
    // avoid calling madvise unless an actual page table change could occur
    Value * hasPagesToDrop = iBuilder->CreateICmpEQ(unnecessaryBytes, ConstantInt::getNullValue(unnecessaryBytes->getType()));
    iBuilder->CreateLikelyCondBr(hasPagesToDrop, produceData, dropPages);

    iBuilder->SetInsertPoint(dropPages);
    iBuilder->CreateMAdvise(iBuilder->CreateIntToPtr(readableBuffer, voidPtrTy), unnecessaryBytes, CBuilder::ADVICE_DONTNEED);    
    readableBuffer = iBuilder->CreateIntToPtr(iBuilder->CreateAdd(readableBuffer, unnecessaryBytes), voidPtrTy);
    setScalarField("readableBuffer", readableBuffer);
    iBuilder->CreateBr(produceData);

    // determine whether or not we've exhausted the file buffer
    iBuilder->SetInsertPoint(produceData);
    ConstantInt * segmentItems = iBuilder->getSize(mSegmentBlocks * iBuilder->getBitBlockWidth());
    Value * const fileSize = getScalarField("fileSize");
    Value * produced = getProducedItemCount("sourceBuffer");
    produced = iBuilder->CreateAdd(produced, segmentItems);

    Value * lessThanFullSegment = iBuilder->CreateICmpULT(fileSize, produced);
    iBuilder->CreateCondBr(lessThanFullSegment, setTermination, mmapSourceExit);
    iBuilder->SetInsertPoint(setTermination);

    setTerminationSignal();
    iBuilder->CreateBr(mmapSourceExit);

    // finally, set the "produced" count to reflect current position in the file
    iBuilder->SetInsertPoint(mmapSourceExit);
    PHINode * itemsRead = iBuilder->CreatePHI(produced->getType(), 2);
    itemsRead->addIncoming(produced, produceData);
    itemsRead->addIncoming(fileSize, setTermination);
    setProducedItemCount("sourceBuffer", itemsRead);
}

void MMapSourceKernel::generateFinalizeMethod() {
    iBuilder->CreateMUnmap(getBaseAddress("sourceBuffer"), getBufferedSize("sourceBuffer"));
}

MMapSourceKernel::MMapSourceKernel(IDISA::IDISA_Builder * iBuilder, unsigned blocksPerSegment, unsigned codeUnitWidth)
    : SegmentOrientedKernel(iBuilder, "Parabix:mmap_source" + std::to_string(blocksPerSegment) + "@" + std::to_string(codeUnitWidth),
    {},
    {Binding{iBuilder->getStreamSetTy(1, codeUnitWidth), "sourceBuffer"}},
    {Binding{iBuilder->getInt32Ty(), "fileDescriptor"}}, {Binding{iBuilder->getSizeTy(), "fileSize"}}, {Binding{iBuilder->getVoidPtrTy(), "readableBuffer"}})
, mSegmentBlocks(blocksPerSegment)
, mCodeUnitWidth(codeUnitWidth) {

}

}
