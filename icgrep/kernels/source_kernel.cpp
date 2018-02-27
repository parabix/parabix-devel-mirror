/*
 *  Copyright (c) 2017 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 */
#include "source_kernel.h"
#include <kernels/kernel_builder.h>
#include <kernels/streamset.h>
#include <llvm/IR/Module.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <toolchain/toolchain.h>

using namespace llvm;

uint64_t file_size(const uint32_t fd) {
    struct stat st;
    if (LLVM_UNLIKELY(fstat(fd, &st) != 0)) {
        st.st_size = 0;
    }
    return st.st_size;
}

namespace kernel {

/// MMAP SOURCE KERNEL

Function * MMapSourceKernel::linkFileSizeMethod(const std::unique_ptr<kernel::KernelBuilder> & kb) {
    return kb->LinkFunction("file_size", &file_size);
}

void MMapSourceKernel::generateInitializeMethod(Function * const fileSizeMethod, const unsigned codeUnitWidth, const std::unique_ptr<KernelBuilder> & kb) {

    BasicBlock * const emptyFile = kb->CreateBasicBlock("emptyFile");
    BasicBlock * const nonEmptyFile = kb->CreateBasicBlock("NonEmptyFile");
    BasicBlock * const exit = kb->CreateBasicBlock("Exit");
    IntegerType * const sizeTy = kb->getSizeTy();
    Value * const fd = kb->getScalarField("fileDescriptor");
    assert (fileSizeMethod);
    Value * fileSize = kb->CreateZExtOrTrunc(kb->CreateCall(fileSizeMethod, fd), sizeTy);
    kb->CreateLikelyCondBr(kb->CreateIsNotNull(fileSize), nonEmptyFile, emptyFile);

    kb->SetInsertPoint(nonEmptyFile);
    PointerType * const codeUnitPtrTy = kb->getIntNTy(codeUnitWidth)->getPointerTo();
    Value * const fileBuffer = kb->CreatePointerCast(kb->CreateFileSourceMMap(fd, fileSize), codeUnitPtrTy);
    kb->setScalarField("buffer", fileBuffer);    
    kb->setBaseAddress("sourceBuffer", fileBuffer);    
    kb->CreateMAdvise(fileBuffer, fileSize, CBuilder::ADVICE_WILLNEED);
    if (codeUnitWidth > 8) {
        fileSize = kb->CreateUDiv(fileSize, kb->getSize(codeUnitWidth / 8));
    }
    kb->setBufferedSize("sourceBuffer", fileSize);
    kb->setScalarField("fileSize", fileSize);
    kb->setProducedItemCount("sourceBuffer", fileSize);
    kb->setCapacity("sourceBuffer", fileSize);
    kb->CreateBr(exit);

    kb->SetInsertPoint(emptyFile);
    kb->setTerminationSignal();
    kb->CreateBr(exit);

    kb->SetInsertPoint(exit);
}

void MMapSourceKernel::generateDoSegmentMethod(const unsigned codeUnitWidth, const std::unique_ptr<KernelBuilder> & kb) {

    BasicBlock * dropPages = kb->CreateBasicBlock("dropPages");
    BasicBlock * checkRemaining = kb->CreateBasicBlock("checkRemaining");
    BasicBlock * setTermination = kb->CreateBasicBlock("setTermination");
    BasicBlock * exit = kb->CreateBasicBlock("mmapSourceExit");

    Value * const fileSize = kb->getScalarField("fileSize");
    Constant * const pageSize = kb->getSize(getpagesize());

    Value * consumed = kb->getConsumedItemCount("sourceBuffer");
    consumed = kb->CreateMul(consumed, kb->getSize(codeUnitWidth / 8));
    consumed = kb->CreateAnd(consumed, ConstantExpr::getNeg(pageSize));
    Value * const consumedBuffer = kb->getRawOutputPointer("sourceBuffer", consumed);
    Value * const readableBuffer = kb->getScalarField("buffer");
    Value * const unnecessaryBytes = kb->CreatePtrDiff(consumedBuffer, readableBuffer);

    // avoid calling madvise unless an actual page table change could occur
    kb->CreateLikelyCondBr(kb->CreateIsNotNull(unnecessaryBytes), dropPages, checkRemaining);

    kb->SetInsertPoint(dropPages);
    // instruct the OS that it can safely drop any fully consumed pages
    kb->CreateMAdvise(readableBuffer, unnecessaryBytes, CBuilder::ADVICE_DONTNEED);
    kb->setScalarField("buffer", kb->CreateGEP(readableBuffer, unnecessaryBytes));
    kb->CreateBr(checkRemaining);

    // determine whether or not we've exhausted the file buffer
    kb->SetInsertPoint(checkRemaining);
    Value * const remaining = kb->CreateSub(fileSize, consumed);
    Value * const lastPage = kb->CreateICmpULE(remaining, pageSize);
    kb->CreateUnlikelyCondBr(lastPage, setTermination, exit);

    kb->SetInsertPoint(setTermination);
    kb->setTerminationSignal();
    kb->CreateBr(exit);

    // finally, set the "produced" count to reflect current position in the file
    kb->SetInsertPoint(exit);
}

void MMapSourceKernel::unmapSourceBuffer(const std::unique_ptr<KernelBuilder> & kb) {
    kb->CreateMUnmap(kb->getBaseAddress("sourceBuffer"), kb->getBufferedSize("sourceBuffer"));
}

/// READ SOURCE KERNEL

void ReadSourceKernel::generateInitializeMethod(const unsigned codeUnitWidth, const unsigned stride, const std::unique_ptr<KernelBuilder> & b) {
    const unsigned pageSize = getpagesize();
    const auto bufferSize = std::max(pageSize * 8, codegen::SegmentSize * stride  * 4);
    ConstantInt * const bufferItems = b->getSize(bufferSize);
    const auto codeUnitSize = codeUnitWidth / 8;
    ConstantInt * const bufferBytes = b->getSize(bufferSize * codeUnitSize);
    PointerType * const codeUnitPtrTy = b->getIntNTy(codeUnitWidth)->getPointerTo();
    Value * const buffer = b->CreatePointerCast(b->CreateCacheAlignedMalloc(bufferBytes), codeUnitPtrTy);
    b->setBaseAddress("sourceBuffer", buffer);
    b->setScalarField("buffer", buffer);
    b->setCapacity("sourceBuffer", bufferItems);
}

void ReadSourceKernel::generateDoSegmentMethod(const unsigned codeUnitWidth, const unsigned stride, const std::unique_ptr<KernelBuilder> & b) {

    const unsigned pageSize = getpagesize();
    ConstantInt * const itemsToRead = b->getSize(std::max(pageSize, codegen::SegmentSize * stride * 2));
    ConstantInt * const codeUnitBytes = b->getSize(codeUnitWidth / 8);
    ConstantInt * const itemsPerSegment = b->getSize(codegen::SegmentSize * stride);

    BasicBlock * const entry = b->GetInsertBlock();
    BasicBlock * const checkData = b->CreateBasicBlock("CheckData");
    BasicBlock * const moveData = b->CreateBasicBlock("MoveData");
    BasicBlock * const prepareBuffer = b->CreateBasicBlock("PrepareBuffer");
    BasicBlock * const readData = b->CreateBasicBlock("ReadData");
    BasicBlock * const setTermination = b->CreateBasicBlock("SetTermination");
    BasicBlock * const readExit = b->CreateBasicBlock("ReadExit");

    // Do we have enough unread data to support one segment?
    Value * const produced = b->getProducedItemCount("sourceBuffer");
    Value * const buffered = b->getBufferedSize("sourceBuffer");
    Value * const itemsPending = b->CreateAdd(produced, itemsPerSegment);

    b->CreateLikelyCondBr(b->CreateICmpULT(itemsPending, buffered), readExit, checkData);

    // Can we append to our existing buffer without impacting any subsequent kernel?
    b->SetInsertPoint(checkData);
    Value * const capacity = b->getCapacity("sourceBuffer");
    Value * const readEnd = b->getRawOutputPointer("sourceBuffer", b->CreateAdd(buffered, itemsToRead));
    Value * const baseBuffer = b->getScalarField("buffer");
    Value * const bufferLimit = b->CreateGEP(baseBuffer, capacity);
    b->CreateLikelyCondBr(b->CreateICmpULE(readEnd, bufferLimit), readData, moveData);

    // First wait on any consumers to finish processing then check how much data has been consumed.
    b->SetInsertPoint(moveData);
    b->CreateConsumerWait();

    // Then determine how much data has been consumed and how much needs to be copied back, noting
    // that our "unproduced" data must be block aligned.
    BasicBlock * const copyBack = b->CreateBasicBlock("CopyBack");
    BasicBlock * const expandAndCopyBack = b->CreateBasicBlock("ExpandAndCopyBack");

    const auto blockSize = b->getBitBlockWidth() / 8;
    Constant * const blockSizeAlignmentMask = ConstantExpr::getNeg(b->getSize(blockSize));
    Value * const consumed = b->getConsumedItemCount("sourceBuffer");
    Value * const offset = b->CreateAnd(consumed, blockSizeAlignmentMask);
    Value * const unreadData = b->getRawOutputPointer("sourceBuffer", offset);
    Value * const remainingItems = b->CreateSub(buffered, offset);
    Value * const remainingBytes = b->CreateMul(remainingItems, codeUnitBytes);

    // Have we consumed enough data that we can safely copy back the unconsumed data without needing a temporary buffer?
    Value * const canCopy = b->CreateICmpULT(b->CreateGEP(baseBuffer, remainingItems), b->getRawOutputPointer("sourceBuffer", offset));
    b->CreateLikelyCondBr(canCopy, copyBack, expandAndCopyBack);

    // If so, just copy the data ...
    b->SetInsertPoint(copyBack);
    b->CreateMemCpy(baseBuffer, unreadData, remainingBytes, blockSize);
    b->CreateBr(prepareBuffer);

    // Otherwise, allocate a buffer with twice the capacity and copy the unconsumed data back into it
    b->SetInsertPoint(expandAndCopyBack);
    Value * const expandedCapacity = b->CreateShl(capacity, 1);
    Value * const expandedBytes = b->CreateMul(expandedCapacity, codeUnitBytes);
    Value * const expandedBuffer = b->CreatePointerCast(b->CreateCacheAlignedMalloc(expandedBytes), unreadData->getType());
    b->CreateMemCpy(expandedBuffer, unreadData, remainingBytes, blockSize);
    b->setScalarField("buffer", expandedBuffer);
    b->setCapacity("sourceBuffer", expandedCapacity);
    b->CreateFree(baseBuffer);
    b->CreateBr(prepareBuffer);

    b->SetInsertPoint(prepareBuffer);
    PHINode * newBaseBuffer = b->CreatePHI(baseBuffer->getType(), 2);
    newBaseBuffer->addIncoming(baseBuffer, copyBack);
    newBaseBuffer->addIncoming(expandedBuffer, expandAndCopyBack);
    b->setBaseAddress("sourceBuffer", b->CreateGEP(newBaseBuffer, b->CreateNeg(offset)));
    b->CreateBr(readData);

    // Regardless of whether we're simply appending data or had to allocate a new buffer, read a new page
    // of data into the input source buffer. If we fail to read a full page ...
    b->SetInsertPoint(readData);
    Value * const sourceBuffer = b->getRawOutputPointer("sourceBuffer", buffered);
    Value * const fd = b->getScalarField("fileDescriptor");
    Constant * const bytesToRead = ConstantExpr::getMul(itemsToRead, codeUnitBytes);
    Value * const bytesRead = b->CreateReadCall(fd, sourceBuffer, bytesToRead);
    Value * const itemsRead = b->CreateUDiv(bytesRead, codeUnitBytes);
    Value * const itemsBuffered = b->CreateAdd(buffered, itemsRead);
    b->setBufferedSize("sourceBuffer", itemsBuffered);
    b->CreateUnlikelyCondBr(b->CreateICmpULT(itemsBuffered, itemsPending), setTermination, readExit);

    // ... set the termination signal.
    b->SetInsertPoint(setTermination);
    Value * const bytesToZero = b->CreateMul(b->CreateSub(itemsPending, itemsBuffered), codeUnitBytes);
    b->CreateMemZero(b->getRawOutputPointer("sourceBuffer", itemsBuffered), bytesToZero);
    b->setTerminationSignal();
    b->CreateBr(readExit);

    readExit->moveAfter(setTermination);
    b->SetInsertPoint(readExit);
    PHINode * const itemsProduced = b->CreatePHI(itemsPending->getType(), 3);
    itemsProduced->addIncoming(itemsPending, entry);
    itemsProduced->addIncoming(itemsPending, readData);
    itemsProduced->addIncoming(itemsBuffered, setTermination);
    b->setProducedItemCount("sourceBuffer", itemsProduced);
}

void ReadSourceKernel::freeBuffer(const std::unique_ptr<KernelBuilder> & kb) {
    kb->CreateFree(kb->getScalarField("buffer"));
}

/// Hybrid MMap/Read source kernel

void FDSourceKernel::linkExternalMethods(const std::unique_ptr<kernel::KernelBuilder> & kb) {
    mFileSizeFunction = MMapSourceKernel::linkFileSizeMethod(kb);
}

void FDSourceKernel::generateFinalizeMethod(const std::unique_ptr<KernelBuilder> & kb) {
    BasicBlock * finalizeRead = kb->CreateBasicBlock("finalizeRead");
    BasicBlock * finalizeMMap = kb->CreateBasicBlock("finalizeMMap");
    BasicBlock * finalizeDone = kb->CreateBasicBlock("finalizeDone");
    // if the fileDescriptor is 0, the file is stdin, use readSource kernel logic, otherwise use mmap logic.
    kb->CreateCondBr(kb->CreateICmpEQ(kb->getScalarField("fileDescriptor"), kb->getInt32(STDIN_FILENO)), finalizeRead, finalizeMMap);
    kb->SetInsertPoint(finalizeRead);
    ReadSourceKernel::freeBuffer(kb);
    kb->CreateBr(finalizeDone);
    kb->SetInsertPoint(finalizeMMap);
    MMapSourceKernel::unmapSourceBuffer(kb);
    kb->CreateBr(finalizeDone);
    kb->SetInsertPoint(finalizeDone);
}

void FDSourceKernel::generateInitializeMethod(const std::unique_ptr<KernelBuilder> & kb) {
    BasicBlock * initializeRead = kb->CreateBasicBlock("initializeRead");
    BasicBlock * initializeMMap = kb->CreateBasicBlock("initializeMMap");
    BasicBlock * initializeDone = kb->CreateBasicBlock("initializeDone");
    // if the fileDescriptor is 0, the file is stdin, use readSource kernel logic, otherwise use MMap logic.
    kb->CreateCondBr(kb->CreateICmpEQ(kb->getScalarField("fileDescriptor"), kb->getInt32(STDIN_FILENO)), initializeRead, initializeMMap);
    kb->SetInsertPoint(initializeRead);
    ReadSourceKernel::generateInitializeMethod(mCodeUnitWidth, getStride(), kb);
    kb->CreateBr(initializeDone);
    kb->SetInsertPoint(initializeMMap);
    MMapSourceKernel::generateInitializeMethod(mFileSizeFunction, mCodeUnitWidth, kb);
    kb->CreateBr(initializeDone);
    kb->SetInsertPoint(initializeDone);
}

void FDSourceKernel::generateDoSegmentMethod(const std::unique_ptr<KernelBuilder> & kb) {
    BasicBlock * DoSegmentRead = kb->CreateBasicBlock("DoSegmentRead");
    BasicBlock * DoSegmentMMap = kb->CreateBasicBlock("DoSegmentMMap");
    BasicBlock * DoSegmentDone = kb->CreateBasicBlock("DoSegmentDone");
    // if the fileDescriptor is 0, the file is stdin, use readSource kernel logic, otherwise use MMap logic.
    kb->CreateCondBr(kb->CreateICmpEQ(kb->getScalarField("fileDescriptor"), kb->getInt32(STDIN_FILENO)), DoSegmentRead, DoSegmentMMap);
    kb->SetInsertPoint(DoSegmentRead);
    ReadSourceKernel::generateDoSegmentMethod(mCodeUnitWidth, getStride(), kb);
    kb->CreateBr(DoSegmentDone);
    kb->SetInsertPoint(DoSegmentMMap);
    MMapSourceKernel::generateDoSegmentMethod(mCodeUnitWidth, kb);
    kb->CreateBr(DoSegmentDone);
    kb->SetInsertPoint(DoSegmentDone);
}


/// MEMORY SOURCE KERNEL

void MemorySourceKernel::generateInitializeMethod(const std::unique_ptr<KernelBuilder> & kb) {
    Value * const fileSource = kb->getScalarField("fileSource");
    kb->setBaseAddress("sourceBuffer", fileSource);
    Value * const fileSize = kb->getScalarField("fileSize");
    Value * const fileItems = kb->CreateUDiv(fileSize, kb->getSize(mCodeUnitWidth / 8));
    kb->setBufferedSize("sourceBuffer", fileItems);
    kb->setCapacity("sourceBuffer", fileItems);
    kb->setProducedItemCount("sourceBuffer", fileItems);
    kb->setTerminationSignal();
}

void MemorySourceKernel::generateDoSegmentMethod(const std::unique_ptr<KernelBuilder> & kb) {

}

MMapSourceKernel::MMapSourceKernel(const std::unique_ptr<kernel::KernelBuilder> & b, const unsigned codeUnitWidth)
: SegmentOrientedKernel("mmap_source@" + std::to_string(codeUnitWidth),
{},
{Binding{b->getStreamSetTy(1, codeUnitWidth), "sourceBuffer", FixedRate(), Deferred()}},
{Binding{b->getInt32Ty(), "fileDescriptor"}},
{Binding{b->getSizeTy(), "fileSize"}}, {Binding{b->getIntNTy(codeUnitWidth)->getPointerTo(), "buffer"}})
, mCodeUnitWidth(codeUnitWidth)
, mFileSizeFunction(nullptr) {
    addAttribute(MustExplicitlyTerminate());
}


ReadSourceKernel::ReadSourceKernel(const std::unique_ptr<kernel::KernelBuilder> & b, const unsigned codeUnitWidth)
: SegmentOrientedKernel("read_source" + std::to_string(codegen::SegmentSize) + "@" + std::to_string(codeUnitWidth)
, {}
, {Binding{b->getStreamSetTy(1, codeUnitWidth), "sourceBuffer", FixedRate(), Deferred()}}
, {Binding{b->getInt32Ty(), "fileDescriptor"}}
, {}
, {Binding{b->getIntNTy(codeUnitWidth)->getPointerTo(), "buffer"}})
, mCodeUnitWidth(codeUnitWidth) {
    addAttribute(MustExplicitlyTerminate());
}


FDSourceKernel::FDSourceKernel(const std::unique_ptr<kernel::KernelBuilder> & kb, const unsigned codeUnitWidth)
: SegmentOrientedKernel("FD_source@" + std::to_string(codeUnitWidth)
, {}
, {Binding{kb->getStreamSetTy(1, codeUnitWidth), "sourceBuffer", FixedRate(), Deferred()}}
, {Binding{kb->getInt32Ty(), "fileDescriptor"}}
, {}
, {Binding{kb->getIntNTy(codeUnitWidth)->getPointerTo(), "buffer"}, Binding{kb->getSizeTy(), "fileSize"}})
, mCodeUnitWidth(codeUnitWidth)
, mFileSizeFunction(nullptr) {
    addAttribute(MustExplicitlyTerminate());
}

MemorySourceKernel::MemorySourceKernel(const std::unique_ptr<kernel::KernelBuilder> & kb, Type * const type, const unsigned codeUnitWidth)
: SegmentOrientedKernel("memory_source",
    {},
    {Binding{kb->getStreamSetTy(1, codeUnitWidth), "sourceBuffer"}},
    {Binding{cast<PointerType>(type), "fileSource"}, Binding{kb->getSizeTy(), "fileSize"}}, {}, {})
, mCodeUnitWidth(codeUnitWidth) {
    addAttribute(MustExplicitlyTerminate());
}

}
