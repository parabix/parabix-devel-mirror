/*
 *  Copyright (c) 2017 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 */
#include "source_kernel.h"
#include <kernels/kernel_builder.h>
#include <kernels/streamset.h>
#include <sys/stat.h>
#include <fcntl.h>

using namespace llvm;

inline static size_t round_up_to_nearest(const size_t x, const size_t y) {
    return (((x - 1) | (y - 1)) + 1);
}

uint64_t file_size(const uint32_t fd) {
    struct stat st;
    if (LLVM_UNLIKELY(fstat(fd, &st) != 0)) {
        st.st_size = 0;
    }
    return st.st_size;
}

namespace kernel {

/// MMAP SOURCE KERNEL

void MMapSourceKernel::linkExternalMethods(const std::unique_ptr<kernel::KernelBuilder> & iBuilder) {
    mFileSizeFunction = iBuilder->LinkFunction("file_size", &file_size);
}

void MMapSourceKernel::generateInitializeMethod(const std::unique_ptr<KernelBuilder> & iBuilder) {
    BasicBlock * const emptyFile = iBuilder->CreateBasicBlock("EmptyFile");
    BasicBlock * const nonEmptyFile = iBuilder->CreateBasicBlock("NonEmptyFile");
    BasicBlock * const exit = iBuilder->CreateBasicBlock("Exit");
    IntegerType * const sizeTy = iBuilder->getSizeTy();
    assert (iBuilder->getKernel() == this);
    Value * const fd = iBuilder->getScalarField("fileDescriptor");
    assert (mFileSizeFunction);
    Value * fileSize = iBuilder->CreateCall(mFileSizeFunction, fd);
    fileSize = iBuilder->CreateZExtOrTrunc(fileSize, sizeTy);
    if (mCodeUnitWidth > 8) {
        fileSize = iBuilder->CreateUDiv(fileSize, iBuilder->getSize(mCodeUnitWidth / 8));
    }
    Value * const isEmpty = iBuilder->CreateICmpEQ(fileSize, ConstantInt::getNullValue(fileSize->getType()));
    iBuilder->CreateUnlikelyCondBr(isEmpty, emptyFile, nonEmptyFile);
    // we cannot mmap a 0 length file; just create a 1-page sized fake file buffer for simplicity
    iBuilder->SetInsertPoint(emptyFile);
    Constant * pageSize = iBuilder->getSize(getpagesize());
    Value * fakeFileBuffer = iBuilder->CreateAnonymousMMap(pageSize);
    iBuilder->CreateBr(exit);

    iBuilder->SetInsertPoint(nonEmptyFile);
    Value * fileBackedBuffer = iBuilder->CreateFileSourceMMap(fd, fileSize);
    iBuilder->CreateBr(exit);

    iBuilder->SetInsertPoint(exit);
    PHINode * buffer = iBuilder->CreatePHI(fileBackedBuffer->getType(), 2);
    buffer->addIncoming(fakeFileBuffer, emptyFile);
    buffer->addIncoming(fileBackedBuffer, nonEmptyFile);
    PHINode * size = iBuilder->CreatePHI(sizeTy, 2);
    size->addIncoming(pageSize, emptyFile);
    size->addIncoming(fileSize, nonEmptyFile);

    iBuilder->setBaseAddress("sourceBuffer", buffer);
    iBuilder->setBufferedSize("sourceBuffer", size);
    iBuilder->setScalarField("readableBuffer", buffer);
    iBuilder->setScalarField("fileSize", fileSize);
    iBuilder->CreateMAdvise(buffer, fileSize, CBuilder::ADVICE_WILLNEED);

}

void MMapSourceKernel::generateDoSegmentMethod(const std::unique_ptr<KernelBuilder> & iBuilder) {

    BasicBlock * dropPages = iBuilder->CreateBasicBlock("dropPages");
    BasicBlock * processSegment = iBuilder->CreateBasicBlock("produceData");
    BasicBlock * setTermination = iBuilder->CreateBasicBlock("setTermination");
    BasicBlock * mmapSourceExit = iBuilder->CreateBasicBlock("mmapSourceExit");

    // instruct the OS that it can safely drop any fully consumed pages
    Value * consumed = iBuilder->getConsumedItemCount("sourceBuffer");
    IntegerType * const consumedTy = cast<IntegerType>(consumed->getType());
    Type * const voidPtrTy = iBuilder->getVoidPtrTy();

    DataLayout DL(iBuilder->getModule());
    IntegerType * const intAddrTy = iBuilder->getIntPtrTy(DL);

    // multiply the consumed count by the code unit size then mask off any partial pages
    if (mCodeUnitWidth > 8) {
        consumed = iBuilder->CreateMul(consumed, ConstantInt::get(consumedTy, mCodeUnitWidth / 8));
    }
    const auto pageSize = getpagesize();
    if (LLVM_LIKELY((pageSize & (pageSize - 1)) == 0)) {
        consumed = iBuilder->CreateAnd(consumed, ConstantExpr::getNot(ConstantInt::get(consumedTy, pageSize - 1)));
    } else {
        consumed = iBuilder->CreateSub(consumed, iBuilder->CreateURem(consumed, ConstantInt::get(consumedTy, pageSize)));
    }

    Value * sourceBuffer = iBuilder->getBaseAddress("sourceBuffer");
    sourceBuffer = iBuilder->CreatePtrToInt(sourceBuffer, intAddrTy);
    if (LLVM_UNLIKELY(intAddrTy->getBitWidth() > consumedTy->getBitWidth())) {
        consumed = iBuilder->CreateZExt(consumed, intAddrTy);
    } else if (LLVM_UNLIKELY(intAddrTy->getBitWidth() < consumedTy->getBitWidth())) {
        sourceBuffer = iBuilder->CreateZExt(sourceBuffer, consumedTy);
    }
    Value * consumedBuffer = iBuilder->CreateAdd(sourceBuffer, consumed);
    Value * readableBuffer = iBuilder->getScalarField("readableBuffer");
    readableBuffer = iBuilder->CreatePtrToInt(readableBuffer, consumedBuffer->getType());
    Value * unnecessaryBytes = iBuilder->CreateSub(consumedBuffer, readableBuffer);

    // avoid calling madvise unless an actual page table change could occur
    Value * hasPagesToDrop = iBuilder->CreateICmpEQ(unnecessaryBytes, ConstantInt::getNullValue(intAddrTy));
    iBuilder->CreateLikelyCondBr(hasPagesToDrop, processSegment, dropPages);

    iBuilder->SetInsertPoint(dropPages);
    iBuilder->CreateMAdvise(iBuilder->CreateIntToPtr(readableBuffer, voidPtrTy), unnecessaryBytes, CBuilder::ADVICE_DONTNEED);
    readableBuffer = iBuilder->CreateIntToPtr(iBuilder->CreateAdd(readableBuffer, unnecessaryBytes), voidPtrTy);
    iBuilder->setScalarField("readableBuffer", readableBuffer);
    iBuilder->CreateBr(processSegment);

    // determine whether or not we've exhausted the file buffer
    iBuilder->SetInsertPoint(processSegment);
    ConstantInt * segmentItems = iBuilder->getSize(mSegmentBlocks * iBuilder->getBitBlockWidth());
    Value * const fileSize = iBuilder->getScalarField("fileSize");
    Value * const produced = iBuilder->CreateAdd(iBuilder->getProducedItemCount("sourceBuffer"), segmentItems);
    Value * const lessThanFullSegment = iBuilder->CreateICmpULT(fileSize, produced);
    iBuilder->CreateUnlikelyCondBr(lessThanFullSegment, setTermination, mmapSourceExit);
    iBuilder->SetInsertPoint(setTermination);

    iBuilder->setTerminationSignal();
    iBuilder->CreateBr(mmapSourceExit);

    // finally, set the "produced" count to reflect current position in the file
    iBuilder->SetInsertPoint(mmapSourceExit);
    PHINode * itemsRead = iBuilder->CreatePHI(produced->getType(), 2);
    itemsRead->addIncoming(produced, processSegment);
    itemsRead->addIncoming(fileSize, setTermination);
    iBuilder->setProducedItemCount("sourceBuffer", itemsRead);
}

void MMapSourceKernel::generateFinalizeMethod(const std::unique_ptr<KernelBuilder> & iBuilder) {
    iBuilder->CreateMUnmap(iBuilder->getBaseAddress("sourceBuffer"), iBuilder->getBufferedSize("sourceBuffer"));
}

MMapSourceKernel::MMapSourceKernel(const std::unique_ptr<kernel::KernelBuilder> & iBuilder, unsigned blocksPerSegment, unsigned codeUnitWidth)
: SegmentOrientedKernel("mmap_source" + std::to_string(blocksPerSegment) + "@" + std::to_string(codeUnitWidth),
{},
{Binding{iBuilder->getStreamSetTy(1, codeUnitWidth), "sourceBuffer"}},
{Binding{iBuilder->getInt32Ty(), "fileDescriptor"}},
{Binding{iBuilder->getSizeTy(), "fileSize"}}, {Binding{iBuilder->getVoidPtrTy(), "readableBuffer"}})
, mSegmentBlocks(blocksPerSegment)
, mCodeUnitWidth(codeUnitWidth)
, mFileSizeFunction(nullptr) {

}

/// READ SOURCE KERNEL

void ReadSourceKernel::generateInitializeMethod(const std::unique_ptr<KernelBuilder> & iBuilder) {
    ConstantInt * const bufferSize = iBuilder->getSize(64 * getpagesize());
    Value * const buffer = iBuilder->CreateAlignedMalloc(bufferSize, iBuilder->getCacheAlignment());
    iBuilder->setScalarField("buffer", buffer);
    iBuilder->setScalarField("capacity", bufferSize);
    iBuilder->setBaseAddress("sourceBuffer", buffer);
    iBuilder->setBufferedSize("sourceBuffer", iBuilder->getSize(0));
}

void ReadSourceKernel::generateDoSegmentMethod(const std::unique_ptr<KernelBuilder> & iBuilder) {

    ConstantInt * const pageSize = iBuilder->getSize(getpagesize());
    PointerType * const codeUnitPtrTy = IntegerType::get(iBuilder->getContext(), mCodeUnitWidth)->getPointerTo();
    BasicBlock * const entryBlock = iBuilder->GetInsertBlock();
    BasicBlock * const exhaustedBuffer = iBuilder->CreateBasicBlock("ExhaustedBuffer");
    BasicBlock * const waitOnConsumers = iBuilder->CreateBasicBlock("WaitOnConsumers");
    BasicBlock * const readData = iBuilder->CreateBasicBlock("ReadData");
    BasicBlock * const stdInExit = iBuilder->CreateBasicBlock("StdInExit");

    assert(iBuilder->getKernel() == this);

    // The ReadSourceKernel begins by checking whether it needs to read another page of data
    ConstantInt * const segmentSize = iBuilder->getSize(mSegmentBlocks * iBuilder->getBitBlockWidth());
    Value * bufferedSize = iBuilder->getBufferedSize("sourceBuffer");
    Value * const produced = iBuilder->getProducedItemCount("sourceBuffer");
    Value * unreadSize = iBuilder->CreateSub(bufferedSize, produced);
    iBuilder->CreateUnlikelyCondBr(iBuilder->CreateICmpULT(unreadSize, segmentSize), exhaustedBuffer, stdInExit);

    // If so, it checks whether it can simply append another page to the existing buffer or whether
    // we need to perform a copyback.

    iBuilder->SetInsertPoint(exhaustedBuffer);

    // Otherwise, we're going to have to perform a copy back...

    // Let L be the logical buffer address (i.e., the position of the "first byte" of the input stream)
    // and B be the address pointing to the beginning of our actual buffer. Check whether:

    //     L + produced + pagesize < B + capacity

    // If so, we can append to our existing buffer without impacting any subsequent kernel.

    Value * inputStream = iBuilder->getRawOutputPointer("sourceBuffer", iBuilder->getInt32(0), iBuilder->getInt32(0));
    inputStream = iBuilder->CreatePointerCast(inputStream, codeUnitPtrTy);
    Value * const originalPtr = iBuilder->CreateGEP(inputStream, produced);
    Value * const buffer = iBuilder->CreatePointerCast(iBuilder->getScalarField("buffer"), codeUnitPtrTy);
    Value * const capacity = iBuilder->getScalarField("capacity");
    Value * const canAppend = iBuilder->CreateICmpULT(iBuilder->CreateGEP(originalPtr, pageSize), iBuilder->CreateGEP(buffer, capacity));
    iBuilder->CreateLikelyCondBr(canAppend, readData, waitOnConsumers);

    // First wait on any consumers to finish processing then check how much data has been consumed.
    iBuilder->SetInsertPoint(waitOnConsumers);
    iBuilder->CreateConsumerWait();
    // Then determine how much data has been consumed and how much needs to be copied back, noting
    // that our "unproduced" data must be block aligned.
    const auto alignment = iBuilder->getBitBlockWidth() / 8;
    Constant * const alignmentMask = ConstantExpr::getNeg(iBuilder->getSize(alignment));
    Value * const consumed = iBuilder->CreateAnd(iBuilder->getConsumedItemCount("sourceBuffer"), alignmentMask);
    Value * const remaining = iBuilder->CreateSub(bufferedSize, consumed);
    Value * const unconsumedPtr = iBuilder->CreateGEP(inputStream, consumed);
    Value * const consumedMajority = iBuilder->CreateICmpULT(iBuilder->CreateGEP(buffer, remaining), unconsumedPtr);
    BasicBlock * const copyBack = iBuilder->CreateBasicBlock("CopyBack");
    BasicBlock * const expandAndCopyBack = iBuilder->CreateBasicBlock("ExpandAndCopyBack");
    BasicBlock * const calculateLogicalAddress = iBuilder->CreateBasicBlock("CalculateLogicalAddress");
    // Have we consumed enough data that we can safely copy back the unconsumed data without needing
    // a temporary buffer? (i.e., B + remaining < L + consumed)
    iBuilder->CreateLikelyCondBr(consumedMajority, copyBack, expandAndCopyBack);
    iBuilder->SetInsertPoint(copyBack);
    // If so, just copy the data ...
    iBuilder->CreateMemCpy(buffer, unconsumedPtr, remaining, alignment);
    iBuilder->CreateBr(calculateLogicalAddress);
    // Otherwise, allocate a buffer with twice the capacity and copy the unconsumed data back into it
    iBuilder->SetInsertPoint(expandAndCopyBack);
    Value * const expandedCapacity = iBuilder->CreateShl(capacity, 1);
    Value * const expandedBuffer = iBuilder->CreateAlignedMalloc(expandedCapacity, iBuilder->getCacheAlignment());
    Value * const expandedPtr = iBuilder->CreatePointerCast(expandedBuffer, codeUnitPtrTy);
    iBuilder->CreateMemCpy(expandedPtr, unconsumedPtr, remaining, alignment);
    iBuilder->CreateAlignedFree(buffer);
    iBuilder->setScalarField("buffer", expandedBuffer);
    iBuilder->setScalarField("capacity", expandedCapacity);
    iBuilder->CreateBr(calculateLogicalAddress);
    // Update the logical address for this buffer....
    iBuilder->SetInsertPoint(calculateLogicalAddress);
    PHINode * const baseAddress = iBuilder->CreatePHI(codeUnitPtrTy, 2);
    baseAddress->addIncoming(buffer, copyBack);
    baseAddress->addIncoming(expandedPtr, expandAndCopyBack);
    Value * const modifiedPtr = iBuilder->CreateGEP(baseAddress, remaining);
    Value * const logicalAddress = iBuilder->CreateGEP(modifiedPtr, iBuilder->CreateNeg(produced));
    iBuilder->setBaseAddress("sourceBuffer", logicalAddress);
    iBuilder->CreateBr(readData);
    // Regardless of whether we're simply appending data or had to allocate a new buffer, read a new page
    // of data into the input source buffer. If we fail to read a full segment ...
    readData->moveAfter(calculateLogicalAddress);
    iBuilder->SetInsertPoint(readData);
    calculateLogicalAddress->moveAfter(calculateLogicalAddress);
    PHINode * const addr = iBuilder->CreatePHI(codeUnitPtrTy, 2);
    addr->addIncoming(originalPtr, exhaustedBuffer);
    addr->addIncoming(modifiedPtr, calculateLogicalAddress);
    assert(iBuilder->getKernel() == this);
    Value * const fd = iBuilder->getScalarField("fileDescriptor");
    Value * bytesRead = iBuilder->CreateReadCall(fd, addr, pageSize);
    unreadSize = iBuilder->CreateAdd(unreadSize, bytesRead);
    bufferedSize = iBuilder->CreateAdd(bufferedSize, bytesRead);
    iBuilder->setBufferedSize("sourceBuffer", bufferedSize);
    Value * const exhaustedInputSource = iBuilder->CreateICmpULT(unreadSize, segmentSize);
    BasicBlock * const setTermination = iBuilder->CreateBasicBlock("SetTermination");
    iBuilder->CreateUnlikelyCondBr(exhaustedInputSource, setTermination, stdInExit);

    // ... zero out the remaining bytes and set the termination signal.
    iBuilder->SetInsertPoint(setTermination);
    Value * const bytesToZero = iBuilder->CreateSub(segmentSize, unreadSize);
    iBuilder->CreateMemZero(iBuilder->CreateGEP(addr, unreadSize), bytesToZero);
    iBuilder->setTerminationSignal();
    iBuilder->CreateBr(stdInExit);

    // finally add the segment item count to the produced item count to inform the subsequent kernels how
    // much data is available for processing
    iBuilder->SetInsertPoint(stdInExit);
    stdInExit->moveAfter(setTermination);
    PHINode * const items = iBuilder->CreatePHI(produced->getType(), 3);
    items->addIncoming(segmentSize, entryBlock);
    items->addIncoming(segmentSize, readData);
    items->addIncoming(unreadSize, setTermination);
    iBuilder->setProducedItemCount("sourceBuffer", iBuilder->CreateAdd(produced, items));
}

void ReadSourceKernel::generateFinalizeMethod(const std::unique_ptr<KernelBuilder> & iBuilder) {
    iBuilder->CreateAlignedFree(iBuilder->getScalarField("buffer"));
}

ReadSourceKernel::ReadSourceKernel(const std::unique_ptr<kernel::KernelBuilder> & iBuilder, unsigned blocksPerSegment, unsigned codeUnitWidth)
: SegmentOrientedKernel("read_source"
, {}
, {Binding{iBuilder->getStreamSetTy(1, codeUnitWidth), "sourceBuffer"}}
, {Binding{iBuilder->getInt32Ty(), "fileDescriptor"}}
, {}
, {Binding{iBuilder->getVoidPtrTy(), "buffer"}, Binding{iBuilder->getSizeTy(), "capacity"}})
, mSegmentBlocks(blocksPerSegment)
, mCodeUnitWidth(codeUnitWidth) {

}

/// MEMORY SOURCE KERNEL

void MemorySourceKernel::generateInitializeMethod(const std::unique_ptr<KernelBuilder> & iBuilder) {
    iBuilder->setBaseAddress("sourceBuffer", iBuilder->CreatePointerCast(iBuilder->getScalarField("fileSource"), iBuilder->getVoidPtrTy()));
    iBuilder->setBufferedSize("sourceBuffer", iBuilder->getScalarField("fileSize"));
}

void MemorySourceKernel::generateDoSegmentMethod(const std::unique_ptr<KernelBuilder> & iBuilder) {

    BasicBlock * entryBlock = iBuilder->GetInsertBlock();
    BasicBlock * setTermination = iBuilder->CreateBasicBlock("setTermination");
    BasicBlock * mmapSourceExit = iBuilder->CreateBasicBlock("sourceExit");
    ConstantInt * segmentItems = iBuilder->getSize(mSegmentBlocks * iBuilder->getBitBlockWidth());
    Value * fileItems = iBuilder->getScalarField("fileSize");
    if (mCodeUnitWidth > 8) {
        fileItems = iBuilder->CreateUDiv(fileItems, iBuilder->getSize(mCodeUnitWidth / 8));
    }
    Value * produced = iBuilder->getProducedItemCount("sourceBuffer");
    produced = iBuilder->CreateAdd(produced, segmentItems);
    Value * lessThanFullSegment = iBuilder->CreateICmpULT(fileItems, produced);
    iBuilder->CreateCondBr(lessThanFullSegment, setTermination, mmapSourceExit);
    iBuilder->SetInsertPoint(setTermination);
    iBuilder->setTerminationSignal();
    iBuilder->CreateBr(mmapSourceExit);

    iBuilder->SetInsertPoint(mmapSourceExit);

    PHINode * itemsRead = iBuilder->CreatePHI(produced->getType(), 2);
    itemsRead->addIncoming(produced, entryBlock);
    itemsRead->addIncoming(fileItems, setTermination);
    iBuilder->setProducedItemCount("sourceBuffer", itemsRead);
}

MemorySourceKernel::MemorySourceKernel(const std::unique_ptr<kernel::KernelBuilder> & iBuilder, Type * type, unsigned blocksPerSegment, unsigned codeUnitWidth)
: SegmentOrientedKernel("memory_source",
    {},
    {Binding{iBuilder->getStreamSetTy(1, codeUnitWidth), "sourceBuffer"}},
    {Binding{cast<PointerType>(type), "fileSource"}, Binding{iBuilder->getSizeTy(), "fileSize"}}, {}, {})
, mSegmentBlocks(blocksPerSegment)
, mCodeUnitWidth(codeUnitWidth) {

}

}
