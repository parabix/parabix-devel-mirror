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

void MMapSourceKernel::linkExternalMethods(const std::unique_ptr<kernel::KernelBuilder> & kb) {
    mFileSizeFunction = kb->LinkFunction("file_size", &file_size);
}

void MMapSourceKernel::generateInitializeMethod(const std::unique_ptr<KernelBuilder> & kb) {
    BasicBlock * const emptyFile = kb->CreateBasicBlock("EmptyFile");
    BasicBlock * const nonEmptyFile = kb->CreateBasicBlock("NonEmptyFile");
    BasicBlock * const exit = kb->CreateBasicBlock("Exit");
    IntegerType * const sizeTy = kb->getSizeTy();
    assert (kb->getKernel() == this);
    Value * const fd = kb->getScalarField("fileDescriptor");
    assert (mFileSizeFunction);
    Value * fileSize = kb->CreateCall(mFileSizeFunction, fd);
    fileSize = kb->CreateZExtOrTrunc(fileSize, sizeTy);
    if (mCodeUnitWidth > 8) {
        fileSize = kb->CreateUDiv(fileSize, kb->getSize(mCodeUnitWidth / 8));
    }
    Value * const isEmpty = kb->CreateICmpEQ(fileSize, ConstantInt::getNullValue(fileSize->getType()));
    kb->CreateUnlikelyCondBr(isEmpty, emptyFile, nonEmptyFile);
    // we cannot mmap a 0 length file; just create a 1-page sized fake file buffer for simplicity
    kb->SetInsertPoint(emptyFile);
    Constant * pageSize = kb->getSize(getpagesize());
    Value * fakeFileBuffer = kb->CreateAnonymousMMap(pageSize);
    kb->CreateBr(exit);

    kb->SetInsertPoint(nonEmptyFile);
    Value * fileBackedBuffer = kb->CreateFileSourceMMap(fd, fileSize);
    kb->CreateBr(exit);

    kb->SetInsertPoint(exit);
    PHINode * buffer = kb->CreatePHI(fileBackedBuffer->getType(), 2);
    buffer->addIncoming(fakeFileBuffer, emptyFile);
    buffer->addIncoming(fileBackedBuffer, nonEmptyFile);
    PHINode * size = kb->CreatePHI(sizeTy, 2);
    size->addIncoming(pageSize, emptyFile);
    size->addIncoming(fileSize, nonEmptyFile);
    Value * bufferPtr = kb->CreatePointerCast(buffer, kb->getInt8PtrTy());
    kb->setBaseAddress("sourceBuffer", bufferPtr);
    kb->setBufferedSize("sourceBuffer", size);
    kb->setScalarField("readableBuffer", bufferPtr);
    kb->setScalarField("fileSize", fileSize);
    kb->setCapacity("sourceBuffer", fileSize);
    kb->CreateMAdvise(buffer, fileSize, CBuilder::ADVICE_WILLNEED);

}

void MMapSourceKernel::generateDoSegmentMethod(const std::unique_ptr<KernelBuilder> & kb) {

    BasicBlock * dropPages = kb->CreateBasicBlock("dropPages");
    BasicBlock * processSegment = kb->CreateBasicBlock("produceData");
    BasicBlock * setTermination = kb->CreateBasicBlock("setTermination");
    BasicBlock * mmapSourceExit = kb->CreateBasicBlock("mmapSourceExit");

    // instruct the OS that it can safely drop any fully consumed pages
    Value * consumed = kb->getConsumedItemCount("sourceBuffer");
    IntegerType * const consumedTy = cast<IntegerType>(consumed->getType());
    Type * const int8PtrTy = kb->getInt8PtrTy();

    DataLayout DL(kb->getModule());
    IntegerType * const intAddrTy = kb->getIntPtrTy(DL);

    // multiply the consumed count by the code unit size then mask off any partial pages
    if (mCodeUnitWidth > 8) {
        consumed = kb->CreateMul(consumed, ConstantInt::get(consumedTy, mCodeUnitWidth / 8));
    }
    const auto pageSize = getpagesize();
    if (LLVM_LIKELY((pageSize & (pageSize - 1)) == 0)) {
        consumed = kb->CreateAnd(consumed, ConstantExpr::getNeg(ConstantInt::get(consumedTy, pageSize)));
    } else {
        consumed = kb->CreateSub(consumed, kb->CreateURem(consumed, ConstantInt::get(consumedTy, pageSize)));
    }

    Value * sourceBuffer = kb->getBaseAddress("sourceBuffer");
    sourceBuffer = kb->CreatePtrToInt(sourceBuffer, intAddrTy);
    if (LLVM_UNLIKELY(intAddrTy->getBitWidth() > consumedTy->getBitWidth())) {
        consumed = kb->CreateZExt(consumed, intAddrTy);
    } else if (LLVM_UNLIKELY(intAddrTy->getBitWidth() < consumedTy->getBitWidth())) {
        sourceBuffer = kb->CreateZExt(sourceBuffer, consumedTy);
    }
    Value * consumedBuffer = kb->CreateAdd(sourceBuffer, consumed);
    Value * readableBuffer = kb->getScalarField("readableBuffer");
    readableBuffer = kb->CreatePtrToInt(readableBuffer, consumedBuffer->getType());
    Value * unnecessaryBytes = kb->CreateSub(consumedBuffer, readableBuffer);

    // avoid calling madvise unless an actual page table change could occur
    Value * hasPagesToDrop = kb->CreateICmpEQ(unnecessaryBytes, ConstantInt::getNullValue(intAddrTy));
    kb->CreateLikelyCondBr(hasPagesToDrop, processSegment, dropPages);

    kb->SetInsertPoint(dropPages);
    kb->CreateMAdvise(kb->CreateIntToPtr(readableBuffer, int8PtrTy), unnecessaryBytes, CBuilder::ADVICE_DONTNEED);
    readableBuffer = kb->CreateIntToPtr(kb->CreateAdd(readableBuffer, unnecessaryBytes), int8PtrTy);
    kb->setScalarField("readableBuffer", readableBuffer);
    kb->CreateBr(processSegment);

    // determine whether or not we've exhausted the file buffer
    kb->SetInsertPoint(processSegment);
    ConstantInt * segmentItems = kb->getSize(mSegmentBlocks * kb->getBitBlockWidth());
    Value * const fileSize = kb->getScalarField("fileSize");
    Value * const produced = kb->CreateAdd(kb->getProducedItemCount("sourceBuffer"), segmentItems);
    Value * const lessThanFullSegment = kb->CreateICmpULT(fileSize, produced);
    kb->CreateUnlikelyCondBr(lessThanFullSegment, setTermination, mmapSourceExit);
    kb->SetInsertPoint(setTermination);

    kb->setTerminationSignal();
    kb->CreateBr(mmapSourceExit);

    // finally, set the "produced" count to reflect current position in the file
    kb->SetInsertPoint(mmapSourceExit);
    PHINode * itemsRead = kb->CreatePHI(produced->getType(), 2);
    itemsRead->addIncoming(produced, processSegment);
    itemsRead->addIncoming(fileSize, setTermination);
    kb->setProducedItemCount("sourceBuffer", itemsRead);
}

void MMapSourceKernel::generateFinalizeMethod(const std::unique_ptr<KernelBuilder> & kb) {
    kb->CreateMUnmap(kb->getBaseAddress("sourceBuffer"), kb->getBufferedSize("sourceBuffer"));
}

MMapSourceKernel::MMapSourceKernel(const std::unique_ptr<kernel::KernelBuilder> & kb, unsigned blocksPerSegment, unsigned codeUnitWidth)
: SegmentOrientedKernel("mmap_source" + std::to_string(blocksPerSegment) + "@" + std::to_string(codeUnitWidth),
{},
{Binding{kb->getStreamSetTy(1, codeUnitWidth), "sourceBuffer"}},
{Binding{kb->getInt32Ty(), "fileDescriptor"}},
{Binding{kb->getSizeTy(), "fileSize"}}, {Binding{kb->getInt8PtrTy(), "readableBuffer"}})
, mSegmentBlocks(blocksPerSegment)
, mCodeUnitWidth(codeUnitWidth)
, mFileSizeFunction(nullptr) {

}

/// READ SOURCE KERNEL

void ReadSourceKernel::generateInitializeMethod(const std::unique_ptr<KernelBuilder> & kb) {
    const size_t initialBufferSize = 8 * getpagesize() * mCodeUnitWidth;
    ConstantInt * const bufferBytes = kb->getSize(initialBufferSize * mCodeUnitWidth/8);
    PointerType * const codeUnitPtrTy = IntegerType::get(kb->getContext(), mCodeUnitWidth)->getPointerTo();
    Value * const buffer = kb->CreatePointerCast(kb->CreateCacheAlignedMalloc(bufferBytes), codeUnitPtrTy);
    kb->setScalarField("buffer", buffer);
    kb->setScalarField("capacity", kb->getSize(initialBufferSize));
    kb->setBaseAddress("sourceBuffer", buffer);
    kb->setBufferedSize("sourceBuffer", kb->getSize(0));
    kb->setCapacity("sourceBuffer", kb->getSize(initialBufferSize));
}

void ReadSourceKernel::generateDoSegmentMethod(const std::unique_ptr<KernelBuilder> & kb) {

    ConstantInt * const readSize = kb->getSize(getpagesize() * 8/mCodeUnitWidth);
    PointerType * const codeUnitPtrTy = IntegerType::get(kb->getContext(), mCodeUnitWidth)->getPointerTo();
    PointerType * const i8PtrTy = IntegerType::get(kb->getContext(), 8)->getPointerTo();
    ConstantInt * const codeUnitBytes = kb->getSize(mCodeUnitWidth/8);
    BasicBlock * const entryBlock = kb->GetInsertBlock();
    BasicBlock * const exhaustedBuffer = kb->CreateBasicBlock("ExhaustedBuffer");
    BasicBlock * const waitOnConsumers = kb->CreateBasicBlock("WaitOnConsumers");
    BasicBlock * const readData = kb->CreateBasicBlock("ReadData");
    BasicBlock * const stdInExit = kb->CreateBasicBlock("StdInExit");

    assert(kb->getKernel() == this);

    // Check whether we need to read another page of data
    ConstantInt * const segmentSize = kb->getSize(mSegmentBlocks * kb->getBitBlockWidth());
    Value * bufferedSize = kb->getBufferedSize("sourceBuffer");
    Value * const produced = kb->getProducedItemCount("sourceBuffer");
    Value * unreadSize = kb->CreateSub(bufferedSize, produced);
    kb->CreateUnlikelyCondBr(kb->CreateICmpULT(unreadSize, segmentSize), exhaustedBuffer, stdInExit);

    // If so, it checks whether it can simply append another page to the existing buffer or whether
    // we need to perform a copyback.

    kb->SetInsertPoint(exhaustedBuffer);

    // Otherwise, we're going to have to perform a copy back...

    // Let L be the logical buffer address (i.e., the position of the "first code unit" of the input stream)
    // and B be the address pointing to the beginning of our actual buffer. Check whether:

    //     L + produced + readSize < B + capacity

    // If so, we can append to our existing buffer without impacting any subsequent kernel.

    Value * inputStream = kb->getRawOutputPointer("sourceBuffer", kb->getInt32(0), kb->getInt32(0));
    Value * const originalPtr = kb->CreateGEP(inputStream, produced);

    Value * const buffer = kb->getScalarField("buffer");
    Value * const capacity = kb->getScalarField("capacity");

    Value * L = kb->CreateGEP(originalPtr, readSize);
    Value * B = kb->CreateGEP(buffer, capacity);
    Value * const canAppend = kb->CreateICmpULT(L, B);
    kb->CreateLikelyCondBr(canAppend, readData, waitOnConsumers);

    // First wait on any consumers to finish processing then check how much data has been consumed.
    kb->SetInsertPoint(waitOnConsumers);
    kb->CreateConsumerWait();

    // Then determine how much data has been consumed and how much needs to be copied back, noting
    // that our "unproduced" data must be block aligned.
    const size_t blockAlignment = kb->getBitBlockWidth() / 8;
    Constant * const alignmentMask = kb->getSize(-(blockAlignment * 8 / mCodeUnitWidth));
    Value * const consumed = kb->CreateAnd(kb->getConsumedItemCount("sourceBuffer"), alignmentMask);
    Value * const remaining = kb->CreateSub(bufferedSize, consumed);
    Value * const unconsumedPtr = kb->CreateGEP(inputStream, consumed);
    Value * const consumedMajority = kb->CreateICmpULT(kb->CreateGEP(buffer, remaining), unconsumedPtr);
    Value * target = buffer;
    Value * source = unconsumedPtr;
    Value * toCopy = remaining;
    if (mCodeUnitWidth != 8) {
        source = kb->CreatePointerCast(unconsumedPtr, i8PtrTy);
        toCopy = kb->CreateMul(remaining, codeUnitBytes);
    }

    BasicBlock * const copyBack = kb->CreateBasicBlock("CopyBack");
    BasicBlock * const expandAndCopyBack = kb->CreateBasicBlock("ExpandAndCopyBack");
    BasicBlock * const calculateLogicalAddress = kb->CreateBasicBlock("CalculateLogicalAddress");

    // Have we consumed enough data that we can safely copy back the unconsumed data without needing
    // a temporary buffer? (i.e., B + remaining < L + consumed)
    kb->CreateLikelyCondBr(consumedMajority, copyBack, expandAndCopyBack);
    kb->SetInsertPoint(copyBack);
    // If so, just copy the data ...
    if (mCodeUnitWidth != 8) {
        target = kb->CreatePointerCast(buffer, i8PtrTy);
    }
    kb->CreateMemCpy(target, source, toCopy, 1);
    kb->CreateBr(calculateLogicalAddress);
    
    // Otherwise, allocate a buffer with twice the capacity and copy the unconsumed data back into it
    kb->SetInsertPoint(expandAndCopyBack);
    Value * const expandedCapacity = kb->CreateShl(capacity, 1);
    Value * const expandedBytes = mCodeUnitWidth == 8 ? expandedCapacity : kb->CreateMul(expandedCapacity, codeUnitBytes);
    Value * const expandedBuffer = kb->CreatePointerCast(kb->CreateCacheAlignedMalloc(expandedBytes), codeUnitPtrTy);
    target = mCodeUnitWidth == 8 ? expandedBuffer : kb->CreatePointerCast(expandedBuffer, i8PtrTy);
    kb->CreateMemCpy(target, source, toCopy, 1);
    kb->CreateFree(buffer);
    kb->setScalarField("buffer", expandedBuffer);
    kb->setScalarField("capacity", expandedCapacity);
    kb->setCapacity("sourceBuffer", expandedCapacity);
    kb->CreateBr(calculateLogicalAddress);

    // Update the logical address for this buffer....
    kb->SetInsertPoint(calculateLogicalAddress);
    PHINode * const baseAddress = kb->CreatePHI(codeUnitPtrTy, 2);
    baseAddress->addIncoming(buffer, copyBack);
    baseAddress->addIncoming(expandedBuffer, expandAndCopyBack);
    Value * const logicalAddress = kb->CreateGEP(baseAddress, kb->CreateNeg(consumed));
    Value * const modifiedPtr = kb->CreateGEP(baseAddress, remaining);
    kb->setBaseAddress("sourceBuffer", logicalAddress);
    kb->CreateBr(readData);

    // Regardless of whether we're simply appending data or had to allocate a new buffer, read a new page
    // of data into the input source buffer. If we fail to read a full segment ...
    readData->moveAfter(calculateLogicalAddress);
    kb->SetInsertPoint(readData);
    calculateLogicalAddress->moveAfter(calculateLogicalAddress);
    PHINode * const addr = kb->CreatePHI(codeUnitPtrTy, 2);
    addr->addIncoming(originalPtr, exhaustedBuffer);
    addr->addIncoming(modifiedPtr, calculateLogicalAddress);
    assert(kb->getKernel() == this);
    Value * const fd = kb->getScalarField("fileDescriptor");
    Value * toRead = readSize;
    if (mCodeUnitWidth != 8) {
        toRead = kb->CreateMul(toRead, codeUnitBytes);
    }
    Value * bytesRead = kb->CreateReadCall(fd, addr, toRead);
    Value * itemsRead = bytesRead;
    if (mCodeUnitWidth != 8) {
        itemsRead = kb->CreateUDiv(bytesRead, codeUnitBytes);
    }
    unreadSize = kb->CreateAdd(unreadSize, itemsRead);
    bufferedSize = kb->CreateAdd(bufferedSize, itemsRead);
    kb->setBufferedSize("sourceBuffer", bufferedSize);
    Value * const exhaustedInputSource = kb->CreateICmpULT(unreadSize, segmentSize);
    BasicBlock * const setTermination = kb->CreateBasicBlock("SetTermination");
    kb->CreateUnlikelyCondBr(exhaustedInputSource, setTermination, stdInExit);

    // ... zero out the remaining bytes and set the termination signal.
    kb->SetInsertPoint(setTermination);
    Value * bytesToZero = kb->CreateSub(segmentSize, unreadSize);
    Value * unreadPtr = kb->CreateGEP(addr, unreadSize);
    bytesToZero = mCodeUnitWidth == 8 ? bytesToZero : kb->CreateMul(bytesToZero, codeUnitBytes);
    if (mCodeUnitWidth != 8) {
        bytesToZero = kb->CreateMul(bytesToZero, codeUnitBytes);
        unreadPtr = kb->CreatePointerCast(unreadPtr, i8PtrTy);
    }
    kb->CreateMemZero(unreadPtr, bytesToZero);
    kb->setTerminationSignal();
    kb->CreateBr(stdInExit);

    // finally add the segment item count to the produced item count to inform the subsequent kernels how
    // much data is available for processing
    kb->SetInsertPoint(stdInExit);
    stdInExit->moveAfter(setTermination);
    PHINode * const items = kb->CreatePHI(produced->getType(), 3);
    items->addIncoming(segmentSize, entryBlock);
    items->addIncoming(segmentSize, readData);
    items->addIncoming(unreadSize, setTermination);
    kb->setProducedItemCount("sourceBuffer", kb->CreateAdd(produced, items));
}

void ReadSourceKernel::generateFinalizeMethod(const std::unique_ptr<KernelBuilder> & kb) {
    kb->CreateFree(kb->getScalarField("buffer"));
}

ReadSourceKernel::ReadSourceKernel(const std::unique_ptr<kernel::KernelBuilder> & kb, unsigned blocksPerSegment, unsigned codeUnitWidth)
: SegmentOrientedKernel("read_source"  + std::to_string(blocksPerSegment) + "@" + std::to_string(codeUnitWidth)
, {}
, {Binding{kb->getStreamSetTy(1, codeUnitWidth), "sourceBuffer"}}
, {Binding{kb->getInt32Ty(), "fileDescriptor"}}
, {}
, {Binding{IntegerType::get(kb->getContext(), codeUnitWidth)->getPointerTo(), "buffer"}, Binding{kb->getSizeTy(), "capacity"}})
, mSegmentBlocks(blocksPerSegment)
, mCodeUnitWidth(codeUnitWidth) {

}

// Hybrid MMap/Read source kernel
    
FDSourceKernel::FDSourceKernel(const std::unique_ptr<kernel::KernelBuilder> & kb, unsigned blocksPerSegment, unsigned codeUnitWidth)
: SegmentOrientedKernel("FD_source" + std::to_string(blocksPerSegment) + "@" + std::to_string(codeUnitWidth)
, {}
, {Binding{kb->getStreamSetTy(1, codeUnitWidth), "sourceBuffer"}}
, {Binding{kb->getInt32Ty(), "fileDescriptor"}}
, {}
, {Binding{IntegerType::get(kb->getContext(), codeUnitWidth)->getPointerTo(), "buffer"}, Binding{kb->getSizeTy(), "capacity"},
    Binding{kb->getSizeTy(), "fileSize"}, Binding{kb->getInt8PtrTy(), "readableBuffer"}})
, mSegmentBlocks(blocksPerSegment)
, mCodeUnitWidth(codeUnitWidth)
, mFileSizeFunction(nullptr) {
    
}

void FDSourceKernel::generateFinalizeMethod(const std::unique_ptr<KernelBuilder> & kb) {
    BasicBlock * finalizeRead = kb->CreateBasicBlock("finalizeRead");
    BasicBlock * finalizeMMap = kb->CreateBasicBlock("finalizeMMap");
    BasicBlock * finalizeDone = kb->CreateBasicBlock("finalizeDone");
    // if the fileDescriptor is 0, the file is stdin, use readSource kernel logic, otherwise use mmap logic.
    kb->CreateCondBr(kb->CreateICmpEQ(kb->getScalarField("fileDescriptor"), kb->getInt32(STDIN_FILENO)), finalizeRead, finalizeMMap);
    kb->SetInsertPoint(finalizeRead);
    reinterpret_cast<ReadSourceKernel *>(this)->ReadSourceKernel::generateFinalizeMethod(kb);
    kb->CreateBr(finalizeDone);
    kb->SetInsertPoint(finalizeMMap);
    reinterpret_cast<MMapSourceKernel *>(this)->MMapSourceKernel::generateFinalizeMethod(kb);
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
    reinterpret_cast<ReadSourceKernel *>(this)->ReadSourceKernel::generateInitializeMethod(kb);
    kb->CreateBr(initializeDone);
    kb->SetInsertPoint(initializeMMap);
    reinterpret_cast<MMapSourceKernel *>(this)->MMapSourceKernel::generateInitializeMethod(kb);
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
    reinterpret_cast<ReadSourceKernel *>(this)->ReadSourceKernel::generateDoSegmentMethod(kb);
    kb->CreateBr(DoSegmentDone);
    kb->SetInsertPoint(DoSegmentMMap);
    reinterpret_cast<MMapSourceKernel *>(this)->MMapSourceKernel::generateDoSegmentMethod(kb);
    kb->CreateBr(DoSegmentDone);
    kb->SetInsertPoint(DoSegmentDone);
}


void FDSourceKernel::linkExternalMethods(const std::unique_ptr<kernel::KernelBuilder> & kb) {
    mFileSizeFunction = kb->LinkFunction("file_size", &file_size);
}
    
    
/// MEMORY SOURCE KERNEL

void MemorySourceKernel::generateInitializeMethod(const std::unique_ptr<KernelBuilder> & kb) {
    kb->setBaseAddress("sourceBuffer", kb->CreatePointerCast(kb->getScalarField("fileSource"), kb->getVoidPtrTy()));
    kb->setBufferedSize("sourceBuffer", kb->getScalarField("fileSize"));
    kb->setCapacity("sourceBuffer", kb->getScalarField("fileSize"));
}

void MemorySourceKernel::generateDoSegmentMethod(const std::unique_ptr<KernelBuilder> & kb) {

    BasicBlock * entryBlock = kb->GetInsertBlock();
    BasicBlock * setTermination = kb->CreateBasicBlock("setTermination");
    BasicBlock * mmapSourceExit = kb->CreateBasicBlock("sourceExit");
    ConstantInt * segmentItems = kb->getSize(mSegmentBlocks * kb->getBitBlockWidth());
    Value * fileItems = kb->getScalarField("fileSize");
    if (mCodeUnitWidth > 8) {
        fileItems = kb->CreateUDiv(fileItems, kb->getSize(mCodeUnitWidth / 8));
    }
    Value * produced = kb->getProducedItemCount("sourceBuffer");
    produced = kb->CreateAdd(produced, segmentItems);
    Value * lessThanFullSegment = kb->CreateICmpULT(fileItems, produced);
    kb->CreateCondBr(lessThanFullSegment, setTermination, mmapSourceExit);
    kb->SetInsertPoint(setTermination);
    kb->setTerminationSignal();
    kb->CreateBr(mmapSourceExit);

    kb->SetInsertPoint(mmapSourceExit);

    PHINode * itemsRead = kb->CreatePHI(produced->getType(), 2);
    itemsRead->addIncoming(produced, entryBlock);
    itemsRead->addIncoming(fileItems, setTermination);
    kb->setProducedItemCount("sourceBuffer", itemsRead);
}

MemorySourceKernel::MemorySourceKernel(const std::unique_ptr<kernel::KernelBuilder> & kb, Type * type, unsigned blocksPerSegment, unsigned codeUnitWidth)
: SegmentOrientedKernel("memory_source",
    {},
    {Binding{kb->getStreamSetTy(1, codeUnitWidth), "sourceBuffer"}},
    {Binding{cast<PointerType>(type), "fileSource"}, Binding{kb->getSizeTy(), "fileSize"}}, {}, {})
, mSegmentBlocks(blocksPerSegment)
, mCodeUnitWidth(codeUnitWidth) {

}

}
