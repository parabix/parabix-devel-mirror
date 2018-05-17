/*
 *  Copyright (c) 2018 International Characters.
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

extern "C" uint64_t file_size(const uint32_t fd) {
    struct stat st;
    if (LLVM_UNLIKELY(fstat(fd, &st) != 0)) {
        st.st_size = 0;
    }
    return st.st_size;
}

namespace kernel {

/// MMAP SOURCE KERNEL

Function * MMapSourceKernel::linkFileSizeMethod(const std::unique_ptr<kernel::KernelBuilder> & b) {
    return b->LinkFunction("file_size", &file_size);
}

void MMapSourceKernel::generateInitializeMethod(Function * const fileSizeMethod, const unsigned codeUnitWidth, const std::unique_ptr<KernelBuilder> & b) {

    BasicBlock * const emptyFile = b->CreateBasicBlock("emptyFile");
    BasicBlock * const nonEmptyFile = b->CreateBasicBlock("NonEmptyFile");
    BasicBlock * const exit = b->CreateBasicBlock("Exit");
    IntegerType * const sizeTy = b->getSizeTy();
    ConstantInt * const PAGE_SIZE = b->getSize(getpagesize());
    Value * const fd = b->getScalarField("fileDescriptor");
    assert (fileSizeMethod);
    Value * fileSize = b->CreateZExtOrTrunc(b->CreateCall(fileSizeMethod, fd), sizeTy);
    b->CreateLikelyCondBr(b->CreateIsNotNull(fileSize), nonEmptyFile, emptyFile);

    b->SetInsertPoint(nonEmptyFile);
    PointerType * const codeUnitPtrTy = b->getIntNTy(codeUnitWidth)->getPointerTo();
    Value * const fileBuffer = b->CreatePointerCast(b->CreateFileSourceMMap(fd, fileSize), codeUnitPtrTy);
    b->setScalarField("buffer", fileBuffer);
    b->setBaseAddress("sourceBuffer", fileBuffer);
    b->CreateMAdvise(fileBuffer, fileSize, CBuilder::ADVICE_WILLNEED);
    if (LLVM_UNLIKELY(codeUnitWidth > 8)) {
        fileSize = b->CreateUDiv(fileSize, b->getSize(codeUnitWidth / 8));
    }
    b->setScalarField("fileSize", fileSize);
    b->CreateBr(exit);

    b->SetInsertPoint(emptyFile);
    Value * const emptyFilePtr = b->CreatePointerCast(b->CreateAnonymousMMap(PAGE_SIZE), codeUnitPtrTy);
    b->setScalarField("buffer", emptyFilePtr);
    b->setBaseAddress("sourceBuffer", emptyFilePtr);
    b->setScalarField("fileSize", PAGE_SIZE);
    b->setTerminationSignal();
    b->CreateBr(exit);

    b->SetInsertPoint(exit);
}


void MMapSourceKernel::generateDoSegmentMethod(const unsigned codeUnitWidth, const std::unique_ptr<KernelBuilder> & b) {

    BasicBlock * const dropPages = b->CreateBasicBlock("dropPages");
    BasicBlock * const checkRemaining = b->CreateBasicBlock("checkRemaining");
    BasicBlock * const setTermination = b->CreateBasicBlock("setTermination");
    BasicBlock * const exit = b->CreateBasicBlock("mmapSourceExit");

    Constant * const PAGE_SIZE = b->getSize(getpagesize());
    Constant * const BLOCK_WIDTH = b->getSize(b->getBitBlockWidth());
    Constant * const CODE_UNIT_BYTES = b->getSize(codeUnitWidth / 8);

    Value * const fileItems = b->getScalarField("fileSize");
    Value * const consumedItems = b->getConsumedItemCount("sourceBuffer");
    Value * const consumedBytes = b->CreateMul(consumedItems, CODE_UNIT_BYTES);
    Value * const consumedPageOffset = b->CreateAnd(consumedBytes, ConstantExpr::getNeg(PAGE_SIZE));
    Value * const consumedBuffer = b->getRawOutputPointer("sourceBuffer", consumedPageOffset);
    Value * const readableBuffer = b->getScalarField("buffer");
    Value * const unnecessaryBytes = b->CreatePtrDiff(consumedBuffer, readableBuffer);

    // avoid calling madvise unless an actual page table change could occur
    b->CreateLikelyCondBr(b->CreateIsNotNull(unnecessaryBytes), dropPages, checkRemaining);

    b->SetInsertPoint(dropPages);
    // instruct the OS that it can safely drop any fully consumed pages
    b->CreateMAdvise(readableBuffer, unnecessaryBytes, CBuilder::ADVICE_DONTNEED);
    b->setScalarField("buffer", b->CreateGEP(readableBuffer, unnecessaryBytes));
    b->CreateBr(checkRemaining);

    // determine whether or not we've exhausted the file buffer
    b->SetInsertPoint(checkRemaining);
    Value * const producedItems = b->getProducedItemCount("sourceBuffer");
    Value * const nextProducedItems = b->CreateAdd(producedItems, PAGE_SIZE);
    Value * const lastPage = b->CreateICmpULE(fileItems, nextProducedItems);
    b->CreateUnlikelyCondBr(lastPage, setTermination, exit);

    // If this is the last page, create a temporary buffer of up to two pages size, copy the unconsumed data
    // and zero any bytes that are not used.
    b->SetInsertPoint(setTermination);
    Value * const consumedOffset = b->CreateAnd(consumedItems, ConstantExpr::getNeg(BLOCK_WIDTH));
    Value * const readStart = b->getRawOutputPointer("sourceBuffer", consumedOffset);
    Value * const readEnd = b->getRawOutputPointer("sourceBuffer", fileItems);
    Value * const unconsumedBytes = b->CreateTrunc(b->CreatePtrDiff(readEnd, readStart), b->getSizeTy());
    Value * const bufferSize = b->CreateRoundUp(b->CreateAdd(unconsumedBytes, BLOCK_WIDTH), PAGE_SIZE);
    Value * const buffer = b->CreateAlignedMalloc(bufferSize, b->getCacheAlignment());
    b->CreateMemCpy(buffer, readStart, unconsumedBytes, 1);
    b->CreateMemZero(b->CreateGEP(buffer, unconsumedBytes), b->CreateSub(bufferSize, unconsumedBytes), 1);
    // get the difference between our base and from position then compute an offsetted temporary buffer address
    Value * const base = b->getBaseAddress("sourceBuffer");
    Value * const diff = b->CreatePtrDiff(b->CreatePointerCast(base, readStart->getType()), readStart);
    Value * const offsettedBuffer = b->CreateGEP(buffer, diff);
    b->CreateConsumerWait();
    // Unmap the file buffer and set the temporary buffer as the new source buffer
    Value * const fileSize = b->CreateMul(fileItems, CODE_UNIT_BYTES);
    b->CreateMUnmap(base, fileSize);
    PointerType * const codeUnitPtrTy = b->getIntNTy(codeUnitWidth)->getPointerTo();
    b->setScalarField("buffer", b->CreatePointerCast(buffer, codeUnitPtrTy));
    b->setBaseAddress("sourceBuffer", b->CreatePointerCast(offsettedBuffer, codeUnitPtrTy));
    b->setTerminationSignal();
    BasicBlock * const terminationExit = b->GetInsertBlock();
    b->CreateBr(exit);

    // finally, set the "produced" count to reflect current position in the file
    b->SetInsertPoint(exit);
    PHINode * const newProducedItems = b->CreatePHI(b->getSizeTy(), 2);
    newProducedItems->addIncoming(nextProducedItems, checkRemaining);
    newProducedItems->addIncoming(fileItems, terminationExit);
    b->setProducedItemCount("sourceBuffer", newProducedItems);
}

void MMapSourceKernel::freeBuffer(const std::unique_ptr<KernelBuilder> & b) {
    b->CreateFree(b->getScalarField("buffer"));
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
    b->setScalarField("fileSize", b->getSize(0));
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
    Value * const buffered = b->getScalarField("fileSize");
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
    b->setScalarField("fileSize", itemsBuffered);
    b->CreateUnlikelyCondBr(b->CreateICmpULT(itemsBuffered, itemsPending), setTermination, readExit);

    // ... set the termination signal.
    b->SetInsertPoint(setTermination);
    Value * const bytesToZero = b->CreateMul(b->CreateSub(itemsPending, itemsBuffered), codeUnitBytes);
    b->CreateMemZero(b->getRawOutputPointer("sourceBuffer", itemsBuffered), bytesToZero);
    b->setTerminationSignal();
    b->CreateBr(readExit);

    b->SetInsertPoint(readExit);
    PHINode * const itemsProduced = b->CreatePHI(itemsPending->getType(), 3);
    itemsProduced->addIncoming(itemsPending, entry);
    itemsProduced->addIncoming(itemsPending, readData);
    itemsProduced->addIncoming(itemsBuffered, setTermination);
    b->setProducedItemCount("sourceBuffer", itemsProduced);
}

void ReadSourceKernel::freeBuffer(const std::unique_ptr<KernelBuilder> & b) {
    b->CreateFree(b->getScalarField("buffer"));
}

/// Hybrid MMap/Read source kernel

void FDSourceKernel::linkExternalMethods(const std::unique_ptr<kernel::KernelBuilder> & b) {
    mFileSizeFunction = MMapSourceKernel::linkFileSizeMethod(b);
}

void FDSourceKernel::generateFinalizeMethod(const std::unique_ptr<KernelBuilder> & b) {
    BasicBlock * finalizeRead = b->CreateBasicBlock("finalizeRead");
    BasicBlock * finalizeMMap = b->CreateBasicBlock("finalizeMMap");
    BasicBlock * finalizeDone = b->CreateBasicBlock("finalizeDone");
    b->CreateCondBr(b->CreateTrunc(b->getScalarField("useMMap"), b->getInt1Ty()), finalizeMMap, finalizeRead);
    b->SetInsertPoint(finalizeMMap);
    MMapSourceKernel::freeBuffer(b);
    b->CreateBr(finalizeDone);
    b->SetInsertPoint(finalizeRead);
    ReadSourceKernel::freeBuffer(b);
    b->CreateBr(finalizeDone);
    b->SetInsertPoint(finalizeDone);
}

void FDSourceKernel::generateInitializeMethod(const std::unique_ptr<KernelBuilder> & b) {
    BasicBlock * initializeRead = b->CreateBasicBlock("initializeRead");
    BasicBlock * tryMMap = b->CreateBasicBlock("tryMMap");
    BasicBlock * initializeMMap = b->CreateBasicBlock("initializeMMap");
    BasicBlock * initializeDone = b->CreateBasicBlock("initializeDone");

    // The source will use MMapSource or readSoure kernel logic depending on the useMMap
    // parameter, possibly overridden.
    Value * useMMap = b->CreateTrunc(b->getScalarField("useMMap"), b->getInt1Ty());
    // if the fileDescriptor is 0, the file is stdin, use readSource kernel logic.
    Value * fd = b->getScalarField("fileDescriptor");
    useMMap = b->CreateAnd(useMMap, b->CreateICmpNE(fd, b->getInt32(STDIN_FILENO)));
    b->CreateCondBr(useMMap, tryMMap, initializeRead);

    b->SetInsertPoint(tryMMap);
    // If the fileSize is 0, we may have a virtual file such as /proc/cpuinfo
    Value * fileSize = b->CreateZExtOrTrunc(b->CreateCall(mFileSizeFunction, fd), b->getSizeTy());
    useMMap = b->CreateICmpNE(fileSize, b->getSize(0));
    b->CreateCondBr(useMMap, initializeMMap, initializeRead);
    b->SetInsertPoint(initializeMMap);
    MMapSourceKernel::generateInitializeMethod(mFileSizeFunction, mCodeUnitWidth, b);
    b->CreateBr(initializeDone);

    b->SetInsertPoint(initializeRead);
    // Ensure that readSource logic is used throughout.
    b->setScalarField("useMMap", b->getInt8(0));
    ReadSourceKernel::generateInitializeMethod(mCodeUnitWidth, getStride(), b);
    b->CreateBr(initializeDone);
    b->SetInsertPoint(initializeDone);
}

void FDSourceKernel::generateDoSegmentMethod(const std::unique_ptr<KernelBuilder> & b) {
    BasicBlock * DoSegmentRead = b->CreateBasicBlock("DoSegmentRead");
    BasicBlock * DoSegmentMMap = b->CreateBasicBlock("DoSegmentMMap");
    BasicBlock * DoSegmentDone = b->CreateBasicBlock("DoSegmentDone");
    b->CreateCondBr(b->CreateTrunc(b->getScalarField("useMMap"), b->getInt1Ty()), DoSegmentMMap, DoSegmentRead);
    b->SetInsertPoint(DoSegmentMMap);
    MMapSourceKernel::generateDoSegmentMethod(mCodeUnitWidth, b);
    b->CreateBr(DoSegmentDone);
    b->SetInsertPoint(DoSegmentRead);
    ReadSourceKernel::generateDoSegmentMethod(mCodeUnitWidth, getStride(), b);
    b->CreateBr(DoSegmentDone);
    b->SetInsertPoint(DoSegmentDone);
}

/// MEMORY SOURCE KERNEL

void MemorySourceKernel::generateInitializeMethod(const std::unique_ptr<KernelBuilder> & b) {
    Value * const fileSource = b->getScalarField("fileSource");
    b->setBaseAddress("sourceBuffer", fileSource);
    Value * const fileSize = b->getScalarField("fileSize");
    b->setCapacity("sourceBuffer", fileSize);
    if (mStreamSetCount > 1) {
        b->setProducedItemCount("sourceBuffer", fileSize);
        b->setTerminationSignal();
    }
}

void MemorySourceKernel::generateDoSegmentMethod(const std::unique_ptr<KernelBuilder> & b) {
    if (mStreamSetCount == 1) {
        Constant * const PAGE_SIZE = b->getSize(getStride());
        Constant * const BLOCK_WIDTH = b->getSize(b->getBitBlockWidth());

        BasicBlock * const entry = b->GetInsertBlock();
        BasicBlock * const createTemporary = b->CreateBasicBlock("createTemporary");
        BasicBlock * const exit = b->CreateBasicBlock("exit");

        Value * const fileItems = b->getScalarField("fileSize");
        Value * const producedItems = b->getProducedItemCount("sourceBuffer");
        Value * const nextProducedItems = b->CreateAdd(producedItems, PAGE_SIZE);
        Value * const lastPage = b->CreateICmpULE(fileItems, nextProducedItems);
        b->CreateUnlikelyCondBr(lastPage, createTemporary, exit);

        b->SetInsertPoint(createTemporary);
        Value * const consumedItems = b->getConsumedItemCount("sourceBuffer");
        Value * const consumedOffset = b->CreateAnd(consumedItems, ConstantExpr::getNeg(BLOCK_WIDTH));
        Value * const readStart = b->getRawOutputPointer("sourceBuffer", consumedOffset);
        Value * const readEnd = b->getRawOutputPointer("sourceBuffer", fileItems);
        Value * const unconsumedBytes = b->CreateTrunc(b->CreatePtrDiff(readEnd, readStart), b->getSizeTy());
        Value * const bufferSize = b->CreateRoundUp(b->CreateAdd(unconsumedBytes, BLOCK_WIDTH), PAGE_SIZE);
        Value * const buffer = b->CreateAlignedMalloc(bufferSize, b->getCacheAlignment());
        b->CreateMemCpy(buffer, readStart, unconsumedBytes, 1);
        b->CreateMemZero(b->CreateGEP(buffer, unconsumedBytes), b->CreateSub(bufferSize, unconsumedBytes), 1);

        // get the difference between our base and from position then compute an offsetted temporary buffer address
        Value * const base = b->getBaseAddress("sourceBuffer");
        Value * const diff = b->CreatePtrDiff(b->CreatePointerCast(base, readStart->getType()), readStart);
        Value * const offsettedBuffer = b->CreateGEP(buffer, diff);
        b->CreateConsumerWait();

        // set the temporary buffer as the new source buffer
        PointerType * const codeUnitPtrTy = b->getIntNTy(mCodeUnitWidth)->getPointerTo();
        b->setScalarField("buffer", b->CreatePointerCast(buffer, codeUnitPtrTy));
        b->setBaseAddress("sourceBuffer", b->CreatePointerCast(offsettedBuffer, codeUnitPtrTy));
        b->setTerminationSignal();
        BasicBlock * const terminationExit = b->GetInsertBlock();
        b->CreateBr(exit);

        b->SetInsertPoint(exit);
        PHINode * const newProducedItems = b->CreatePHI(b->getSizeTy(), 2);
        newProducedItems->addIncoming(nextProducedItems, entry);
        newProducedItems->addIncoming(fileItems, terminationExit);
        b->setProducedItemCount("sourceBuffer", newProducedItems);
    }
}

void MemorySourceKernel::generateFinalizeMethod(const std::unique_ptr<KernelBuilder> & b) {
    if (mStreamSetCount == 1) {
        b->CreateFree(b->getScalarField("buffer"));
    }
}

MMapSourceKernel::MMapSourceKernel(const std::unique_ptr<kernel::KernelBuilder> & b, const unsigned codeUnitWidth)
: SegmentOrientedKernel("mmap_source@" + std::to_string(codeUnitWidth)
// input streams
, {}
// output streams
, {Binding{b->getStreamSetTy(1, codeUnitWidth), "sourceBuffer"}}
// input scalars
, {Binding{b->getInt32Ty(), "fileDescriptor"}}
// output scalars
, {}
// internal scalars
, {Binding{b->getIntNTy(codeUnitWidth)->getPointerTo(), "buffer"}
,  Binding{b->getSizeTy(), "fileSize"}})
, mCodeUnitWidth(codeUnitWidth)
, mFileSizeFunction(nullptr) {
    addAttribute(MustExplicitlyTerminate());
    setStride(getpagesize());
}

ReadSourceKernel::ReadSourceKernel(const std::unique_ptr<kernel::KernelBuilder> & b, const unsigned codeUnitWidth)
: SegmentOrientedKernel("read_source" + std::to_string(codegen::SegmentSize) + "@" + std::to_string(codeUnitWidth)
// input streams
, {}
// output streams
, {Binding{b->getStreamSetTy(1, codeUnitWidth), "sourceBuffer"}}
// input scalars
, {Binding{b->getInt32Ty(), "fileDescriptor"}}
// output scalars
, {}
// internal scalars
, {Binding{b->getIntNTy(codeUnitWidth)->getPointerTo(), "buffer"}
,  Binding{b->getSizeTy(), "fileSize"}})
, mCodeUnitWidth(codeUnitWidth) {
    addAttribute(MustExplicitlyTerminate());
    setStride(getpagesize());
}


FDSourceKernel::FDSourceKernel(const std::unique_ptr<kernel::KernelBuilder> & b, const unsigned codeUnitWidth)
: SegmentOrientedKernel("FD_source@" + std::to_string(codeUnitWidth)
// input streams
, {}
// output stream
, {Binding{b->getStreamSetTy(1, codeUnitWidth), "sourceBuffer"}}
// input scalar
, {Binding{b->getInt8Ty(), "useMMap"}, Binding{b->getInt32Ty(), "fileDescriptor"}}
, {}
// internal scalars
, {Binding{b->getIntNTy(codeUnitWidth)->getPointerTo(), "buffer"},
   Binding{b->getSizeTy(), "fileSize"}})
, mCodeUnitWidth(codeUnitWidth)
, mFileSizeFunction(nullptr) {
    addAttribute(MustExplicitlyTerminate());
    setStride(getpagesize());
}

MemorySourceKernel::MemorySourceKernel(const std::unique_ptr<kernel::KernelBuilder> & b, const unsigned streamSetCount, const unsigned codeUnitWidth)
: SegmentOrientedKernel("memory_source@" + std::to_string(streamSetCount) + ":" + std::to_string(codeUnitWidth),
// input streams
{},
// output stream
{Binding{b->getStreamSetTy(streamSetCount, codeUnitWidth), "sourceBuffer"}},
// input scalar
{Binding{b->getIntNTy(codeUnitWidth)->getPointerTo(), "fileSource"}, Binding{b->getSizeTy(), "fileSize"}},
{},
// internal scalar
{Binding{b->getIntNTy(codeUnitWidth)->getPointerTo(), "buffer"}})
, mStreamSetCount(streamSetCount)
, mCodeUnitWidth(codeUnitWidth) {
    addAttribute(MustExplicitlyTerminate());
    setStride(getpagesize());
}

}
