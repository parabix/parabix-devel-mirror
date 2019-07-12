/*
 *  Copyright (c) 2018 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 */
#include <kernel/util/source_kernel.h>
#include <kernel/core/kernel_builder.h>
#include <kernel/core/streamset.h>
#include <llvm/IR/Module.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <toolchain/toolchain.h>
#include <boost/interprocess/mapped_region.hpp>

using namespace llvm;

inline unsigned getPageSize() {
    return boost::interprocess::mapped_region::get_page_size();
}

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
    return b->LinkFunction("file_size", file_size);
}

void MMapSourceKernel::generateInitializeMethod(Function * const fileSizeMethod, const unsigned codeUnitWidth, const unsigned stride, const std::unique_ptr<KernelBuilder> & b) {

    BasicBlock * const emptyFile = b->CreateBasicBlock("emptyFile");
    BasicBlock * const nonEmptyFile = b->CreateBasicBlock("NonEmptyFile");
    BasicBlock * const exit = b->CreateBasicBlock("Exit");
    IntegerType * const sizeTy = b->getSizeTy();
    ConstantInt * const STRIDE_SIZE = b->getSize(stride);
    Constant * const PAGE_ITEMS = b->getSize((8 * stride) / codeUnitWidth);
    Value * const fd = b->getScalarField("fileDescriptor");
    PointerType * const codeUnitPtrTy = b->getIntNTy(codeUnitWidth)->getPointerTo();
    b->setScalarField("ancillaryBuffer", ConstantPointerNull::get(codeUnitPtrTy));
    assert (fileSizeMethod);
    Value * fileSize = b->CreateZExtOrTrunc(b->CreateCall(fileSizeMethod, fd), sizeTy);
    b->CreateLikelyCondBr(b->CreateIsNotNull(fileSize), nonEmptyFile, emptyFile);

    b->SetInsertPoint(nonEmptyFile);
    Value * const fileBuffer = b->CreatePointerCast(b->CreateFileSourceMMap(fd, fileSize), codeUnitPtrTy);
    b->setScalarField("buffer", fileBuffer);
    b->setBaseAddress("sourceBuffer", fileBuffer);
    b->CreateMAdvise(fileBuffer, fileSize, CBuilder::ADVICE_WILLNEED);
    Value * fileItems = fileSize;
    if (LLVM_UNLIKELY(codeUnitWidth > 8)) {
        fileItems = b->CreateUDiv(fileSize, b->getSize(codeUnitWidth / 8));
    }
    b->setScalarField("fileItems", fileItems);
    b->setCapacity("sourceBuffer", fileItems);
    b->CreateBr(exit);

    b->SetInsertPoint(emptyFile);
    Value * const emptyFilePtr = b->CreatePointerCast(b->CreateAnonymousMMap(STRIDE_SIZE), codeUnitPtrTy);
    b->setScalarField("buffer", emptyFilePtr);
    b->setBaseAddress("sourceBuffer", emptyFilePtr);
    b->setScalarField("fileItems", PAGE_ITEMS);
    b->setTerminationSignal();
    b->CreateBr(exit);

    b->SetInsertPoint(exit);
}


void MMapSourceKernel::generateDoSegmentMethod(const unsigned codeUnitWidth, const unsigned stride, const std::unique_ptr<KernelBuilder> & b) {

    BasicBlock * const dropPages = b->CreateBasicBlock("dropPages");
    BasicBlock * const checkRemaining = b->CreateBasicBlock("checkRemaining");
    BasicBlock * const setTermination = b->CreateBasicBlock("setTermination");
    BasicBlock * const exit = b->CreateBasicBlock("mmapSourceExit");

    ConstantInt * const MMAP_PAGE_SIZE = b->getSize(getPageSize());
    ConstantInt * const STRIDE_ITEMS = b->getSize(stride);
    ConstantInt * const BLOCK_WIDTH = b->getSize(b->getBitBlockWidth());
    ConstantInt * const CODE_UNIT_BYTES = b->getSize(codeUnitWidth / 8);

    ConstantInt * const STRIDE_BYTES = b->getSize((codeUnitWidth * stride) / 8);
    ConstantInt * const PADDING_SIZE = b->getSize(b->getBitBlockWidth() * codeUnitWidth / 8);

    Value * const consumedItems = b->getConsumedItemCount("sourceBuffer");
    Value * const consumedBytes = b->CreateMul(consumedItems, CODE_UNIT_BYTES);
    Value * const consumedPageOffset = b->CreateAnd(consumedBytes, ConstantExpr::getNeg(MMAP_PAGE_SIZE));
    Value * const consumedBuffer = b->getRawOutputPointer("sourceBuffer", consumedPageOffset);
    Value * const readableBuffer = b->getScalarField("buffer");
    Value * const unnecessaryBytes = b->CreatePtrDiff(consumedBuffer, readableBuffer);

    // avoid calling madvise unless an actual page table change could occur
    b->CreateLikelyCondBr(b->CreateIsNotNull(unnecessaryBytes), dropPages, checkRemaining);

    b->SetInsertPoint(dropPages);
    // instruct the OS that it can safely drop any fully consumed pages
    b->CreateMAdvise(readableBuffer, unnecessaryBytes, CBuilder::ADVICE_DONTNEED);
    b->CreateBr(checkRemaining);

    // determine whether or not we've exhausted the "safe" region of the file buffer
    b->SetInsertPoint(checkRemaining);
    Value * const producedItems = b->getProducedItemCount("sourceBuffer");
    Value * const nextProducedItems = b->CreateAdd(producedItems, STRIDE_ITEMS);
    Value * const fileItems = b->getScalarField("fileItems");
    Value * const lastPage = b->CreateICmpULE(fileItems, nextProducedItems);
    b->CreateUnlikelyCondBr(lastPage, setTermination, exit);

    // If this is the last page, create a temporary buffer of up to two pages size, copy the unconsumed data
    // and zero any bytes that are not used.
    b->SetInsertPoint(setTermination);
    Value * const consumedOffset = b->CreateAnd(consumedItems, ConstantExpr::getNeg(BLOCK_WIDTH));
    Value * const readStart = b->getRawOutputPointer("sourceBuffer", consumedOffset);
    Value * const readEnd = b->getRawOutputPointer("sourceBuffer", fileItems);

    DataLayout DL(b->getModule());
    Type * const intPtrTy = DL.getIntPtrType(readEnd->getType());
    Value * const readEndInt = b->CreatePtrToInt(readEnd, intPtrTy);
    Value * const readStartInt = b->CreatePtrToInt(readStart, intPtrTy);
    Value * unconsumedBytes = b->CreateSub(readEndInt, readStartInt);
    unconsumedBytes = b->CreateTrunc(unconsumedBytes, b->getSizeTy());
    Value * const bufferSize = b->CreateRoundUp(b->CreateAdd(unconsumedBytes, PADDING_SIZE), STRIDE_BYTES);
    Value * const buffer = b->CreateAlignedMalloc(bufferSize, b->getCacheAlignment());
    b->CreateMemCpy(buffer, readStart, unconsumedBytes, 1);
    b->CreateMemZero(b->CreateGEP(buffer, unconsumedBytes), b->CreateSub(bufferSize, unconsumedBytes), 1);
    // get the difference between our base and from position then compute an offsetted temporary buffer address
    Value * const base = b->getBaseAddress("sourceBuffer");
    Value * const baseInt = b->CreatePtrToInt(base, intPtrTy);
    Value * const diff = b->CreateSub(baseInt, readStartInt);
    Value * const offsettedBuffer = b->CreateGEP(buffer, diff);
    PointerType * const codeUnitPtrTy = b->getIntNTy(codeUnitWidth)->getPointerTo();
    b->setScalarField("ancillaryBuffer", b->CreatePointerCast(buffer, codeUnitPtrTy));
    b->setBaseAddress("sourceBuffer", b->CreatePointerCast(offsettedBuffer, codeUnitPtrTy));
    b->setTerminationSignal();
    b->setProducedItemCount("sourceBuffer", fileItems);
    b->CreateBr(exit);

    b->SetInsertPoint(exit);
}
void MMapSourceKernel::freeBuffer(const std::unique_ptr<KernelBuilder> & b, const unsigned codeUnitWidth) {
    b->CreateFree(b->getScalarField("ancillaryBuffer"));
    Value * const fileItems = b->getScalarField("fileItems");
    Constant * const CODE_UNIT_BYTES = b->getSize(codeUnitWidth / 8);
    Value * const fileSize = b->CreateMul(fileItems, CODE_UNIT_BYTES);
    b->CreateMUnmap(b->getScalarField("buffer"), fileSize);
}

/// READ SOURCE KERNEL

void ReadSourceKernel::generateInitializeMethod(const unsigned codeUnitWidth, const unsigned stride, const std::unique_ptr<KernelBuilder> & b) {
    ConstantInt * const bufferItems = b->getSize(stride * 4);
    const auto codeUnitSize = codeUnitWidth / 8;
    ConstantInt * const bufferBytes = b->getSize(stride * 4 * codeUnitSize);
    PointerType * const codeUnitPtrTy = b->getIntNTy(codeUnitWidth)->getPointerTo();
    Value * const buffer = b->CreatePointerCast(b->CreateCacheAlignedMalloc(bufferBytes), codeUnitPtrTy);
    b->setBaseAddress("sourceBuffer", buffer);
    b->setScalarField("ancillaryBuffer", ConstantPointerNull::get(codeUnitPtrTy));
    b->setScalarField("buffer", buffer);
    b->setCapacity("sourceBuffer", bufferItems);
}

void ReadSourceKernel::generateDoSegmentMethod(const unsigned codeUnitWidth, const unsigned stride, const std::unique_ptr<KernelBuilder> & b) {

    ConstantInt * const itemsToRead = b->getSize(stride);
    ConstantInt * const codeUnitBytes = b->getSize(codeUnitWidth / 8);

    BasicBlock * const moveData = b->CreateBasicBlock("MoveData");
    BasicBlock * const prepareBuffer = b->CreateBasicBlock("PrepareBuffer");
    BasicBlock * const readData = b->CreateBasicBlock("ReadData");
    BasicBlock * const setTermination = b->CreateBasicBlock("SetTermination");
    BasicBlock * const readExit = b->CreateBasicBlock("ReadExit");

    // Can we append to our existing buffer without impacting any subsequent kernel?
    Value * const produced = b->getProducedItemCount("sourceBuffer");
    Value * const itemsPending = b->CreateAdd(produced, itemsToRead);
    Value * const capacity = b->getCapacity("sourceBuffer");
    Value * const readEnd = b->getRawOutputPointer("sourceBuffer", itemsPending);
    Value * const baseBuffer = b->getScalarField("buffer");
    Value * const bufferLimit = b->CreateGEP(baseBuffer, capacity);
    b->CreateLikelyCondBr(b->CreateICmpULE(readEnd, bufferLimit), readData, moveData);

    // No. If we can copy the unconsumed data back to the start of the buffer *and* write a full
    // segment of data without overwriting the currently unconsumed data, do so since it won't
    // affect any potential consumer that could be using the "stale" output base pointer.
    b->SetInsertPoint(moveData);

    // Determine how much data has been consumed and how much needs to be copied back, noting
    // that our "unproduced" data must be block aligned.
    BasicBlock * const copyBack = b->CreateBasicBlock("CopyBack");
    BasicBlock * const expandAndCopyBack = b->CreateBasicBlock("ExpandAndCopyBack");
    const auto blockSize = b->getBitBlockWidth() / 8;
    Value * const consumedItems = b->getConsumedItemCount("sourceBuffer");
    ConstantInt * const BLOCK_WIDTH = b->getSize(b->getBitBlockWidth());
    Constant * const ALIGNMENT_MASK = ConstantExpr::getNeg(BLOCK_WIDTH);
    Value * const consumed = b->CreateAnd(consumedItems, ALIGNMENT_MASK);
    Value * const unreadData = b->getRawOutputPointer("sourceBuffer", consumed);
    Value * const remainingItems = b->CreateSub(produced, consumed);
    Value * const potentialItems = b->CreateAdd(remainingItems, itemsToRead);
    Value * const remainingBytes = b->CreateMul(remainingItems, codeUnitBytes);
    // Have we consumed enough data that we can safely copy back the unconsumed data and still
    // leave enough space for one segment without needing a temporary buffer?
    Value * const canCopy = b->CreateICmpULT(b->CreateGEP(baseBuffer, potentialItems), unreadData);
    b->CreateLikelyCondBr(canCopy, copyBack, expandAndCopyBack);

    // If so, just copy the data ...
    b->SetInsertPoint(copyBack);
    b->CreateMemCpy(baseBuffer, unreadData, remainingBytes, blockSize);
    BasicBlock * const copyBackExit = b->GetInsertBlock();
    b->CreateBr(prepareBuffer);

    // Otherwise, allocate a buffer with twice the capacity and copy the unconsumed data back into it
    b->SetInsertPoint(expandAndCopyBack);
    Value * const expandedCapacity = b->CreateShl(capacity, 1);
    Value * const expandedBytes = b->CreateMul(expandedCapacity, codeUnitBytes);
    Value * const expandedBuffer = b->CreatePointerCast(b->CreateCacheAlignedMalloc(expandedBytes), unreadData->getType());
    b->CreateMemCpy(expandedBuffer, unreadData, remainingBytes, blockSize);
    // Free the prior buffer if it exists
    Value * const ancillaryBuffer = b->getScalarField("ancillaryBuffer");
    b->setScalarField("ancillaryBuffer", baseBuffer);
    b->CreateFree(ancillaryBuffer);
    b->setScalarField("buffer", expandedBuffer);
    b->setCapacity("sourceBuffer", expandedCapacity);
    BasicBlock * const expandAndCopyBackExit = b->GetInsertBlock();
    b->CreateBr(prepareBuffer);

    b->SetInsertPoint(prepareBuffer);
    PHINode * const newBaseBuffer = b->CreatePHI(baseBuffer->getType(), 2);
    newBaseBuffer->addIncoming(baseBuffer, copyBackExit);
    newBaseBuffer->addIncoming(expandedBuffer, expandAndCopyBackExit);
    Value * const newBaseAddress = b->CreateGEP(newBaseBuffer, b->CreateNeg(consumed));
    b->setBaseAddress("sourceBuffer", newBaseAddress);
    b->CreateBr(readData);

    // Regardless of whether we're simply appending data or had to allocate a new buffer, read a new page
    // of data into the input source buffer. If we fail to read a full page ...
    b->SetInsertPoint(readData);
    Value * const sourceBuffer = b->getRawOutputPointer("sourceBuffer", produced);
    Value * const fd = b->getScalarField("fileDescriptor");
    Constant * const bytesToRead = ConstantExpr::getMul(itemsToRead, codeUnitBytes);
    Value * const bytesRead = b->CreateReadCall(fd, sourceBuffer, bytesToRead);
    Value * const itemsRead = b->CreateUDiv(bytesRead, codeUnitBytes);
    Value * const itemsBuffered = b->CreateAdd(produced, itemsRead);
    b->CreateUnlikelyCondBr(b->CreateICmpULT(itemsBuffered, itemsPending), setTermination, readExit);

    // ... set the termination signal.
    b->SetInsertPoint(setTermination);
    Value * const bytesToZero = b->CreateMul(b->CreateSub(itemsPending, itemsBuffered), codeUnitBytes);
    b->CreateMemZero(b->getRawOutputPointer("sourceBuffer", itemsBuffered), bytesToZero);
    b->setScalarField("fileItems", itemsBuffered);
    b->setTerminationSignal();
    b->setProducedItemCount("sourceBuffer", itemsBuffered);
    b->CreateBr(readExit);

    b->SetInsertPoint(readExit);
}

void ReadSourceKernel::freeBuffer(const std::unique_ptr<KernelBuilder> & b) {
    b->CreateFree(b->getScalarField("ancillaryBuffer"));
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
    Value * const useMMap = b->CreateIsNotNull(b->getScalarField("useMMap"));
    b->CreateCondBr(useMMap, finalizeMMap, finalizeRead);
    b->SetInsertPoint(finalizeMMap);
    MMapSourceKernel::freeBuffer(b, mCodeUnitWidth);
    b->CreateBr(finalizeDone);
    b->SetInsertPoint(finalizeRead);
    ReadSourceKernel::freeBuffer(b);
    b->CreateBr(finalizeDone);
    b->SetInsertPoint(finalizeDone);
}

void FDSourceKernel::generateInitializeMethod(const std::unique_ptr<KernelBuilder> & b) {
    BasicBlock * initializeRead = b->CreateBasicBlock("initializeRead");
    BasicBlock * checkFileSize = b->CreateBasicBlock("checkFileSize");
    BasicBlock * initializeMMap = b->CreateBasicBlock("initializeMMap");
    BasicBlock * initializeDone = b->CreateBasicBlock("initializeDone");

    // The source will use MMapSource or readSoure kernel logic depending on the useMMap
    // parameter, possibly overridden.

    Value * const useMMap = b->getScalarField("useMMap");
    Constant * const ZERO = ConstantInt::getNullValue(useMMap->getType());
    // if the fileDescriptor is 0, the file is stdin, use readSource kernel logic.
    Value * const fd = b->getScalarField("fileDescriptor");
    Value * const notStdIn = b->CreateICmpNE(fd, b->getInt32(STDIN_FILENO));
    Value * const tryMMap = b->CreateICmpNE(useMMap, ZERO);
    b->CreateCondBr(b->CreateAnd(tryMMap, notStdIn), checkFileSize, initializeRead);

    b->SetInsertPoint(checkFileSize);
    // If the fileSize is 0, we may have a virtual file such as /proc/cpuinfo
    Value * const fileSize = b->CreateCall(mFileSizeFunction, fd);
    Value * const emptyFile = b->CreateIsNotNull(fileSize);
    b->CreateUnlikelyCondBr(emptyFile, initializeRead, initializeMMap);

    b->SetInsertPoint(initializeMMap);
    MMapSourceKernel::generateInitializeMethod(mFileSizeFunction, mCodeUnitWidth, mStride, b);
    b->CreateBr(initializeDone);

    b->SetInsertPoint(initializeRead);
    // Ensure that readSource logic is used throughout.
    b->setScalarField("useMMap", ZERO);
    ReadSourceKernel::generateInitializeMethod(mCodeUnitWidth, mStride,b);
    b->CreateBr(initializeDone);

    b->SetInsertPoint(initializeDone);
}

void FDSourceKernel::generateDoSegmentMethod(const std::unique_ptr<KernelBuilder> & b) {
    BasicBlock * DoSegmentRead = b->CreateBasicBlock("DoSegmentRead");
    BasicBlock * DoSegmentMMap = b->CreateBasicBlock("DoSegmentMMap");
    BasicBlock * DoSegmentDone = b->CreateBasicBlock("DoSegmentDone");
    Value * const useMMap = b->CreateIsNotNull(b->getScalarField("useMMap"));
    b->CreateCondBr(useMMap, DoSegmentMMap, DoSegmentRead);
    b->SetInsertPoint(DoSegmentMMap);
    MMapSourceKernel::generateDoSegmentMethod(mCodeUnitWidth, mStride, b);
    b->CreateBr(DoSegmentDone);
    b->SetInsertPoint(DoSegmentRead);
    ReadSourceKernel::generateDoSegmentMethod(mCodeUnitWidth, mStride, b);
    b->CreateBr(DoSegmentDone);
    b->SetInsertPoint(DoSegmentDone);
}

/// MEMORY SOURCE KERNEL

void MemorySourceKernel::generateInitializeMethod(const std::unique_ptr<KernelBuilder> & b) {
    Value * const fileSource = b->getScalarField("fileSource");
    b->setBaseAddress("sourceBuffer", fileSource);
    if (LLVM_UNLIKELY(codegen::DebugOptionIsSet(codegen::EnableAsserts))) {
        b->CreateAssert(fileSource, getName() + " fileSource cannot be null");
    }
    Value * const fileItems = b->getScalarField("fileItems");
    b->setCapacity("sourceBuffer", fileItems);
}

void MemorySourceKernel::generateDoSegmentMethod(const std::unique_ptr<KernelBuilder> & b) {

    Constant * const STRIDE_ITEMS = b->getSize(getStride());
    Constant * const STRIDE_SIZE = b->getSize(getStride() * mCodeUnitWidth);
    Constant * const BLOCK_WIDTH = b->getSize(b->getBitBlockWidth());

    BasicBlock * const createTemporary = b->CreateBasicBlock("createTemporary");
    BasicBlock * const exit = b->CreateBasicBlock("exit");

    Value * const fileItems = b->getScalarField("fileItems");
    Value * const producedItems = b->getProducedItemCount("sourceBuffer");
    Value * const nextProducedItems = b->CreateAdd(producedItems, STRIDE_ITEMS);
    Value * const lastPage = b->CreateICmpULE(fileItems, nextProducedItems);
    b->CreateUnlikelyCondBr(lastPage, createTemporary, exit);

    b->SetInsertPoint(createTemporary);
    Value * const consumedItems = b->getConsumedItemCount("sourceBuffer");
    Value * readStart = nullptr;
    Value * readEnd = nullptr;

    // compute the range of our unconsumed buffer slice
    if (LLVM_UNLIKELY(mStreamSetCount > 1)) {
        Constant * const ZERO = b->getSize(0);
        const StreamSetBuffer * const sourceBuffer = getOutputStreamSetBuffer("sourceBuffer");
        Value * const fromIndex = b->CreateUDiv(consumedItems, BLOCK_WIDTH);
        Value * const sourceBufferBaseAddress = sourceBuffer->getBaseAddress(b);
        readStart = sourceBuffer->getStreamBlockPtr(b, sourceBufferBaseAddress, ZERO, fromIndex);
        Value * const toIndex = b->CreateCeilUDiv(fileItems, BLOCK_WIDTH);
        // since we know this is an ExternalBuffer, we don't need to consider any potential modulus calculations.
        readEnd = sourceBuffer->getStreamBlockPtr(b, sourceBufferBaseAddress, ZERO, toIndex);
    } else {
        // make sure our copy is block-aligned
        Value * const consumedOffset = b->CreateAnd(consumedItems, ConstantExpr::getNeg(BLOCK_WIDTH));
        readStart = b->getRawOutputPointer("sourceBuffer", consumedOffset);
        readEnd = b->getRawOutputPointer("sourceBuffer", fileItems);
    }

    DataLayout DL(b->getModule());
    Type * const intPtrTy = DL.getIntPtrType(readEnd->getType());
    Value * const readEndInt = b->CreatePtrToInt(readEnd, intPtrTy);
    Value * const readStartInt = b->CreatePtrToInt(readStart, intPtrTy);
    Value * const unconsumedBytes = b->CreateTrunc(b->CreateSub(readEndInt, readStartInt), b->getSizeTy());
    Value * const bufferSize = b->CreateRoundUp(b->CreateAdd(unconsumedBytes, BLOCK_WIDTH), STRIDE_SIZE);
    Value * const buffer = b->CreateAlignedMalloc(bufferSize, b->getCacheAlignment());
    PointerType * const codeUnitPtrTy = b->getIntNTy(mCodeUnitWidth)->getPointerTo();
    b->setScalarField("ancillaryBuffer", b->CreatePointerCast(buffer, codeUnitPtrTy));
    b->CreateMemCpy(buffer, readStart, unconsumedBytes, 1);
    b->CreateMemZero(b->CreateGEP(buffer, unconsumedBytes), b->CreateSub(bufferSize, unconsumedBytes), 1);

    // get the difference between our base and from position then compute an offsetted temporary buffer address
    Value * const base = b->getBaseAddress("sourceBuffer");
    Value * const baseInt = b->CreatePtrToInt(base, intPtrTy);
    Value * const diff = b->CreateSub(baseInt, readStartInt);
    Value * const offsettedBuffer = b->CreateGEP(buffer, diff);
    // set the temporary buffer as the new source buffer
    b->setBaseAddress("sourceBuffer", b->CreatePointerCast(offsettedBuffer, codeUnitPtrTy));
    b->setTerminationSignal();
    b->setProducedItemCount("sourceBuffer", fileItems);
    b->CreateBr(exit);

    b->SetInsertPoint(exit);
}

void MemorySourceKernel::generateFinalizeMethod(const std::unique_ptr<KernelBuilder> & b) {
    b->CreateFree(b->getScalarField("ancillaryBuffer"));
}

MMapSourceKernel::MMapSourceKernel(const std::unique_ptr<kernel::KernelBuilder> & b, Scalar * const fd, StreamSet * const outputStream)
: SegmentOrientedKernel(b, "mmap_source" + std::to_string(codegen::SegmentSize) + "@" + std::to_string(outputStream->getFieldWidth())
// input streams
,{}
// output streams
,{Binding{"sourceBuffer", outputStream, FixedRate(), { ManagedBuffer(), Linear() }}}
// input scalars
,{Binding{"fileDescriptor", fd}}
// output scalars
,{Binding{b->getSizeTy(), "fileItems"}}
// internal scalars
,{})
, mCodeUnitWidth(outputStream->getFieldWidth())
, mFileSizeFunction(nullptr) {
    PointerType * const codeUnitPtrTy = b->getIntNTy(mCodeUnitWidth)->getPointerTo();
    addInternalScalar(codeUnitPtrTy, "buffer");
    addInternalScalar(codeUnitPtrTy, "ancillaryBuffer");
    addAttribute(MustExplicitlyTerminate());
    setStride(codegen::SegmentSize);
}

ReadSourceKernel::ReadSourceKernel(const std::unique_ptr<kernel::KernelBuilder> & b, Scalar * const fd, StreamSet * const outputStream)
: SegmentOrientedKernel(b, "read_source" + std::to_string(codegen::SegmentSize) + "@" + std::to_string(outputStream->getFieldWidth())
// input streams
,{}
// output streams
,{Binding{"sourceBuffer", outputStream, FixedRate(), { ManagedBuffer(), Linear() }}}
// input scalars
,{Binding{"fileDescriptor", fd}}
// output scalars
,{Binding{b->getSizeTy(), "fileItems"}}
// internal scalars
,{})
, mCodeUnitWidth(outputStream->getFieldWidth()) {
    PointerType * const codeUnitPtrTy = b->getIntNTy(mCodeUnitWidth)->getPointerTo();
    addInternalScalar(codeUnitPtrTy, "buffer");
    addInternalScalar(codeUnitPtrTy, "ancillaryBuffer");
    addAttribute(MustExplicitlyTerminate());
    setStride(codegen::SegmentSize);
}


FDSourceKernel::FDSourceKernel(const std::unique_ptr<kernel::KernelBuilder> & b, Scalar * const useMMap, Scalar * const fd, StreamSet * const outputStream)
: SegmentOrientedKernel(b, "FD_source" + std::to_string(codegen::SegmentSize) + "@" + std::to_string(outputStream->getFieldWidth())
// input streams
,{}
// output stream
,{Binding{"sourceBuffer", outputStream, FixedRate(), { ManagedBuffer(), Linear() }}}
// input scalar
,{Binding{"useMMap", useMMap}
, Binding{"fileDescriptor", fd}}
// output scalar
,{Binding{b->getSizeTy(), "fileItems"}}
// internal scalars
,{})
, mCodeUnitWidth(outputStream->getFieldWidth())
, mFileSizeFunction(nullptr) {
    PointerType * const codeUnitPtrTy = b->getIntNTy(mCodeUnitWidth)->getPointerTo();
    addInternalScalar(codeUnitPtrTy, "buffer");
    addInternalScalar(codeUnitPtrTy, "ancillaryBuffer");
    addAttribute(MustExplicitlyTerminate());
    setStride(codegen::SegmentSize);
}

MemorySourceKernel::MemorySourceKernel(const std::unique_ptr<kernel::KernelBuilder> & b, Scalar * fileSource, Scalar * fileItems, StreamSet * const outputStream)
: SegmentOrientedKernel(b, "memory_source" + std::to_string(codegen::SegmentSize) + "@" + std::to_string(outputStream->getFieldWidth()) + ":" + std::to_string(outputStream->getNumElements()),
// input streams
{},
// output stream
{Binding{"sourceBuffer", outputStream, FixedRate(), { ManagedBuffer(), Linear() }}},
// input scalar
{Binding{"fileSource", fileSource}, Binding{"fileItems", fileItems}},
{},
// internal scalar
{InternalScalar{fileSource->getType(), "buffer"}
,InternalScalar{fileSource->getType(), "ancillaryBuffer"}
})
, mStreamSetCount(outputStream->getNumElements())
, mCodeUnitWidth(outputStream->getFieldWidth()) {
    addAttribute(MustExplicitlyTerminate());
    setStride(codegen::SegmentSize);
}

}
