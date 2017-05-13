/*
 *  Copyright (c) 2016 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 */
#include "stdout_kernel.h"
#include <llvm/IR/Module.h>
#include <kernels/kernel_builder.h>
#include <kernels/streamset.h>

namespace llvm { class Type; }

using namespace llvm;
using namespace parabix;

namespace kernel {

// Rather than using doBlock logic to write one block at a time, this custom
// doSegment method attempts to write the entire segment with a single write call.
// However, if the segment spans two memory areas (e.g., because of wraparound),
// then two write calls are made.
void StdOutKernel::generateMultiBlockLogic(const std::unique_ptr<KernelBuilder> & iBuilder) {
    PointerType * i8PtrTy = iBuilder->getInt8PtrTy();
    Constant * itemBytes = iBuilder->getSize(mCodeUnitWidth / 8);
    
    Function::arg_iterator args = mCurrentMethod->arg_begin();
    /* self = */ args++;
    Value * itemsToDo = &*(args++);
    Value * codeUnitBuffer = &*(args++);

    Value * bytesToDo = mCodeUnitWidth == 8 ? itemsToDo : iBuilder->CreateMul(itemsToDo, itemBytes);
    Value * bytePtr = iBuilder->CreatePointerCast(codeUnitBuffer, i8PtrTy);
    iBuilder->CreateWriteCall(iBuilder->getInt32(1), bytePtr, bytesToDo);
}

StdOutKernel::StdOutKernel(const std::unique_ptr<kernel::KernelBuilder> & iBuilder, unsigned codeUnitWidth)
: MultiBlockKernel("stdout", {Binding{iBuilder->getStreamSetTy(1, codeUnitWidth), "codeUnitBuffer"}}, {}, {}, {}, {})
, mCodeUnitWidth(codeUnitWidth) {
    setNoTerminateAttribute(true);
}

void FileSink::generateInitializeMethod(const std::unique_ptr<kernel::KernelBuilder> & iBuilder) {
    BasicBlock * setTerminationOnFailure = iBuilder->CreateBasicBlock("setTerminationOnFailure");
    BasicBlock * fileSinkInitExit = iBuilder->CreateBasicBlock("fileSinkInitExit");
    Value * fileName = iBuilder->getScalarField("fileName");
    Value * fileNameLength = iBuilder->CreateStrlenCall(fileName);
    // Make a temporary file name template with the characters "XXXXXX" appended 
    // as required by mkstemp.
    Constant * suffixPlusNullLength = iBuilder->getSize(7);
    Value * tmpFileNamePtr = iBuilder->CreatePointerCast(iBuilder->CreateMalloc(iBuilder->CreateAdd(fileNameLength, suffixPlusNullLength)), iBuilder->getInt8PtrTy());
    iBuilder->setScalarField("tmpFileName", tmpFileNamePtr);
    iBuilder->CreateMemCpy(tmpFileNamePtr, fileName, fileNameLength, 1);
#ifdef BACKUP_OLDFILE
    iBuilder->CreateMemCpy(iBuilder->CreateGEP(tmpFileNamePtr, fileNameLength), iBuilder->GetString(".saved"), suffixPlusNullLength, 1);
    iBuilder->CreateRenameCall(fileName, tmpFileNamePtr);
#else
    iBuilder->CreateUnlinkCall(fileName);
#endif
    iBuilder->CreateMemCpy(iBuilder->CreateGEP(tmpFileNamePtr, fileNameLength), iBuilder->GetString("XXXXXX"), suffixPlusNullLength, 1);
    Value * fileDes = iBuilder->CreateMkstempCall(tmpFileNamePtr);
    iBuilder->setScalarField("fileDes", fileDes);
    Value * failure = iBuilder->CreateICmpEQ(fileDes, iBuilder->getInt32(-1));
    iBuilder->CreateCondBr(failure, setTerminationOnFailure, fileSinkInitExit);
    iBuilder->SetInsertPoint(setTerminationOnFailure);
    iBuilder->setTerminationSignal();
    iBuilder->CreateBr(fileSinkInitExit);
    iBuilder->SetInsertPoint(fileSinkInitExit);
}

void FileSink::generateDoSegmentMethod(const std::unique_ptr<KernelBuilder> &iBuilder) {

    PointerType * i8PtrTy = iBuilder->getInt8PtrTy();

    BasicBlock * closeFile = iBuilder->CreateBasicBlock("closeFile");
    BasicBlock * fileOutExit = iBuilder->CreateBasicBlock("fileOutExit");
    Constant * blockItems = iBuilder->getSize(iBuilder->getBitBlockWidth());
    Constant * itemBytes = iBuilder->getSize(mCodeUnitWidth/8);

    Value * fileDes = iBuilder->getScalarField("fileDes");
    Value * available = iBuilder->getAvailableItemCount("codeUnitBuffer");
    Value * processed = iBuilder->getProcessedItemCount("codeUnitBuffer");
    Value * itemsToDo = iBuilder->CreateSub(available, processed);
    // There may be two memory areas if we are at the physical end of a circular buffer.
    const auto b  = getInputStreamSetBuffer("codeUnitBuffer");
    Value * wraparound = nullptr;
    if (isa<CircularBuffer>(b) || isa<CircularCopybackBuffer>(b)) {
        Value * accessible = iBuilder->getLinearlyAccessibleItems("codeUnitBuffer", processed);
        wraparound = iBuilder->CreateICmpULT(accessible, itemsToDo);
        itemsToDo = iBuilder->CreateSelect(wraparound, accessible, itemsToDo);
    }
    
    Value * byteOffset = iBuilder->CreateMul(iBuilder->CreateURem(processed, blockItems), itemBytes);
    Value * bytePtr = iBuilder->CreatePointerCast(iBuilder->getInputStreamBlockPtr("codeUnitBuffer", iBuilder->getInt32(0)), i8PtrTy);
    bytePtr = iBuilder->CreateGEP(bytePtr, byteOffset);
    Value * bytesToDo = mCodeUnitWidth == 8 ? itemsToDo : iBuilder->CreateMul(itemsToDo, itemBytes);
    iBuilder->CreateWriteCall(fileDes, bytePtr, bytesToDo);
    
    processed = iBuilder->CreateAdd(processed, itemsToDo);
    iBuilder->setProcessedItemCount("codeUnitBuffer", processed);
    
    // Now we may process the second area (if required).
    if (isa<CircularBuffer>(b) || isa<CircularCopybackBuffer>(b)) {
        BasicBlock * wrapAroundWrite = iBuilder->CreateBasicBlock("wrapAroundWrite");
        BasicBlock * checkFinal = iBuilder->CreateBasicBlock("checkFinal");
        iBuilder->CreateCondBr(wraparound, wrapAroundWrite, checkFinal);
        iBuilder->SetInsertPoint(wrapAroundWrite);
        
        // Calculate from the updated value of processed;
        byteOffset = iBuilder->CreateMul(iBuilder->CreateURem(processed, blockItems), itemBytes);
        Value * bytePtr = iBuilder->CreatePointerCast(iBuilder->getInputStreamBlockPtr("codeUnitBuffer", iBuilder->getInt32(0)), i8PtrTy);
        bytePtr = iBuilder->CreateGEP(bytePtr, byteOffset);
        itemsToDo = iBuilder->CreateSub(available, processed);
        bytesToDo = mCodeUnitWidth == 8 ? itemsToDo : iBuilder->CreateMul(itemsToDo, itemBytes);
        iBuilder->CreateWriteCall(fileDes, bytePtr, bytesToDo);
        processed = iBuilder->CreateAdd(processed, itemsToDo);
        iBuilder->setProcessedItemCount("codeUnitBuffer", available);
        iBuilder->CreateBr(checkFinal);
        iBuilder->SetInsertPoint(checkFinal);
    }
    iBuilder->CreateCondBr(mIsFinal, closeFile, fileOutExit);

    iBuilder->SetInsertPoint(closeFile);
    iBuilder->CreateCloseCall(fileDes);
    Value * newFileNamePtr = iBuilder->getScalarField("fileName");
    Value * tmpFileNamePtr = iBuilder->getScalarField("tmpFileName");
    iBuilder->CreateRenameCall(tmpFileNamePtr, newFileNamePtr);
    iBuilder->CreateFree(tmpFileNamePtr);
    
    iBuilder->CreateBr(fileOutExit);

    iBuilder->SetInsertPoint(fileOutExit);
}

FileSink::FileSink(const std::unique_ptr<kernel::KernelBuilder> & iBuilder, unsigned codeUnitWidth)
: SegmentOrientedKernel("filesink", {Binding{iBuilder->getStreamSetTy(1, codeUnitWidth), "codeUnitBuffer"}}, {},
                {Binding{iBuilder->getInt8PtrTy(), "fileName"}}, {}, {Binding{iBuilder->getInt8PtrTy(), "tmpFileName"}, Binding{iBuilder->getInt32Ty(), "fileDes"}})
, mCodeUnitWidth(codeUnitWidth) {
}

}




