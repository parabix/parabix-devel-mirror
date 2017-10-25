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

void StdOutKernel::generateMultiBlockLogic(const std::unique_ptr<KernelBuilder> & iBuilder, llvm::Value * const /* numOfStrides */) {
    Value * codeUnitBuffer = iBuilder->getInputStreamBlockPtr("codeUnitBuffer", iBuilder->getInt32(0));
    codeUnitBuffer = iBuilder->CreatePointerCast(codeUnitBuffer, iBuilder->getInt8PtrTy());
    Value * bytesToDo = mAvailableItemCount[0];
    if (LLVM_UNLIKELY(mCodeUnitWidth > 8)) {
        bytesToDo = iBuilder->CreateMul(bytesToDo, iBuilder->getSize(mCodeUnitWidth / 8));
    } else if (LLVM_UNLIKELY(mCodeUnitWidth < 8)) {
        bytesToDo = iBuilder->CreateUDiv(bytesToDo, iBuilder->getSize(8 / mCodeUnitWidth));
    }
    iBuilder->CreateWriteCall(iBuilder->getInt32(1), codeUnitBuffer, bytesToDo);
}

StdOutKernel::StdOutKernel(const std::unique_ptr<kernel::KernelBuilder> & iBuilder, unsigned codeUnitWidth)
: MultiBlockKernel("stdout", {Binding{iBuilder->getStreamSetTy(1, codeUnitWidth), "codeUnitBuffer"}}, {}, {}, {}, {})
, mCodeUnitWidth(codeUnitWidth) {
    setNoTerminateAttribute(true);
    // setKernelStride(getpagesize());
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

void FileSink::generateMultiBlockLogic(const std::unique_ptr<KernelBuilder> & iBuilder, Value * const /* numOfStrides */) {
    BasicBlock * const closeFile = iBuilder->CreateBasicBlock("closeFile");
    BasicBlock * const fileOutExit = iBuilder->CreateBasicBlock("fileOutExit");

    Value * const fileDes = iBuilder->getScalarField("fileDes");
    Value * const codeUnitBuffer = iBuilder->CreatePointerCast(getStreamSetInputBufferPtr(0), iBuilder->getInt8PtrTy());
    Value * bytesToDo = mAvailableItemCount[0];
    if (LLVM_UNLIKELY(mCodeUnitWidth > 8)) {
        bytesToDo = iBuilder->CreateMul(bytesToDo, iBuilder->getSize(mCodeUnitWidth / 8));
    } else if (LLVM_UNLIKELY(mCodeUnitWidth < 8)) {
        bytesToDo = iBuilder->CreateUDiv(bytesToDo, iBuilder->getSize(8 / mCodeUnitWidth));
    }    
    iBuilder->CreateWriteCall(fileDes, codeUnitBuffer, bytesToDo);
    iBuilder->CreateUnlikelyCondBr(mIsFinal, closeFile, fileOutExit);

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
: MultiBlockKernel("filesink" + std::to_string(codeUnitWidth),
{Binding{iBuilder->getStreamSetTy(1, codeUnitWidth), "codeUnitBuffer"}},
{},
{Binding{iBuilder->getInt8PtrTy(), "fileName"}}, {}, {Binding{iBuilder->getInt8PtrTy(), "tmpFileName"}, Binding{iBuilder->getInt32Ty(), "fileDes"}})
, mCodeUnitWidth(codeUnitWidth) {
    // setKernelStride(getpagesize());
}

}




