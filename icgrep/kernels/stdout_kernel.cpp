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

void StdOutKernel::generateMultiBlockLogic(const std::unique_ptr<KernelBuilder> & b, llvm::Value * const numOfStrides) {
    Value * codeUnitBuffer = b->getInputStreamBlockPtr("codeUnitBuffer", b->getInt32(0));
    codeUnitBuffer = b->CreatePointerCast(codeUnitBuffer, b->getInt8PtrTy());
    Value * bytesToDo = mAccessibleInputItems[0];
    if (LLVM_UNLIKELY(mCodeUnitWidth > 8)) {
        bytesToDo = b->CreateMul(bytesToDo, b->getSize(mCodeUnitWidth / 8));
    } else if (LLVM_UNLIKELY(mCodeUnitWidth < 8)) {
        bytesToDo = b->CreateUDiv(bytesToDo, b->getSize(8 / mCodeUnitWidth));
    }
    b->CreateWriteCall(b->getInt32(1), codeUnitBuffer, bytesToDo);
}

StdOutKernel::StdOutKernel(const std::unique_ptr<kernel::KernelBuilder> & b, unsigned codeUnitWidth)
: MultiBlockKernel("stdout",
// input
{Binding{b->getStreamSetTy(1, codeUnitWidth), "codeUnitBuffer", FixedRate(), RequiresLinearAccess()}}
// output & scalars
, {}, {}, {}, {})
, mCodeUnitWidth(codeUnitWidth) {
    //setStride(getpagesize());
}

void FileSink::generateInitializeMethod(const std::unique_ptr<kernel::KernelBuilder> & b) {
    BasicBlock * setTerminationOnFailure = b->CreateBasicBlock("setTerminationOnFailure");
    BasicBlock * fileSinkInitExit = b->CreateBasicBlock("fileSinkInitExit");
    Value * fileName = b->getScalarField("fileName");
    Value * fileNameLength = b->CreateStrlenCall(fileName);
    // Make a temporary file name template with the characters "XXXXXX" appended 
    // as required by mkstemp.
    Constant * suffixPlusNullLength = b->getSize(7);
    Value * tmpFileNamePtr = b->CreatePointerCast(b->CreateMalloc(b->CreateAdd(fileNameLength, suffixPlusNullLength)), b->getInt8PtrTy());
    b->setScalarField("tmpFileName", tmpFileNamePtr);
    b->CreateMemCpy(tmpFileNamePtr, fileName, fileNameLength, 1);
#ifdef BACKUP_OLDFILE
    b->CreateMemCpy(b->CreateGEP(tmpFileNamePtr, fileNameLength), b->GetString(".saved"), suffixPlusNullLength, 1);
    b->CreateRenameCall(fileName, tmpFileNamePtr);
#else
    b->CreateUnlinkCall(fileName);
#endif
    b->CreateMemCpy(b->CreateGEP(tmpFileNamePtr, fileNameLength), b->GetString("XXXXXX"), suffixPlusNullLength, 1);
    Value * fileDes = b->CreateMkstempCall(tmpFileNamePtr);
    b->setScalarField("fileDes", fileDes);
    Value * failure = b->CreateICmpEQ(fileDes, b->getInt32(-1));
    b->CreateCondBr(failure, setTerminationOnFailure, fileSinkInitExit);

    b->SetInsertPoint(setTerminationOnFailure);
    b->setTerminationSignal();
    b->CreateBr(fileSinkInitExit);

    b->SetInsertPoint(fileSinkInitExit);
}

void FileSink::generateMultiBlockLogic(const std::unique_ptr<KernelBuilder> & b, Value * const numOfStrides) {
    Value * const fileDes = b->getScalarField("fileDes");
    Value * codeUnitBuffer = b->getInputStreamBlockPtr("codeUnitBuffer", b->getInt32(0));
    codeUnitBuffer = b->CreatePointerCast(codeUnitBuffer, b->getInt8PtrTy());
    Value * bytesToDo = mAccessibleInputItems[0];
    if (LLVM_UNLIKELY(mCodeUnitWidth > 8)) {
        bytesToDo = b->CreateMul(bytesToDo, b->getSize(mCodeUnitWidth / 8));
    } else if (LLVM_UNLIKELY(mCodeUnitWidth < 8)) {
        bytesToDo = b->CreateUDiv(bytesToDo, b->getSize(8 / mCodeUnitWidth));
    }    
    b->CreateWriteCall(fileDes, codeUnitBuffer, bytesToDo);
}

void FileSink::generateFinalizeMethod(const std::unique_ptr<KernelBuilder> & b) {
    Value * const fileDes = b->getScalarField("fileDes");
    b->CreateCloseCall(fileDes);
    Value * newFileNamePtr = b->getScalarField("fileName");
    Value * tmpFileNamePtr = b->getScalarField("tmpFileName");
    b->CreateRenameCall(tmpFileNamePtr, newFileNamePtr);
    b->CreateFree(tmpFileNamePtr);
}

FileSink::FileSink(const std::unique_ptr<kernel::KernelBuilder> & b, unsigned codeUnitWidth)
: MultiBlockKernel("filesink" + std::to_string(codeUnitWidth),
// input
{Binding{b->getStreamSetTy(1, codeUnitWidth), "codeUnitBuffer", FixedRate(), RequiresLinearAccess()}},
// output
{},
// scalars
{Binding{b->getInt8PtrTy(), "fileName"}}, {}, {Binding{b->getInt8PtrTy(), "tmpFileName"}, Binding{b->getInt32Ty(), "fileDes"}})
, mCodeUnitWidth(codeUnitWidth) {
    //setStride(getpagesize());
}

}




