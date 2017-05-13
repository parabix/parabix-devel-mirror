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

void FileSink::generateMultiBlockLogic(const std::unique_ptr<KernelBuilder> & iBuilder) {
    BasicBlock * closeFile = iBuilder->CreateBasicBlock("closeFile");
    BasicBlock * fileOutExit = iBuilder->CreateBasicBlock("fileOutExit");
    
    PointerType * i8PtrTy = iBuilder->getInt8PtrTy();
    Constant * itemBytes = iBuilder->getSize(mCodeUnitWidth / 8);
    Value * fileDes = iBuilder->getScalarField("fileDes");

    Function::arg_iterator args = mCurrentMethod->arg_begin();
    /* self = */ args++;
    Value * itemsToDo = &*(args++);
    Value * codeUnitBuffer = &*(args++);
    
    Value * bytesToDo = mCodeUnitWidth == 8 ? itemsToDo : iBuilder->CreateMul(itemsToDo, itemBytes);
    Value * bytePtr = iBuilder->CreatePointerCast(codeUnitBuffer, i8PtrTy);
    
    iBuilder->CreateWriteCall(fileDes, bytePtr, bytesToDo);
    iBuilder->CreateCondBr(iBuilder->CreateICmpULT(itemsToDo, iBuilder->getSize(iBuilder->getBitBlockWidth())), closeFile, fileOutExit);
    
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
: MultiBlockKernel("filesink", {Binding{iBuilder->getStreamSetTy(1, codeUnitWidth), "codeUnitBuffer"}}, {},
                {Binding{iBuilder->getInt8PtrTy(), "fileName"}}, {}, {Binding{iBuilder->getInt8PtrTy(), "tmpFileName"}, Binding{iBuilder->getInt32Ty(), "fileDes"}})
, mCodeUnitWidth(codeUnitWidth) {
}

}




