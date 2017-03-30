/*
 *  Copyright (c) 2017 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 */
#include <kernels/stdin_kernel.h>
#include <llvm/IR/Module.h>
#include <kernels/kernel.h>
#include <IR_Gen/idisa_builder.h>

using namespace llvm;

namespace kernel {

void StdInKernel::generateDoSegmentMethod(Value * /* doFinal */, const std::vector<Value *> & /* producerPos */) {

    BasicBlock * setTermination = CreateBasicBlock("setTermination");
    BasicBlock * stdInExit = CreateBasicBlock("stdInExit");
    ConstantInt * segmentBytes = iBuilder->getSize(mSegmentBlocks * iBuilder->getBitBlockWidth());
    ConstantInt * segmentBytes2 = iBuilder->getSize(2 * mSegmentBlocks * iBuilder->getBitBlockWidth());
    // on the first segment, we buffer twice the data to ensure the ScanMatch kernel can safely check for a non-LF line break
    Value * const itemsAlreadyRead = getProducedItemCount("codeUnitBuffer");
    Value * const bytesToRead = iBuilder->CreateSelect(iBuilder->CreateICmpEQ(itemsAlreadyRead, iBuilder->getSize(0)), segmentBytes2, segmentBytes);
    reserveBytes("codeUnitBuffer", bytesToRead);
    Value * const bytePtr = iBuilder->CreatePointerCast(getOutputStreamBlockPtr("codeUnitBuffer", iBuilder->getInt32(0)), iBuilder->getInt8PtrTy());
    Value * const bytesRead = iBuilder->CreateReadCall(iBuilder->getInt32(STDIN_FILENO), bytePtr, bytesToRead);
    Value * const itemsRead = iBuilder->CreateAdd(itemsAlreadyRead, iBuilder->CreateUDiv(bytesRead, iBuilder->getSize(mCodeUnitWidth / 8)));
    setProducedItemCount("codeUnitBuffer", itemsRead);
    iBuilder->CreateCondBr(iBuilder->CreateICmpEQ(bytesRead, ConstantInt::getNullValue(bytesRead->getType())), setTermination, stdInExit);
    iBuilder->SetInsertPoint(setTermination);
    setTerminationSignal();
    iBuilder->CreateBr(stdInExit);
    stdInExit->moveAfter(iBuilder->GetInsertBlock());
    iBuilder->SetInsertPoint(stdInExit);
}

StdInKernel::StdInKernel(IDISA::IDISA_Builder * iBuilder, unsigned blocksPerSegment, unsigned codeUnitWidth)
: SegmentOrientedKernel(iBuilder, "stdin_source", {}, {Binding{iBuilder->getStreamSetTy(1, codeUnitWidth), "codeUnitBuffer"}}, {}, {}, {})
, mSegmentBlocks(blocksPerSegment)
, mCodeUnitWidth(codeUnitWidth) {
    
}

void FileSource::generateInitMethod() {
    BasicBlock * setTerminationOnFailure = CreateBasicBlock("setTerminationOnFailure");
    BasicBlock * fileSourceInitExit = CreateBasicBlock("fileSourceInitExit");
    Value * handle = iBuilder->CreateFOpenCall(getScalarField("fileName"), iBuilder->CreateGlobalStringPtr("r"));
    setScalarField("IOstreamPtr", handle);
    Value * failure = iBuilder->CreateICmpEQ(iBuilder->CreatePtrToInt(handle, iBuilder->getSizeTy()), iBuilder->getSize(0));
    iBuilder->CreateCondBr(failure, setTerminationOnFailure, fileSourceInitExit);
    iBuilder->SetInsertPoint(setTerminationOnFailure);
    setTerminationSignal();
    iBuilder->CreateBr(fileSourceInitExit);
    iBuilder->SetInsertPoint(fileSourceInitExit);
}
    
void FileSource::generateDoSegmentMethod(Value * /* doFinal */, const std::vector<Value *> & /* producerPos */) {

    BasicBlock * closeFile = CreateBasicBlock("closeFile");
    BasicBlock * fileSourceExit = CreateBasicBlock("fileSourceExit");
    Constant * itemBytes = iBuilder->getSize(mCodeUnitWidth/8);
    
    Value * produced = getProducedItemCount("codeUnitBuffer");
    Value * bytePtr = getOutputStreamBlockPtr("codeUnitBuffer", iBuilder->getInt32(0));
    bytePtr = iBuilder->CreatePointerCast(bytePtr, iBuilder->getInt8PtrTy());

    Value * IOstreamPtr = getScalarField("IOstreamPtr");
    Value * itemsToDo = iBuilder->getSize(mSegmentBlocks * iBuilder->getBitBlockWidth());
    Value * nRead = iBuilder->CreateFReadCall(bytePtr, itemsToDo, itemBytes, IOstreamPtr);
    produced = iBuilder->CreateAdd(produced, nRead);
    setProducedItemCount("codeUnitBuffer", produced);
    Value * lessThanFullSegment = iBuilder->CreateICmpULT(nRead, itemsToDo);
    iBuilder->CreateCondBr(lessThanFullSegment, closeFile, fileSourceExit);

    iBuilder->SetInsertPoint(closeFile);
    iBuilder->CreateFCloseCall(IOstreamPtr);
    setTerminationSignal();
    iBuilder->CreateBr(fileSourceExit);
    
    iBuilder->SetInsertPoint(fileSourceExit);
    
}
    
FileSource::FileSource(IDISA::IDISA_Builder * iBuilder, unsigned blocksPerSegment, unsigned codeUnitWidth)
: SegmentOrientedKernel(iBuilder, "filesink", {Binding{iBuilder->getStreamSetTy(1, codeUnitWidth), "codeUnitBuffer"}}, {},
                {Binding{iBuilder->getInt8PtrTy(), "fileName"}}, {}, {Binding{iBuilder->getFILEptrTy(), "IOstreamPtr"}})
, mSegmentBlocks(blocksPerSegment)
, mCodeUnitWidth(codeUnitWidth) {
}

}
