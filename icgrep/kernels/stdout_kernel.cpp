/*
 *  Copyright (c) 2016 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 */
#include <kernels/stdout_kernel.h>
#include <kernels/kernel.h>
#include <IDISA/idisa_builder.h>
#include <llvm/IR/TypeBuilder.h>

namespace kernel {

static Function * create_write(Module * const mod) {
    Function * write = mod->getFunction("write");
    if (write == nullptr) {
        FunctionType *write_type =
        TypeBuilder<long(int, char *, long), false>::get(mod->getContext());
        write = cast<Function>(mod->getOrInsertFunction("write", write_type,
                                                         AttributeSet().addAttribute(mod->getContext(), 2U, Attribute::NoAlias)));
    }
    return write;
}

// The doBlock method is deprecated.   But incase it is used, just call doSegment with
// 1 as the number of blocks to do.
void stdOutKernel::generateDoBlockMethod() {
    IDISA::IDISA_Builder::InsertPoint savePoint = iBuilder->saveIP();
    Module * m = iBuilder->getModule();
    Function * doBlockFunction = m->getFunction(mKernelName + doBlock_suffix);
    Function * doSegmentFunction = m->getFunction(mKernelName + doSegment_suffix);
    iBuilder->SetInsertPoint(BasicBlock::Create(iBuilder->getContext(), "entry", doBlockFunction, 0));
    Value * self = getParameter(doBlockFunction, "self");
    iBuilder->CreateCall(doSegmentFunction, {self, ConstantInt::get(iBuilder->getSizeTy(), 1)});
    iBuilder->CreateRetVoid();
    iBuilder->restoreIP(savePoint);
}
            
void stdOutKernel::generateDoSegmentMethod() {
    IDISA::IDISA_Builder::InsertPoint savePoint = iBuilder->saveIP();
    Module * m = iBuilder->getModule();
    Function * writefn = create_write(m);
    Function * doSegmentFunction = m->getFunction(mKernelName + doSegment_suffix);
    Type * i8PtrTy = iBuilder->getInt8PtrTy();
    
    iBuilder->SetInsertPoint(BasicBlock::Create(iBuilder->getContext(), "entry", doSegmentFunction, 0));
    Constant * stride = ConstantInt::get(iBuilder->getSizeTy(), iBuilder->getStride());
    Constant * strideBytes = ConstantInt::get(iBuilder->getSizeTy(), iBuilder->getStride() * mCodeUnitWidth/8);
    
    Function::arg_iterator args = doSegmentFunction->arg_begin();
    Value * self = &*(args++);
    Value * blocksToDo = &*(args);
    ////iBuilder->CallPrintInt("blocksToDo", blocksToDo);
    Value * segmentNo = getLogicalSegmentNo(self);
    Value * streamStructPtr = getStreamSetStructPtr(self, "codeUnitBuffer");
    //iBuilder->CallPrintInt("streamStructPtr", iBuilder->CreatePtrToInt(streamStructPtr, iBuilder->getInt64Ty()));

    LoadInst * producerPos = iBuilder->CreateAlignedLoad(mStreamSetInputBuffers[0]->getProducerPosPtr(streamStructPtr), sizeof(size_t));
    producerPos->setOrdering(AtomicOrdering::Acquire);
    //iBuilder->CallPrintInt("producerPos", producerPos);

    Value * processed = getProcessedItemCount(self);
    Value * itemsAvail = iBuilder->CreateSub(producerPos, processed);
    //iBuilder->CallPrintInt("previously processed", processed);
    Value * blocksAvail = iBuilder->CreateUDiv(itemsAvail, stride);
    //iBuilder->CallPrintInt("blocksAvail", blocksAvail);
    /* Adjust the number of full blocks to do, based on the available data, if necessary. */
    blocksToDo = iBuilder->CreateSelect(iBuilder->CreateICmpULT(blocksToDo, blocksAvail), blocksToDo, blocksAvail);
    Value * blockNo = getScalarField(self, blockNoScalar);
    //iBuilder->CallPrintInt("blockNo", blockNo);
    Value * basePtr = getStreamSetBlockPtr(self, "codeUnitBuffer", blockNo);
    //iBuilder->CallPrintInt("basePtr", iBuilder->CreatePtrToInt(basePtr, iBuilder->getInt64Ty()));
    Value * bytesToDo = iBuilder->CreateMul(blocksToDo, strideBytes);
    //iBuilder->CallPrintInt("bytesToDo", bytesToDo);
    iBuilder->CreateCall(writefn, std::vector<Value *>({iBuilder->getInt32(1), iBuilder->CreateBitCast(basePtr, i8PtrTy), bytesToDo}));
    
    setScalarField(self, blockNoScalar, iBuilder->CreateAdd(blockNo, blocksToDo));
    processed = iBuilder->CreateAdd(processed, iBuilder->CreateMul(blocksToDo, stride));
    setProcessedItemCount(self, processed);
    mStreamSetInputBuffers[0]->setConsumerPos(streamStructPtr, processed);
    // Must be the last action, for synchronization.
    setLogicalSegmentNo(self, iBuilder->CreateAdd(segmentNo, ConstantInt::get(iBuilder->getSizeTy(), 1)));
    
    iBuilder->CreateRetVoid();
    iBuilder->restoreIP(savePoint);
}

void stdOutKernel::generateFinalBlockMethod() {
    IDISA::IDISA_Builder::InsertPoint savePoint = iBuilder->saveIP();
    Module * m = iBuilder->getModule();
    Function * writefn = create_write(m);
    Function * finalBlockFunction = m->getFunction(mKernelName + finalBlock_suffix);
    Type * i8PtrTy = iBuilder->getInt8PtrTy();
    
    iBuilder->SetInsertPoint(BasicBlock::Create(iBuilder->getContext(), "fb_flush", finalBlockFunction, 0));
    Value * self = getParameter(finalBlockFunction, "self");
    Value * streamStructPtr = getStreamSetStructPtr(self, "codeUnitBuffer");
    LoadInst * producerPos = iBuilder->CreateAlignedLoad(mStreamSetInputBuffers[0]->getProducerPosPtr(streamStructPtr), sizeof(size_t));
    producerPos->setOrdering(AtomicOrdering::Acquire);
    Value * processed = getProcessedItemCount(self);
    Value * itemsAvail = iBuilder->CreateSub(producerPos, processed);
    Value * blockNo = getScalarField(self, blockNoScalar);
    Value * basePtr = getStreamSetBlockPtr(self, "codeUnitBuffer", blockNo);
    Value * bytesToDo = iBuilder->CreateMul(itemsAvail, ConstantInt::get(iBuilder->getSizeTy(), mCodeUnitWidth/8));

    iBuilder->CreateCall(writefn, std::vector<Value *>({iBuilder->getInt32(1), iBuilder->CreateBitCast(basePtr, i8PtrTy), bytesToDo}));
    
    setProcessedItemCount(self, producerPos);
    mStreamSetInputBuffers[0]->setConsumerPos(streamStructPtr, producerPos);
    iBuilder->CreateRetVoid();
    iBuilder->restoreIP(savePoint);
}
}
