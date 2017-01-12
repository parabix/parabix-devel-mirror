/*
 *  Copyright (c) 2017 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 */
#include <kernels/mmap_kernel.h>
#include <IR_Gen/idisa_builder.h>

namespace kernel {
            
void MMapSourceKernel::generateDoSegmentMethod() const {
    auto savePoint = iBuilder->saveIP();
    Module * m = iBuilder->getModule();
    Function * doSegmentFunction = m->getFunction(mKernelName + doSegment_suffix);
    
    iBuilder->SetInsertPoint(BasicBlock::Create(iBuilder->getContext(), mKernelName + "_entry", doSegmentFunction, 0));
    BasicBlock * setTermination = BasicBlock::Create(iBuilder->getContext(), "setTermination", doSegmentFunction, 0);
    BasicBlock * mmapSourceExit = BasicBlock::Create(iBuilder->getContext(), "mmapSourceExit", doSegmentFunction, 0);
    Constant * segmentItems = iBuilder->getSize(mSegmentBlocks * iBuilder->getBitBlockWidth());
    
    
    Function::arg_iterator args = doSegmentFunction->arg_begin();
    Value * self = &*(args);
    
    Value * fileItems = getScalarField(self, "fileSize");
    if (mCodeUnitWidth > 8) {
        fileItems = iBuilder->CreateUDiv(fileItems, iBuilder->getSize(mCodeUnitWidth/8));
    }
    Value * produced = getProducedItemCount(self, "sourceBuffer");
    Value * itemsAvail = iBuilder->CreateSub(fileItems, produced);
    Value * lessThanFullSegment = iBuilder->CreateICmpULT(itemsAvail, segmentItems);
    Value * itemsToDo = iBuilder->CreateSelect(lessThanFullSegment, itemsAvail, segmentItems);
    setProducedItemCount(self, "sourceBuffer", iBuilder->CreateAdd(produced, itemsToDo));
    
    iBuilder->CreateCondBr(lessThanFullSegment, setTermination, mmapSourceExit);
    iBuilder->SetInsertPoint(setTermination);
    setTerminationSignal(self);
    iBuilder->CreateBr(mmapSourceExit);
    iBuilder->SetInsertPoint(mmapSourceExit);
    iBuilder->CreateRetVoid();
    iBuilder->restoreIP(savePoint);
}

// The doBlock method is deprecated.   But in case it is used, just call doSegment with
// 1 as the number of blocks to do.
void MMapSourceKernel::generateDoBlockMethod() const {
    auto savePoint = iBuilder->saveIP();
    Module * m = iBuilder->getModule();
    Function * doBlockFunction = m->getFunction(mKernelName + doBlock_suffix);
    Function * doSegmentFunction = m->getFunction(mKernelName + doSegment_suffix);
    iBuilder->SetInsertPoint(BasicBlock::Create(iBuilder->getContext(), "entry", doBlockFunction, 0));
    Value * self = getParameter(doBlockFunction, "self");
    iBuilder->CreateCall(doSegmentFunction, {self, iBuilder->getSize(1)});
    iBuilder->CreateRetVoid();
    iBuilder->restoreIP(savePoint);
}
    
}
