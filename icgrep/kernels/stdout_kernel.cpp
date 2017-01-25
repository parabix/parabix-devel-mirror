/*
 *  Copyright (c) 2016 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 */
#include "stdout_kernel.h"
#include <llvm/IR/Module.h>
#include <IR_Gen/idisa_builder.h>
#include <kernels/streamset.h>
// #include <llvm/IR/Type.h>
namespace llvm { class Type; }

using namespace llvm;

namespace kernel {
            
// Rather than using doBlock logic to write one block at a time, this custom
// doSegment method, writes the entire segment with a single write call.
void StdOutKernel::generateDoSegmentMethod() const {
    auto savePoint = iBuilder->saveIP();
    Module * m = iBuilder->getModule();
    Function * doSegmentFunction = m->getFunction(mKernelName + doSegment_suffix);
    Type * i8PtrTy = iBuilder->getInt8PtrTy();
    
    iBuilder->SetInsertPoint(BasicBlock::Create(iBuilder->getContext(), "entry", doSegmentFunction, 0));
    Constant * blockItems = iBuilder->getSize(iBuilder->getBitBlockWidth());
    Constant * itemBytes = iBuilder->getSize(mCodeUnitWidth/8);
    
    Function::arg_iterator args = doSegmentFunction->arg_begin();
    Value * self = &*(args++);
    /* unused Value * doFinal = &*(args++);*/ args++;
    Value * producerPos = &*(args++);
    Value * processed = getProcessedItemCount(self, "codeUnitBuffer");
    Value * itemsToDo = iBuilder->CreateSub(producerPos, processed);
    
    Value * blockNo = getScalarField(self, blockNoScalar);
    Value * byteOffset = iBuilder->CreateMul(iBuilder->CreateURem(processed, blockItems), itemBytes);
    Value * bytePtr = getStreamView(i8PtrTy, self, "codeUnitBuffer", blockNo, byteOffset);
    iBuilder->CreateWriteCall(iBuilder->getInt32(1), bytePtr, iBuilder->CreateMul(itemsToDo, itemBytes));

    processed = iBuilder->CreateAdd(processed, itemsToDo);
    setProcessedItemCount(self, "codeUnitBuffer", processed);
    setScalarField(self, blockNoScalar, iBuilder->CreateUDiv(processed, blockItems));

    iBuilder->CreateRetVoid();
    iBuilder->restoreIP(savePoint);
}

StdOutKernel::StdOutKernel(IDISA::IDISA_Builder * iBuilder, unsigned codeUnitWidth)
: KernelBuilder(iBuilder, "stdout", {Binding{iBuilder->getStreamSetTy(1, codeUnitWidth), "codeUnitBuffer"}}, {}, {}, {}, {})
, mCodeUnitWidth(codeUnitWidth) {
    setNoTerminateAttribute(true);
}

}
