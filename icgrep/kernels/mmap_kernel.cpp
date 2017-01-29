/*
 *  Copyright (c) 2017 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 */
#include "mmap_kernel.h"
#include <llvm/IR/Function.h>  // for Function, Function::arg_iterator
#include <llvm/IR/Module.h>
#include <IR_Gen/idisa_builder.h>
#include <kernels/streamset.h>
namespace llvm { class BasicBlock; }
namespace llvm { class Constant; }
namespace llvm { class Module; }
namespace llvm { class Value; }

using namespace llvm;

namespace kernel {
            
void MMapSourceKernel::generateDoSegmentMethod(Function * doSegmentFunction, Value *self, Value *doFinal, const std::vector<Value *> &producerPos) const {
    BasicBlock * setTermination = BasicBlock::Create(iBuilder->getContext(), "setTermination", doSegmentFunction, 0);
    BasicBlock * mmapSourceExit = BasicBlock::Create(iBuilder->getContext(), "mmapSourceExit", doSegmentFunction, 0);
    ConstantInt * segmentItems = iBuilder->getSize(mSegmentBlocks * iBuilder->getBitBlockWidth());
    Value * fileItems = getScalarField(self, "fileSize");
    if (mCodeUnitWidth > 8) {
        fileItems = iBuilder->CreateUDiv(fileItems, iBuilder->getSize(mCodeUnitWidth / 8));
    }
    Value * produced = getProducedItemCount(self, "sourceBuffer");    
    Value * nextProduced = iBuilder->CreateAdd(produced, segmentItems);
    Value * lessThanFullSegment = iBuilder->CreateICmpULT(fileItems, nextProduced);
    produced = iBuilder->CreateSelect(lessThanFullSegment, fileItems, nextProduced);
    setProducedItemCount(self, "sourceBuffer", produced);

    iBuilder->CreateCondBr(lessThanFullSegment, setTermination, mmapSourceExit);
    iBuilder->SetInsertPoint(setTermination);
    setTerminationSignal(self);
    iBuilder->CreateBr(mmapSourceExit);    

    iBuilder->SetInsertPoint(mmapSourceExit);
}


MMapSourceKernel::MMapSourceKernel(IDISA::IDISA_Builder * iBuilder, unsigned blocksPerSegment, unsigned codeUnitWidth)
: SegmentOrientedKernel(iBuilder, "mmap_source", {}, {Binding{iBuilder->getStreamSetTy(1, codeUnitWidth), "sourceBuffer"}}, {Binding{iBuilder->getSizeTy(), "fileSize"}}, {}, {})
, mSegmentBlocks(blocksPerSegment)
, mCodeUnitWidth(codeUnitWidth) {

}

}
