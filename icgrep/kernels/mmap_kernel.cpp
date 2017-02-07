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
            
void MMapSourceKernel::generateDoSegmentMethod(Value *doFinal, const std::vector<Value *> &producerPos) {
    BasicBlock * setTermination = CreateBasicBlock("setTermination");
    BasicBlock * mmapSourceExit = CreateBasicBlock("mmapSourceExit");
    ConstantInt * segmentItems = iBuilder->getSize(mSegmentBlocks * iBuilder->getBitBlockWidth());
    Value * fileItems = getScalarField("fileSize");
    if (mCodeUnitWidth > 8) {
        fileItems = iBuilder->CreateUDiv(fileItems, iBuilder->getSize(mCodeUnitWidth / 8));
    }
    Value * produced = getProducedItemCount("sourceBuffer");
    Value * nextProduced = iBuilder->CreateAdd(produced, segmentItems);
    Value * lessThanFullSegment = iBuilder->CreateICmpULT(fileItems, nextProduced);
    produced = iBuilder->CreateSelect(lessThanFullSegment, fileItems, nextProduced);
    setProducedItemCount("sourceBuffer", produced);

    iBuilder->CreateCondBr(lessThanFullSegment, setTermination, mmapSourceExit);
    iBuilder->SetInsertPoint(setTermination);
    setTerminationSignal();
    iBuilder->CreateBr(mmapSourceExit);    

    iBuilder->SetInsertPoint(mmapSourceExit);
}


MMapSourceKernel::MMapSourceKernel(IDISA::IDISA_Builder * iBuilder, unsigned blocksPerSegment, unsigned codeUnitWidth)
: SegmentOrientedKernel(iBuilder, "mmap_source",
    {},
    {Binding{iBuilder->getStreamSetTy(1, codeUnitWidth), "sourceBuffer", iBuilder->getBitBlockWidth()}},
    {Binding{iBuilder->getSizeTy(), "fileSize"}}, {}, {})
, mSegmentBlocks(blocksPerSegment)
, mCodeUnitWidth(codeUnitWidth) {

}

}
