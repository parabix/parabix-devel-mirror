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

    BasicBlock * entryBlock = iBuilder->GetInsertBlock();
    BasicBlock * setTermination = CreateBasicBlock("setTermination");
    BasicBlock * mmapSourceExit = CreateBasicBlock("mmapSourceExit");
    ConstantInt * segmentItems = iBuilder->getSize(mSegmentBlocks * iBuilder->getBitBlockWidth());
    Value * fileItems = getScalarField("fileSize");
    if (mCodeUnitWidth > 8) {
        fileItems = iBuilder->CreateUDiv(fileItems, iBuilder->getSize(mCodeUnitWidth / 8));
    }
    Value * produced = getProducedItemCount("sourceBuffer");
    produced = iBuilder->CreateAdd(produced, segmentItems);
    Value * lessThanFullSegment = iBuilder->CreateICmpULT(fileItems, produced);
    iBuilder->CreateCondBr(lessThanFullSegment, setTermination, mmapSourceExit);
    iBuilder->SetInsertPoint(setTermination);
    setTerminationSignal();
    iBuilder->CreateBr(mmapSourceExit);

    iBuilder->SetInsertPoint(mmapSourceExit);

    PHINode * itemsRead = iBuilder->CreatePHI(produced->getType(), 2);
    itemsRead->addIncoming(produced, entryBlock);
    itemsRead->addIncoming(fileItems, setTermination);
    setProducedItemCount("sourceBuffer", itemsRead);
}

void MMapSourceKernel::generateInitMethod() {
//    Value * fileSize = getScalarField("fileSize");
//    fileSize = iBuilder->CreateUDiv(fileSize, iBuilder->getSize(mCodeUnitWidth / 8));
//    setProducedItemCount("sourceBuffer", fileSize);
}

MMapSourceKernel::MMapSourceKernel(IDISA::IDISA_Builder * iBuilder, unsigned blocksPerSegment, unsigned codeUnitWidth)
: SegmentOrientedKernel(iBuilder, "mmap_source",
    {},
    {Binding{iBuilder->getStreamSetTy(1, codeUnitWidth), "sourceBuffer"}},
    {Binding{iBuilder->getSizeTy(), "fileSize"}}, {}, {})
, mSegmentBlocks(blocksPerSegment)
, mCodeUnitWidth(codeUnitWidth) {

}

}
