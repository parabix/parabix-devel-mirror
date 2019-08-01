/*
 *  Copyright (c) 2019 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 */

#include <kernel/scan/index_generator.h>

#include <kernel/core/kernel_builder.h>

using namespace llvm;

namespace kernel {

ScanIndexGenerator::ScanIndexGenerator(BuilderRef b, StreamSet * scan, StreamSet * output)
: SingleStreamScanKernelTemplate(b, "ScanIndexGenerator", scan)
{
    assert (scan->getNumElements() == 1 && scan->getFieldWidth() == 1);
    assert (output->getNumElements() == 1 && output->getFieldWidth() == 64);
    mOutputStreamSets.push_back({"output", output, PopcountOf("scan")});
}

void ScanIndexGenerator::generateProcessingLogic(
    BuilderRef b, 
    Value * const absoluteIndex, 
    Value * const blockIndex, 
    Value * const bitOffset) 
{
    Value * const producedItemCount = b->getProducedItemCount("output");
    b->setProducedItemCount("output", b->CreateAdd(producedItemCount, b->getSize(1)));
    Value * const val = b->CreateZExtOrBitCast(absoluteIndex, b->getInt64Ty());
    b->CreateStore(val, b->getRawOutputPointer("output", b->getInt32(0), producedItemCount));
}

}
