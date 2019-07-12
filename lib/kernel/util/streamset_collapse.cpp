/*
 *  Copyright (c) 2019 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 */

#include <kernel/util/streamset_collapse.h>

#include <kernel/core/kernel_builder.h>

using namespace llvm;

namespace kernel {

void CollapseStreamSet::generateDoBlockMethod(const std::unique_ptr<KernelBuilder> & b) {
    const size_t n = getInputStreamSet("input")->getNumElements();
    Value * accum = b->loadInputStreamBlock("input", b->getInt32(0));
    for (size_t i = 1; i < n; ++i) {
        accum = b->CreateOr(accum, b->loadInputStreamBlock("input", b->getInt32(i)));
    }
    b->storeOutputStreamBlock("output", b->getInt32(0), accum);
}

CollapseStreamSet::CollapseStreamSet(const std::unique_ptr<KernelBuilder> & b, StreamSet * input, StreamSet * output)
: BlockOrientedKernel(b, "CollapseStreamSet_x" + std::to_string(input->getNumElements()), {{"input", input}}, {{"output", output}}, {}, {}, {})
{}

}
