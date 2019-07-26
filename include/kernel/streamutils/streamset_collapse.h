/*
 *  Copyright (c) 2019 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 */

#ifndef STREAMSET_COLLAPSE_H
#define STREAMSET_COLLAPSE_H

#include <kernel/core/kernel.h>

namespace kernel {

/**
 * A simple kernel which collapses a set of N streams into a single stream.
 * 
 * kernel CollapseStreamSet :: [<i1>[N] input] -> [<i1>[1] output]
 */
class CollapseStreamSet : public BlockOrientedKernel {
public:
    CollapseStreamSet(const std::unique_ptr<KernelBuilder> & iBuilder, StreamSet * input, StreamSet * output);

    bool isCachable() const override { return true; }
    bool hasSignature() const override { return false; }
protected:
    void generateDoBlockMethod(const std::unique_ptr<KernelBuilder> & iBuilder) override;
};

}

#endif
