/*
 *  Copyright (c) 2019 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 */

#ifndef STREAMSET_COLLAPSE_H
#define STREAMSET_COLLAPSE_H

#include <kernel/core/kernel.h>
#include <kernel/pipeline/pipeline_builder.h>

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

namespace streamutils {

    /**
     * Merges the contents of `i` into a single bitstream.
     * 
     * Streamset `i` may contain an arbitrary number of streams but must have
     * a field width of 1 (i.e., <i1>[N]).
     * 
     * The operation is equivalent to `or`ing together the contents of `i`.
     * 
     * Example:
     *      i[0]: 1...1...
     *      i[1]: 1.1.....
     * 
     *      out:  1.1.1...
     * 
     * This function is a wrapper for a kernel call to `CollapseStreamSet`.
     */
    StreamSet * Collapse(std::unique_ptr<ProgramBuilder> & P, StreamSet * i);

}

}

#endif
