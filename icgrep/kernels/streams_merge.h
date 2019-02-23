/*
 *  Copyright (c) 2017 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 */
#ifndef STREAMSMERGE_H
#define STREAMSMERGE_H

#include <kernels/core/kernel.h>
#include <vector>

namespace IDISA { class IDISA_Builder; }

namespace kernel {

class StreamsMerge : public BlockOrientedKernel {
public:
    StreamsMerge(const std::unique_ptr<kernel::KernelBuilder> & iBuilder, const std::vector<StreamSet *> & inputs, StreamSet * output);
protected:
    void generateDoBlockMethod(const std::unique_ptr<kernel::KernelBuilder> & iBuilder) override;
};

class StreamsCombineKernel : public BlockOrientedKernel {
public:
    StreamsCombineKernel(const std::unique_ptr<kernel::KernelBuilder> & b, std::vector<unsigned> streamsNumOfSets);
protected:
    void generateDoBlockMethod(const std::unique_ptr<kernel::KernelBuilder> & iBuilder) override;
private:
    const std::vector<unsigned> mStreamsNumOfSets;
};

class StreamsSplitKernel : public BlockOrientedKernel {
public:
    StreamsSplitKernel(const std::unique_ptr<kernel::KernelBuilder> & b, std::vector<unsigned> streamsNumOfSets);
protected:
    void generateDoBlockMethod(const std::unique_ptr<kernel::KernelBuilder> & iBuilder) override;
private:
    const std::vector<unsigned> mStreamsNumOfSets;
};

class StreamsIntersect : public BlockOrientedKernel {
public:
    StreamsIntersect(const std::unique_ptr<kernel::KernelBuilder> & iBuilder, const std::vector<StreamSet *> & inputs, StreamSet * output);
protected:
    void generateDoBlockMethod(const std::unique_ptr<kernel::KernelBuilder> & iBuilder) override;
};

}

#endif

