/*
 *  Copyright (c) 2017 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 */
#ifndef STREAMSMERGE_H
#define STREAMSMERGE_H

#include "kernel.h"
#include <vector>

namespace IDISA { class IDISA_Builder; }

namespace kernel {

class StreamsMerge : public BlockOrientedKernel {
public:
    
    StreamsMerge(const std::unique_ptr<kernel::KernelBuilder> & iBuilder, unsigned streamsPerSet=1, unsigned inputSets = 1);
    
protected:
    
    void generateDoBlockMethod(const std::unique_ptr<kernel::KernelBuilder> & iBuilder) override;
    
private:
    const unsigned mStreamsPerSet;
    const unsigned mInputSets;
};

class StreamsCombineKernel : public BlockOrientedKernel {
public:
    StreamsCombineKernel(const std::unique_ptr<kernel::KernelBuilder> & iBuilder, std::vector<unsigned> streamsNumOfSets);
protected:
    void generateDoBlockMethod(const std::unique_ptr<kernel::KernelBuilder> & iBuilder) override;
private:
    const std::vector<unsigned> mStreamsNumOfSets;
};

class StreamsSplitKernel : public BlockOrientedKernel {
public:
    StreamsSplitKernel(const std::unique_ptr<kernel::KernelBuilder> & iBuilder, std::vector<unsigned> streamsNumOfSets);
protected:
    void generateDoBlockMethod(const std::unique_ptr<kernel::KernelBuilder> & iBuilder) override;
private:
    const std::vector<unsigned> mStreamsNumOfSets;
};

class StreamsIntersect : public BlockOrientedKernel {
public:
    
    StreamsIntersect(const std::unique_ptr<kernel::KernelBuilder> & iBuilder, unsigned streamsPerSet=1, unsigned inputSets = 1);
    
protected:
    
    void generateDoBlockMethod(const std::unique_ptr<kernel::KernelBuilder> & iBuilder) override;
    
private:
    const unsigned mStreamsPerSet;
    const unsigned mInputSets;
};

}
    
#endif

