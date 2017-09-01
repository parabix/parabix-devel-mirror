/*
 *  Copyright (c) 2017 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 */
#ifndef ALIGNED_PRINT_H
#define ALIGNED_PRINT_H

#include "kernel.h"  // for KernelBuilder
namespace IDISA { class IDISA_Builder; }  // lines 14-14
namespace llvm { class Value; }

namespace kernel {

class PrintableBits final : public BlockOrientedKernel {
public:
    PrintableBits(const std::unique_ptr<kernel::KernelBuilder> & builder);
private:
    void generateDoBlockMethod(const std::unique_ptr<kernel::KernelBuilder> & iBuilder) override;
};

class SelectStream final : public BlockOrientedKernel {
public:
    SelectStream(const std::unique_ptr<kernel::KernelBuilder> & builder, unsigned sizeInputStreamSet, unsigned streamIndex);
private:
    void generateDoBlockMethod(const std::unique_ptr<kernel::KernelBuilder> & iBuilder) override;
    unsigned mSizeInputStreamSet;
    unsigned mStreamIndex;
};

class ExpandOrSelectStreams final : public BlockOrientedKernel {
public:
    ExpandOrSelectStreams(const std::unique_ptr<kernel::KernelBuilder> & builder, unsigned sizeInputStreamSet, unsigned sizeOutputStreamSet);
private:
    void generateDoBlockMethod(const std::unique_ptr<kernel::KernelBuilder> & iBuilder) override;
    unsigned mSizeInputStreamSet;
    unsigned mSizeOutputStreamSet;
};

class PrintStreamSet final : public BlockOrientedKernel {
public:
    PrintStreamSet(const std::unique_ptr<kernel::KernelBuilder> & builder, std::vector<std::string> && names, const unsigned minWidth = 16);
private:
    void generateDoBlockMethod(const std::unique_ptr<kernel::KernelBuilder> & iBuilder) override;
private:
    const std::vector<std::string> mNames;
    unsigned mNameWidth;
};

}
#endif
