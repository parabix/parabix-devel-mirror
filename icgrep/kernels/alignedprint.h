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

class PrintableBits : public BlockOrientedKernel {
public:
    PrintableBits(const std::unique_ptr<kernel::KernelBuilder> & builder);
    virtual ~PrintableBits() {}
private:
    void generateDoBlockMethod() override;
};

class SelectStream : public BlockOrientedKernel {
public:
    SelectStream(const std::unique_ptr<kernel::KernelBuilder> & builder, unsigned sizeInputStreamSet, unsigned streamIndex);
    virtual ~SelectStream() {}
private:
    void generateDoBlockMethod() override;
    unsigned mSizeInputStreamSet;
    unsigned mStreamIndex;
};

class PrintStreamSet : public BlockOrientedKernel {
public:
    PrintStreamSet(const std::unique_ptr<kernel::KernelBuilder> & builder, std::vector<std::string> && names, const unsigned minWidth = 16);
    virtual ~PrintStreamSet() {}
private:
    void generateDoBlockMethod() override;
private:
    const std::vector<std::string> mNames;
    unsigned mNameWidth;
};

}
#endif
