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
    PrintableBits(IDISA::IDISA_Builder * builder);
    virtual ~PrintableBits() {}
private:
    void generateDoBlockMethod() override;
};

class SelectStream : public BlockOrientedKernel {
public:
    SelectStream(IDISA::IDISA_Builder * builder, unsigned sizeInputStreamSet, unsigned streamIndex);
    virtual ~SelectStream() {}
private:
    void generateDoBlockMethod() override;
    unsigned mSizeInputStreamSet;
    unsigned mStreamIndex;
};

class PrintableStreamSet : public BlockOrientedKernel {
public:
    PrintableStreamSet(IDISA::IDISA_Builder * builder);
    virtual ~PrintableStreamSet() {}
private:
    void generateDoBlockMethod() override;
};

}
#endif
