/*
 *  Copyright (c) 2017 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 */
#ifndef EVEN_ODD_H
#define EVEN_ODD_H

#include "kernel.h"  // for KernelBuilder
namespace IDISA { class IDISA_Builder; }  // lines 14-14
namespace llvm { class Value; }

namespace kernel {

class EvenOddKernel final : public BlockOrientedKernel {
public:
    EvenOddKernel(const std::unique_ptr<kernel::KernelBuilder> & builder);
private:
    void generateDoBlockMethod(const std::unique_ptr<kernel::KernelBuilder> & iBuilder) override;
};

}
#endif
