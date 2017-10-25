/*
 *  Copyright (c) 2017 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 */
#ifndef UNTIL_N_H
#define UNTIL_N_H

#include "kernel.h"  // for KernelBuilder
namespace IDISA { class IDISA_Builder; } 

namespace kernel {

class UntilNkernel : public MultiBlockKernel {
public:
    UntilNkernel(const std::unique_ptr<kernel::KernelBuilder> & iBuilder);
private:
    void generateMultiBlockLogic(const std::unique_ptr<KernelBuilder> & iBuilder, llvm::Value * const numOfStrides) override;

};

}
#endif
