/*
 *  Copyright (c) 2016 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 */
#ifndef STDOUT_KERNEL_H
#define STDOUT_KERNEL_H

#include "kernel.h"

namespace IDISA { class IDISA_Builder; }

namespace kernel {

class StdOutKernel final : public MultiBlockKernel {
public:
    StdOutKernel(const std::unique_ptr<kernel::KernelBuilder> & iBuilder, unsigned codeUnitWidth);
private:
    void generateMultiBlockLogic(const std::unique_ptr<KernelBuilder> & iBuilder, llvm::Value * const numOfStrides) override;
private:
    const unsigned mCodeUnitWidth;
    
};

class FileSink final : public MultiBlockKernel {
public:  
    FileSink(const std::unique_ptr<kernel::KernelBuilder> & iBuilder, unsigned codeUnitWidth);
protected:
    void generateInitializeMethod(const std::unique_ptr<KernelBuilder> & iBuilder) override;
    void generateMultiBlockLogic(const std::unique_ptr<KernelBuilder> & iBuilder, llvm::Value * const numOfStrides) override;
    void generateFinalizeMethod(const std::unique_ptr<KernelBuilder> & b) override;
private:
    const unsigned mCodeUnitWidth;
    
};
}



#endif
