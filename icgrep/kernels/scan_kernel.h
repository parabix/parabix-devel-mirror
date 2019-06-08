/*
 *  Copyright (c) 2019 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 */

#ifndef SCAN_KERNEL_H
#define SCAN_KERNEL_H

#include <kernels/core/kernel.h>

namespace kernel {

// Callback must be of type: 
//          void(*)(const char * ptr, uint64_t pos)
// where `ptr` is a pointer to the chacater at the callback position in the 
// source stream and `pos` is the callback position 

class ScanKernel : public MultiBlockKernel {
public:

    ScanKernel(const std::unique_ptr<KernelBuilder> & iBuilder, StreamSet * scanStream, StreamSet * sourceStream, llvm::StringRef callbackName);

protected:
    void generateMultiBlockLogic(const std::unique_ptr<KernelBuilder> & iBuilder, llvm::Value * const numOfStrides) override;

    llvm::StringRef mCallbackName;
};

}

#endif
