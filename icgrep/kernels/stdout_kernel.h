/*
 *  Copyright (c) 2016 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 */
#ifndef STDOUT_KERNEL_H
#define STDOUT_KERNEL_H

namespace llvm { class Module; }

namespace IDISA { class IDISA_Builder; }

namespace kernel {

class KernelBuilder;

    void generateStdOutKernel(llvm::Module *, IDISA::IDISA_Builder * iBuilder, KernelBuilder * kBuilder, unsigned fw = 8);
}

#endif
