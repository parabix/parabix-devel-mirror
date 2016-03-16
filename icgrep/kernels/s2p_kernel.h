/*
 *  Copyright (c) 2016 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 */
#ifndef S2P_KERNEL_H
#define S2P_KERNEL_H

namespace llvm { class Module; }

namespace IDISA { class IDISA_Builder; }

namespace kernel {

class KernelBuilder;

    void generateS2PKernel(llvm::Module *, IDISA::IDISA_Builder * iBuilder, KernelBuilder * kBuilder);
    void generateS2P_idealKernel(llvm::Module *, IDISA::IDISA_Builder * iBuilder, KernelBuilder * kBuilder);

}

#endif
