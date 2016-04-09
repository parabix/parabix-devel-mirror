/*
 *  Copyright (c) 2016 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 */
#ifndef P2S_KERNEL_H
#define P2S_KERNEL_H

namespace llvm { class Module; }

namespace IDISA { class IDISA_Builder; }

namespace kernel {

class KernelBuilder;

    void generateP2SKernel(llvm::Module *, IDISA::IDISA_Builder * iBuilder, KernelBuilder * kBuilder);

    void generateP2S_16Kernel(llvm::Module *, IDISA::IDISA_Builder * iBuilder, KernelBuilder * kBuilder);

    void generateP2S_16_withCompressedOutputKernel(llvm::Module *, IDISA::IDISA_Builder * iBuilder, KernelBuilder * kBuilder);

}

#endif
