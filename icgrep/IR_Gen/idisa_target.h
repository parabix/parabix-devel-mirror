/*
 *  Copyright (c) 2016 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 */

#ifndef IDISA_TARGET_H
#define IDISA_TARGET_H

#include <llvm/Support/Compiler.h>

namespace llvm { class LLVMContext; }
namespace kernel { class KernelBuilder; }

extern LLVM_READNONE bool AVX2_available();
extern LLVM_READNONE bool AVX512BW_available();

namespace IDISA {
    
kernel::KernelBuilder * GetIDISA_Builder(llvm::LLVMContext & C);

#ifdef CUDA_ENABLED
kernel::KernelBuilder * GetIDISA_GPU_Builder(llvm::LLVMContext & C);
#endif
}

#endif
