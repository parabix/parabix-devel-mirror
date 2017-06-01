/*
 *  Copyright (c) 2016 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 */

#ifndef IDISA_TARGET_H
#define IDISA_TARGET_H

namespace llvm { class LLVMContext; }
namespace kernel { class KernelBuilder; }
#include <string>

namespace IDISA {
    
kernel::KernelBuilder * GetIDISA_Builder(llvm::LLVMContext & C);

kernel::KernelBuilder * GetIDISA_GPU_Builder(llvm::LLVMContext & C);

}

#endif
