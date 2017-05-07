/*
 *  Copyright (c) 2016 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 */

#include "idisa_target.h"
#include <toolchain/toolchain.h>
#include <IR_Gen/idisa_sse_builder.h>
#include <IR_Gen/idisa_avx_builder.h>
#include <IR_Gen/idisa_i64_builder.h>
#include <IR_Gen/idisa_nvptx_builder.h>
#include <llvm/IR/Module.h>
#include <llvm/ADT/Triple.h>

#include <kernels/kernel_builder.h>

using namespace kernel;

namespace IDISA {
    
KernelBuilder * GetIDISA_Builder(llvm::Module * const module) {
    unsigned registerWidth = 0;
    Triple T(module->getTargetTriple());
    if (T.isArch64Bit()) {
        registerWidth = 64;
    } else if (T.isArch32Bit()) {
        registerWidth = 32;
    } else if (T.isArch16Bit()) {
        registerWidth = 16;
    }
    const bool hasAVX2 = AVX2_available();
    if (LLVM_LIKELY(codegen::BlockSize == 0)) {  // No BlockSize override: use processor SIMD width
        codegen::BlockSize = hasAVX2 ? 256 : 128;
    }
    if (codegen::BlockSize >= 256) {
        if (hasAVX2) {
            return new KernelBuilderImpl<IDISA_AVX2_Builder>(module->getContext(), registerWidth, codegen::BlockSize, codegen::BlockSize);
        }
    } else if (codegen::BlockSize == 64) {
        return new KernelBuilderImpl<IDISA_I64_Builder>(module->getContext(), registerWidth, codegen::BlockSize, codegen::BlockSize);
    }
    return new KernelBuilderImpl<IDISA_SSE2_Builder>(module->getContext(), registerWidth, codegen::BlockSize, codegen::BlockSize);
}

KernelBuilder * GetIDISA_GPU_Builder(llvm::Module * const module) {
    return new KernelBuilderImpl<IDISA_NVPTX20_Builder>(module->getContext(), 64, 64, 64);
}

}
