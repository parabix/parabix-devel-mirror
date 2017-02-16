/*
 *  Copyright (c) 2016 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 */

#include "idisa_target.h"
#include <toolchain.h>
#include <IR_Gen/idisa_avx_builder.h>
#include <IR_Gen/idisa_sse_builder.h>
#include <IR_Gen/idisa_i64_builder.h>
#include <IR_Gen/idisa_nvptx_builder.h>
#include <llvm/IR/Module.h>
#ifndef NDEBUG
#include <llvm/ADT/Triple.h>
#endif

namespace IDISA {
    
IDISA_Builder * GetIDISA_Builder(llvm::Module * mod) {
    if (LLVM_UNLIKELY(mod == nullptr)) {
        report_fatal_error("GetIDISA_Builder: module cannot be null");
    }
    if (LLVM_LIKELY(mod->getTargetTriple().empty())) {
        mod->setTargetTriple(llvm::sys::getProcessTriple());
    }
    const bool hasAVX2 = AVX2_available();
    DataLayout DL(mod);    
    Type * const intTy = DL.getIntPtrType(mod->getContext());
    const auto registerWidth = intTy->getIntegerBitWidth();
    #ifndef NDEBUG
    Triple T(mod->getTargetTriple());
    if (LLVM_UNLIKELY((T.isArch16Bit() && registerWidth != 16) || (T.isArch32Bit() && registerWidth != 32) || (T.isArch64Bit() && registerWidth != 64))) {
        report_fatal_error("GetIDISA_Builder: target triple '" + mod->getTargetTriple() + "' register width does not match data layout (" + std::to_string(registerWidth) + ")");
    }
    #endif
    if (LLVM_LIKELY(codegen::BlockSize == 0)) {  // No BlockSize override: use processor SIMD width
        codegen::BlockSize = hasAVX2 ? 256 : 128;
    }
    if (codegen::BlockSize >= 256) {
        if (hasAVX2) {
            return new IDISA_AVX2_Builder(mod, registerWidth, codegen::BlockSize);
        }
    } else if (codegen::BlockSize == 64) {
        return new IDISA_I64_Builder(mod, registerWidth);
    }
    return new IDISA_SSE2_Builder(mod, registerWidth, codegen::BlockSize);
}

IDISA_Builder * GetIDISA_GPU_Builder(llvm::Module * mod) {
    return new IDISA_NVPTX20_Builder(mod, 64);
}

}
