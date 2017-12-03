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
#include <llvm/Support/ErrorHandling.h>
#include <kernels/kernel_builder.h>

using namespace kernel;
using namespace llvm;

struct Features {
    bool hasAVX;
    bool hasAVX2;
    Features() : hasAVX(0), hasAVX2(0) { }
};

Features getHostCPUFeatures() {
    Features hostCPUFeatures;
    StringMap<bool> features;
    if (sys::getHostCPUFeatures(features)) {
        hostCPUFeatures.hasAVX = features.count("avx");
        hostCPUFeatures.hasAVX2 = features.count("avx2");
    }
    return hostCPUFeatures;
}

bool AVX2_available() {
    StringMap<bool> features;
    if (sys::getHostCPUFeatures(features)) {
        return features.count("avx2");
    }
    return false;
}

namespace IDISA {
    
KernelBuilder * GetIDISA_Builder(llvm::LLVMContext & C) {
    const auto hostCPUFeatures = getHostCPUFeatures();
    if (LLVM_LIKELY(codegen::BlockSize == 0)) {  // No BlockSize override: use processor SIMD width
        codegen::BlockSize = hostCPUFeatures.hasAVX2 ? 256 : 128;
    }
    else if (((codegen::BlockSize & (codegen::BlockSize - 1)) != 0) || (codegen::BlockSize < 64)) {
        llvm::report_fatal_error("BlockSize must be a power of 2 and >=64");
    }
    if (codegen::BlockSize >= 128) {
        if (hostCPUFeatures.hasAVX2) {
            return new KernelBuilderImpl<IDISA_AVX2_Builder>(C, codegen::BlockSize, codegen::BlockSize);
        } else if (hostCPUFeatures.hasAVX) {
            return new KernelBuilderImpl<IDISA_AVX_Builder>(C, codegen::BlockSize, codegen::BlockSize);
        }
    } else if (codegen::BlockSize == 64) {
        return new KernelBuilderImpl<IDISA_I64_Builder>(C, codegen::BlockSize, codegen::BlockSize);
    }
    return new KernelBuilderImpl<IDISA_SSE2_Builder>(C, codegen::BlockSize, codegen::BlockSize);
}
#ifdef CUDA_ENABLED
KernelBuilder * GetIDISA_GPU_Builder(llvm::LLVMContext & C) {
    return new KernelBuilderImpl<IDISA_NVPTX20_Builder>(C, 64, 64 * 64);
}
#endif
}
