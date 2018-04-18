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
#include <llvm/Support/raw_ostream.h>
#include <kernels/kernel_builder.h>

using namespace kernel;
using namespace llvm;

struct Features {
    bool hasAVX;
    bool hasAVX2;
    bool hasAVX512F;
    Features() : hasAVX(0), hasAVX2(0), hasAVX512F(0) { }
};

Features getHostCPUFeatures() {
    Features hostCPUFeatures;
    StringMap<bool> features;
    if (sys::getHostCPUFeatures(features)) {
        hostCPUFeatures.hasAVX = features.lookup("avx");
        hostCPUFeatures.hasAVX2 = features.lookup("avx2");
        hostCPUFeatures.hasAVX512F = features.lookup("avx512f");
    }
    return hostCPUFeatures;
}

bool AVX2_available() {
    StringMap<bool> features;
    if (sys::getHostCPUFeatures(features)) {
        return features.lookup("avx2");
    }
    return false;
}

bool AVX512BW_available() {
    StringMap<bool> features;
    if (sys::getHostCPUFeatures(features)) {
        return features.lookup("avx512bw");
    }
    return false;
}

namespace IDISA {

KernelBuilder * GetIDISA_Builder(llvm::LLVMContext & C) {
    const auto hostCPUFeatures = getHostCPUFeatures();
    if (LLVM_LIKELY(codegen::BlockSize == 0)) {  // No BlockSize override: use processor SIMD width

        if (hostCPUFeatures.hasAVX512F) codegen::BlockSize = 512;
        else if (hostCPUFeatures.hasAVX2) codegen::BlockSize = 256;
        else codegen::BlockSize = 128;
    }
    else if (((codegen::BlockSize & (codegen::BlockSize - 1)) != 0) || (codegen::BlockSize < 64)) {
        llvm::report_fatal_error("BlockSize must be a power of 2 and >=64");
    }
    if (codegen::BlockSize >= 512) {
        // AVX512BW builder can only be used for BlockSize multiples of 512
        if (hostCPUFeatures.hasAVX512F) {
            return new KernelBuilderImpl<IDISA_AVX512F_Builder>(C, codegen::BlockSize, codegen::BlockSize);
        }
    }
    if (codegen::BlockSize >= 256) {
        // AVX2 or AVX builders can only be used for BlockSize multiples of 256
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
