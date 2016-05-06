/*
 *  Copyright (c) 2016 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 */

#include <toolchain.h>
#include <IDISA/idisa_avx_builder.h>
#include <IDISA/idisa_sse_builder.h>
#include <IDISA/idisa_i64_builder.h>


// Dynamic processor detection
//#define ISPC_LLVM_VERSION ISPC_LLVM_3_6
#include <util/ispc.h>

namespace IDISA {
    


IDISA_Builder * GetIDISA_Builder(Module * mod) {
    bool hasAVX2 = (strncmp(lGetSystemISA(), "avx2", 4) == 0);
    
    unsigned theBlockSize = codegen::BlockSize;  // from command line
    
    if (theBlockSize == 0) {  // No BlockSize override: use processor SIMD width
        theBlockSize = hasAVX2 ? 256 : 128;
    }
    Type * bitBlockType = VectorType::get(IntegerType::get(getGlobalContext(), 64), theBlockSize/64);
    
    int blockSize = bitBlockType->isIntegerTy() ? cast<IntegerType>(bitBlockType)->getIntegerBitWidth() : cast<VectorType>(bitBlockType)->getBitWidth();
    if (blockSize >= 256) {
        if (hasAVX2) {
            return new IDISA_AVX2_Builder(mod, bitBlockType);
        }
        else{
            return new IDISA_SSE2_Builder(mod, bitBlockType);
        }
    }
    else if (blockSize == 64)
        return new IDISA_I64_Builder(mod, bitBlockType);
    return new IDISA_SSE2_Builder(mod, bitBlockType);
}

}