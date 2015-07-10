#ifndef IDISA_BUILDER_H
#define IDISA_BUILDER_H

/*
 *  Copyright (c) 2015 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */
#include <llvm/IR/Module.h>
#include <llvm/IR/Type.h>
#include <llvm/IR/Value.h>
#include <llvm/IR/IRBuilder.h>

using namespace llvm;

namespace IDISA {

class IDISA_Builder {
public:
    IDISA_Builder(Module * m, IRBuilder <> * b, Type * bitBlockType): mMod(m), llvm_builder(b), mBitBlockType(bitBlockType) {
        if (bitBlockType->isIntegerTy()) mBitBlockSize = dyn_cast<IntegerType>(bitBlockType)-> getIntegerBitWidth();
        else mBitBlockSize = dyn_cast<VectorType>(bitBlockType)-> getBitWidth();
    }
        
    Value * simd_add(unsigned fw, Value * a, Value * b);
    Value * simd_sub(unsigned fw, Value * a, Value * b);
    Value * simd_mult(unsigned fw, Value * a, Value * b);
    Value * simd_eq(unsigned fw, Value * a, Value * b);
    Value * simd_gt(unsigned fw, Value * a, Value * b);
    Value * simd_ugt(unsigned fw, Value * a, Value * b);
    Value * simd_lt(unsigned fw, Value * a, Value * b);
    Value * simd_ult(unsigned fw, Value * a, Value * b);
    Value * simd_max(unsigned fw, Value * a, Value * b);
    Value * simd_umax(unsigned fw, Value * a, Value * b);
    Value * simd_min(unsigned fw, Value * a, Value * b);
    Value * simd_umin(unsigned fw, Value * a, Value * b);
    
    Value * simd_slli(unsigned fw, Value * a, unsigned shift);
    Value * simd_srli(unsigned fw, Value * a, unsigned shift);
    Value * simd_srai(unsigned fw, Value * a, unsigned shift);
    
    Value * simd_cttz(unsigned fw, Value * a);
    
    Value * esimd_mergeh(unsigned fw, Value * a, Value * b);
    Value * esimd_mergel(unsigned fw, Value * a, Value * b);
    
    Value * hsimd_packh(unsigned fw, Value * a, Value * b);
    Value * hsimd_packl(unsigned fw, Value * a, Value * b);
    Value * hsimd_signmask(unsigned fw, Value * a);

private:
    Module * mMod;
    IRBuilder <> * llvm_builder;
    Type * mBitBlockType;
    unsigned mBitBlockSize;
    
    Value * bitBlockCast(Value * a);
    VectorType * fwVectorType(unsigned fw);
    Value * fwCast(unsigned fw, Value * a);
};

}
#endif // IDISA_BUILDER_H
