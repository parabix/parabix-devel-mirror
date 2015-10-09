#ifndef IDISA_BUILDER_H
#define IDISA_BUILDER_H

/*
 *  Copyright (c) 2015 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */
#include <llvm/IR/Module.h>
#include <llvm/IR/Constant.h>
#include <llvm/IR/Type.h>
#include <llvm/IR/Value.h>
#include <llvm/IR/IRBuilder.h>

using namespace llvm;

namespace IDISA {

class IDISA_Builder {
public:

    IDISA_Builder(Type * bitBlockType)
    : mMod(nullptr)
    , mLLVMBuilder(nullptr)
    , mBitBlockType(bitBlockType)
    , mBitBlockWidth(bitBlockType->isIntegerTy() ? cast<IntegerType>(bitBlockType)->getIntegerBitWidth() : cast<VectorType>(bitBlockType)->getBitWidth())
    , mZeroInitializer(Constant::getNullValue(bitBlockType)) 
    , mOneInitializer(Constant::getAllOnesValue(bitBlockType)) {

    }

    void initialize(Module * m, IRBuilder <> * b) {
        mMod = m;
        mLLVMBuilder = b;
    }
    
    Type * getBitBlockType() { return mBitBlockType;}
    int getBitBlockWidth() { return mBitBlockWidth;}
    Constant * allZeroes() {return mZeroInitializer;}
    Constant * allOnes() {return mOneInitializer;}
        
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
    Value * simd_popcount(unsigned fw, Value * a);
    
    Value * esimd_mergeh(unsigned fw, Value * a, Value * b);
    Value * esimd_mergel(unsigned fw, Value * a, Value * b);
    
    Value * hsimd_packh(unsigned fw, Value * a, Value * b);
    Value * hsimd_packl(unsigned fw, Value * a, Value * b);
    Value * hsimd_signmask(unsigned fw, Value * a);

    
    Value * mvmd_extract(unsigned fw, Value * a, unsigned fieldIndex);
    Value * mvmd_dslli(unsigned fw, Value * a, Value * b, unsigned shift);
    
    Value * bitblock_any(Value * a);


private:
    Module * mMod;
    IRBuilder <> * mLLVMBuilder;
    Type * mBitBlockType;
    unsigned mBitBlockWidth;
    Constant * mZeroInitializer;
    Constant * mOneInitializer;
    
    Value * bitBlockCast(Value * a);
    VectorType * fwVectorType(unsigned fw);
    Value * fwCast(unsigned fw, Value * a);
};

}
#endif // IDISA_BUILDER_H
