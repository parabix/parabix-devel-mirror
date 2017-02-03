#ifndef IDISA_BUILDER_H
#define IDISA_BUILDER_H

/*
 *  Copyright (c) 2015 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */
#include "CBuilder.h"
#include <llvm/IR/DerivedTypes.h>
namespace llvm { class Constant; }
namespace llvm { class LoadInst; }
namespace llvm { class Module; }
namespace llvm { class Value; }

namespace IDISA {

class IDISA_Builder : public CBuilder {

public:

    IDISA_Builder(llvm::Module * m, unsigned archBitWidth, unsigned bitBlockWidth, unsigned stride, unsigned CacheAlignment=64);

    virtual ~IDISA_Builder();
    
    std::string getBitBlockTypeName() const;  // A short string such as v4i64 or i256.

    llvm::Value * bitCast(llvm::Value * a) {
        return (a->getType() == mBitBlockType) ? a : CreateBitCast(a, mBitBlockType);
    }

    unsigned getBitBlockWidth() const {
        return mBitBlockWidth;
    }

    unsigned getStride() const {
        return mStride;
    }

    llvm::Constant * allZeroes() const {
        return mZeroInitializer;
    }

    llvm::Constant * allOnes() const {
        return mOneInitializer;
    }
    

    llvm::LoadInst * CreateBlockAlignedLoad(llvm::Value * const ptr);
    llvm::LoadInst * CreateBlockAlignedLoad(llvm::Value * const ptr, llvm::Value * const index);
    llvm::LoadInst * CreateBlockAlignedLoad(llvm::Value * const ptr, std::initializer_list<llvm::Value *> indices);

    void CreateBlockAlignedStore(llvm::Value * const value, llvm::Value * const ptr);
    void CreateBlockAlignedStore(llvm::Value * const value, llvm::Value * const ptr, llvm::Value * const index);
    void CreateBlockAlignedStore(llvm::Value * const value, llvm::Value * const ptr, std::initializer_list<llvm::Value *> indices);

    llvm::VectorType * fwVectorType(unsigned fw);

    llvm::Constant * simd_himask(unsigned fw);
    llvm::Constant * simd_lomask(unsigned fw);
    
    virtual llvm::Value * simd_fill(unsigned fw, llvm::Value * a);

    virtual llvm::Value * simd_add(unsigned fw, llvm::Value * a, llvm::Value * b);
    virtual llvm::Value * simd_sub(unsigned fw, llvm::Value * a, llvm::Value * b);
    virtual llvm::Value * simd_mult(unsigned fw, llvm::Value * a, llvm::Value * b);
    virtual llvm::Value * simd_eq(unsigned fw, llvm::Value * a, llvm::Value * b);
    virtual llvm::Value * simd_gt(unsigned fw, llvm::Value * a, llvm::Value * b);
    virtual llvm::Value * simd_ugt(unsigned fw, llvm::Value * a, llvm::Value * b);
    virtual llvm::Value * simd_lt(unsigned fw, llvm::Value * a, llvm::Value * b);
    virtual llvm::Value * simd_ult(unsigned fw, llvm::Value * a, llvm::Value * b);
    virtual llvm::Value * simd_max(unsigned fw, llvm::Value * a, llvm::Value * b);
    virtual llvm::Value * simd_umax(unsigned fw, llvm::Value * a, llvm::Value * b);
    virtual llvm::Value * simd_min(unsigned fw, llvm::Value * a, llvm::Value * b);
    virtual llvm::Value * simd_umin(unsigned fw, llvm::Value * a, llvm::Value * b);
    virtual llvm::Value * simd_if(unsigned fw, llvm::Value * cond, llvm::Value * a, llvm::Value * b);
    
    virtual llvm::Value * simd_slli(unsigned fw, llvm::Value * a, unsigned shift);
    virtual llvm::Value * simd_srli(unsigned fw, llvm::Value * a, unsigned shift);
    virtual llvm::Value * simd_srai(unsigned fw, llvm::Value * a, unsigned shift);
    
    virtual llvm::Value * simd_cttz(unsigned fw, llvm::Value * a);
    virtual llvm::Value * simd_popcount(unsigned fw, llvm::Value * a);
    
    virtual llvm::Value * esimd_mergeh(unsigned fw, llvm::Value * a, llvm::Value * b);
    virtual llvm::Value * esimd_mergel(unsigned fw, llvm::Value * a, llvm::Value * b);
    virtual llvm::Value * esimd_bitspread(unsigned fw, llvm::Value * bitmask);
    
    virtual llvm::Value * hsimd_packh(unsigned fw, llvm::Value * a, llvm::Value * b);
    virtual llvm::Value * hsimd_packl(unsigned fw, llvm::Value * a, llvm::Value * b);
    virtual llvm::Value * hsimd_packh_in_lanes(unsigned lanes, unsigned fw, llvm::Value * a, llvm::Value * b);
    virtual llvm::Value * hsimd_packl_in_lanes(unsigned lanes, unsigned fw, llvm::Value * a, llvm::Value * b);

    virtual llvm::Value * hsimd_signmask(unsigned fw, llvm::Value * a);
    
    virtual llvm::Value * mvmd_extract(unsigned fw, llvm::Value * a, unsigned fieldIndex);
    virtual llvm::Value * mvmd_insert(unsigned fw, llvm::Value * blk, llvm::Value * elt, unsigned fieldIndex);
    virtual llvm::Value * mvmd_slli(unsigned fw, llvm::Value * a, unsigned shift);
    virtual llvm::Value * mvmd_srli(unsigned fw, llvm::Value * a, unsigned shift);
    virtual llvm::Value * mvmd_dslli(unsigned fw, llvm::Value * a, llvm::Value * b, unsigned shift);
    
    
    virtual llvm::Value * bitblock_any(llvm::Value * a);
    // full add producing {carryout, sum}
    virtual std::pair<llvm::Value *, llvm::Value *> bitblock_add_with_carry(llvm::Value * a, llvm::Value * b, llvm::Value * carryin);
    // full shift producing {shiftout, shifted}
    virtual std::pair<llvm::Value *, llvm::Value *> bitblock_advance(llvm::Value * a, llvm::Value * shiftin, unsigned shift);
    virtual llvm::Value * bitblock_mask_from(llvm::Value * pos);
    virtual llvm::Value * bitblock_set_bit(llvm::Value * pos);
    
    llvm::Value * simd_and(llvm::Value * a, llvm::Value * b);
    llvm::Value * simd_or(llvm::Value * a, llvm::Value * b);
    llvm::Value * simd_xor(llvm::Value * a, llvm::Value * b);
    llvm::Value * simd_not(llvm::Value * a);
    llvm::Value * fwCast(unsigned fw, llvm::Value * a);
    
    inline llvm::VectorType * getBitBlockType() const {
        return mBitBlockType;
    }

    inline llvm::ArrayType * getStreamSetTy(const unsigned NumElements = 1, const unsigned FieldWidth = 1) {
        return llvm::ArrayType::get(getStreamTy(FieldWidth), NumElements);
    }
    
    llvm::VectorType * getStreamTy(const unsigned FieldWidth = 1) {
        return llvm::VectorType::get(llvm::IntegerType::getIntNTy(getContext(), FieldWidth), 0);
    }

    void CallPrintRegister(const std::string & regName, llvm::Value * const value);
    
protected:
    unsigned            mBitBlockWidth;
    unsigned            mStride;
    llvm::VectorType *  mBitBlockType;
    llvm::Constant *    mZeroInitializer;
    llvm::Constant *    mOneInitializer;
    llvm::Constant *    mPrintRegisterFunction;
};

inline llvm::LoadInst * IDISA_Builder::CreateBlockAlignedLoad(llvm::Value * const ptr) {
    return CreateAlignedLoad(ptr, mBitBlockWidth / 8);
}

inline llvm::LoadInst * IDISA_Builder::CreateBlockAlignedLoad(llvm::Value * const ptr, llvm::Value * const index) {
    return CreateBlockAlignedLoad(CreateGEP(ptr, index));
}

inline llvm::LoadInst * IDISA_Builder::CreateBlockAlignedLoad(llvm::Value * const ptr, std::initializer_list<llvm::Value *> indices) {
    return CreateBlockAlignedLoad(CreateGEP(ptr, indices));
}

inline void IDISA_Builder::CreateBlockAlignedStore(llvm::Value * const value, llvm::Value * const ptr) {
    CreateAlignedStore(value, ptr, mBitBlockWidth / 8);
}

inline void IDISA_Builder::CreateBlockAlignedStore(llvm::Value * const value, llvm::Value * const ptr, llvm::Value * const index) {
    CreateBlockAlignedStore(value, CreateGEP(ptr, index));
}

inline void IDISA_Builder::CreateBlockAlignedStore(llvm::Value * const value, llvm::Value * const ptr, std::initializer_list<llvm::Value *> indices) {
    CreateBlockAlignedStore(value, CreateGEP(ptr, indices));
}
    

    
}
#endif // IDISA_BUILDER_H
