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
#include <llvm/Support/Host.h>
#include <llvm/ADT/Triple.h>
#include <IR_Gen/CBuilder.h>
#include <IR_Gen/types/streamtype.h>
#include <boost/container/flat_map.hpp>

using namespace llvm;

namespace IDISA {

class IDISA_Builder : public CBuilder {

    using StreamTypes = boost::container::flat_map<unsigned, StreamType *>;

public:

    IDISA_Builder(Module * m, unsigned archBitWidth, unsigned bitBlockWidth, unsigned stride, unsigned CacheAlignment=64);

    virtual ~IDISA_Builder();
    
    std::string getBitBlockTypeName() const;  // A short string such as v4i64 or i256.

    Value * bitCast(Value * a) {
        return (a->getType() == mBitBlockType) ? a : CreateBitCast(a, mBitBlockType);
    }

    unsigned getBitBlockWidth() const {
        return mBitBlockWidth;
    }

    unsigned getStride() const {
        return mStride;
    }

    Constant * allZeroes() const {
        return mZeroInitializer;
    }

    Constant * allOnes() const {
        return mOneInitializer;
    }
    

    LoadInst * CreateBlockAlignedLoad(Value * const ptr);
    LoadInst * CreateBlockAlignedLoad(Value * const ptr, Value * const index);
    LoadInst * CreateBlockAlignedLoad(Value * const ptr, std::initializer_list<Value *> indices);

    void CreateBlockAlignedStore(Value * const value, Value * const ptr);
    void CreateBlockAlignedStore(Value * const value, Value * const ptr, Value * const index);
    void CreateBlockAlignedStore(Value * const value, Value * const ptr, std::initializer_list<Value *> indices);

    VectorType * fwVectorType(unsigned fw);

    Constant * simd_himask(unsigned fw);
    Constant * simd_lomask(unsigned fw);
    
    virtual Value * simd_fill(unsigned fw, Value * a);

    virtual Value * simd_add(unsigned fw, Value * a, Value * b);
    virtual Value * simd_sub(unsigned fw, Value * a, Value * b);
    virtual Value * simd_mult(unsigned fw, Value * a, Value * b);
    virtual Value * simd_eq(unsigned fw, Value * a, Value * b);
    virtual Value * simd_gt(unsigned fw, Value * a, Value * b);
    virtual Value * simd_ugt(unsigned fw, Value * a, Value * b);
    virtual Value * simd_lt(unsigned fw, Value * a, Value * b);
    virtual Value * simd_ult(unsigned fw, Value * a, Value * b);
    virtual Value * simd_max(unsigned fw, Value * a, Value * b);
    virtual Value * simd_umax(unsigned fw, Value * a, Value * b);
    virtual Value * simd_min(unsigned fw, Value * a, Value * b);
    virtual Value * simd_umin(unsigned fw, Value * a, Value * b);
    virtual Value * simd_if(unsigned fw, Value * cond, Value * a, Value * b);
    
    virtual Value * simd_slli(unsigned fw, Value * a, unsigned shift);
    virtual Value * simd_srli(unsigned fw, Value * a, unsigned shift);
    virtual Value * simd_srai(unsigned fw, Value * a, unsigned shift);
    
    virtual Value * simd_cttz(unsigned fw, Value * a);
    virtual Value * simd_popcount(unsigned fw, Value * a);
    
    virtual Value * esimd_mergeh(unsigned fw, Value * a, Value * b);
    virtual Value * esimd_mergel(unsigned fw, Value * a, Value * b);
    virtual Value * esimd_bitspread(unsigned fw, Value * bitmask);
    
    virtual Value * hsimd_packh(unsigned fw, Value * a, Value * b);
    virtual Value * hsimd_packl(unsigned fw, Value * a, Value * b);
    virtual Value * hsimd_packh_in_lanes(unsigned lanes, unsigned fw, Value * a, Value * b);
    virtual Value * hsimd_packl_in_lanes(unsigned lanes, unsigned fw, Value * a, Value * b);

    virtual Value * hsimd_signmask(unsigned fw, Value * a);
    
    virtual Value * mvmd_extract(unsigned fw, Value * a, unsigned fieldIndex);
    virtual Value * mvmd_insert(unsigned fw, Value * blk, Value * elt, unsigned fieldIndex);
    virtual Value * mvmd_slli(unsigned fw, Value * a, unsigned shift);
    virtual Value * mvmd_srli(unsigned fw, Value * a, unsigned shift);
    virtual Value * mvmd_dslli(unsigned fw, Value * a, Value * b, unsigned shift);
    
    
    virtual Value * bitblock_any(Value * a);
    // full add producing {carryout, sum}
    virtual std::pair<Value *, Value *> bitblock_add_with_carry(Value * a, Value * b, Value * carryin);
    // full shift producing {shiftout, shifted}
    virtual std::pair<Value *, Value *> bitblock_advance(Value * a, Value * shiftin, unsigned shift);
    virtual Value * bitblock_mask_from(Value * pos);
    virtual Value * bitblock_set_bit(Value * pos);
    
    Value * simd_and(Value * a, Value * b);
    Value * simd_or(Value * a, Value * b);
    Value * simd_xor(Value * a, Value * b);
    Value * simd_not(Value * a);
    Value * fwCast(unsigned fw, Value * a);
    
    inline VectorType * getBitBlockType() const {
        return mBitBlockType;
    }

    inline Type * getStreamSetTy(const unsigned NumElements = 1, const unsigned FieldWidth = 1) {
        return ArrayType::get(getStreamTy(FieldWidth), NumElements);
    }
    
    Type * getStreamTy(const unsigned FieldWidth = 1);

    void CallPrintRegister(const std::string & regName, Value * const value);
    
protected:
    unsigned            mBitBlockWidth;
    unsigned            mStride;
    VectorType *        mBitBlockType;

    Constant *          mZeroInitializer;
    Constant *          mOneInitializer;
    Constant *          mPrintRegisterFunction;
    StreamTypes         mStreamTypes;
};

inline LoadInst * IDISA_Builder::CreateBlockAlignedLoad(Value * const ptr) {
    return CreateAlignedLoad(ptr, mBitBlockWidth / 8);
}

inline LoadInst * IDISA_Builder::CreateBlockAlignedLoad(Value * const ptr, Value * const index) {
    return CreateBlockAlignedLoad(CreateGEP(ptr, index));
}

inline LoadInst * IDISA_Builder::CreateBlockAlignedLoad(Value * const ptr, std::initializer_list<Value *> indices) {
    return CreateBlockAlignedLoad(CreateGEP(ptr, indices));
}

inline void IDISA_Builder::CreateBlockAlignedStore(Value * const value, Value * const ptr) {
    CreateAlignedStore(value, ptr, mBitBlockWidth / 8);
}

inline void IDISA_Builder::CreateBlockAlignedStore(Value * const value, Value * const ptr, Value * const index) {
    CreateBlockAlignedStore(value, CreateGEP(ptr, index));
}

inline void IDISA_Builder::CreateBlockAlignedStore(Value * const value, Value * const ptr, std::initializer_list<Value *> indices) {
    CreateBlockAlignedStore(value, CreateGEP(ptr, indices));
}
    

    
}
#endif // IDISA_BUILDER_H
