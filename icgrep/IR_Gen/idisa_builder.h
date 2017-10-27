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

    IDISA_Builder(llvm::LLVMContext & C, unsigned vectorWidth, unsigned stride);

    virtual ~IDISA_Builder();
    
    virtual std::string getBuilderUniqueName() = 0;  // A name uniquely identifying builder/bitBlockWidth/stride.
    
    llvm::Value * bitCast(llvm::Value * a) {
        return CreateBitCast(a, mBitBlockType);
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
    
    llvm::LoadInst * CreateBlockAlignedLoad(llvm::Value * const ptr) {
        return CreateAlignedLoad(ptr, mBitBlockWidth / 8);
    }

    llvm::LoadInst * CreateBlockAlignedLoad(llvm::Value * const ptr, llvm::Value * const index) {
        return CreateBlockAlignedLoad(CreateGEP(ptr, index));
    }

    llvm::LoadInst * CreateBlockAlignedLoad(llvm::Value * const ptr, std::initializer_list<llvm::Value *> indices) {
        return CreateBlockAlignedLoad(CreateGEP(ptr, indices));
    }

    llvm::StoreInst * CreateBlockAlignedStore(llvm::Value * const value, llvm::Value * const ptr) {
        return CreateAlignedStore(value, ptr, mBitBlockWidth / 8);
    }

    llvm::StoreInst * CreateBlockAlignedStore(llvm::Value * const value, llvm::Value * const ptr, llvm::Value * const index) {
        return CreateBlockAlignedStore(value, CreateGEP(ptr, index));
    }

    llvm::StoreInst * CreateBlockAlignedStore(llvm::Value * const value, llvm::Value * const ptr, std::initializer_list<llvm::Value *> indices) {
        return CreateBlockAlignedStore(value, CreateGEP(ptr, indices));
    }

    llvm::Value * CreateBlockAlignedMalloc(llvm::Value * size) {
        return CreateAlignedMalloc(size, mBitBlockWidth / 8);
    }

    llvm::VectorType * fwVectorType(const unsigned fw);

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
    virtual llvm::Value * simd_bitreverse(unsigned fw, llvm::Value * a);
    
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
    virtual std::pair<llvm::Value *, llvm::Value *> bitblock_indexed_advance(llvm::Value * a, llvm::Value * index_strm, llvm::Value * shiftin, unsigned shift);
    virtual llvm::Value * bitblock_mask_from(llvm::Value * pos);
    virtual llvm::Value * bitblock_set_bit(llvm::Value * pos);
    

    virtual void CreateBaseFunctions() {}
    
    llvm::Value * simd_and(llvm::Value * a, llvm::Value * b);
    llvm::Value * simd_or(llvm::Value * a, llvm::Value * b);
    llvm::Value * simd_xor(llvm::Value * a, llvm::Value * b);
    llvm::Value * simd_not(llvm::Value * a);
    llvm::Value * fwCast(unsigned fw, llvm::Value * a);
    
    inline llvm::VectorType * getBitBlockType() const {
        return mBitBlockType;
    }

    static llvm::VectorType * getStreamTy(llvm::LLVMContext & C, const unsigned FieldWidth = 1) {
        return llvm::VectorType::get(llvm::IntegerType::getIntNTy(C, FieldWidth), 0);
    }

    static llvm::ArrayType * getStreamSetTy(llvm::LLVMContext & C, const unsigned NumElements = 1, const unsigned FieldWidth = 1) {
        return llvm::ArrayType::get(getStreamTy(C, FieldWidth), NumElements);
    }

    llvm::VectorType * getStreamTy(const unsigned FieldWidth = 1) {
        return getStreamTy(getContext(), FieldWidth);
    }

    llvm::ArrayType * getStreamSetTy(const unsigned NumElements = 1, const unsigned FieldWidth = 1) {
        return getStreamSetTy(getContext(), NumElements, FieldWidth);
    }

    void CallPrintRegister(const std::string & regName, llvm::Value * const value);

protected:
    const unsigned              mBitBlockWidth;
    const unsigned              mStride;
    llvm::VectorType * const    mBitBlockType;
    llvm::Constant * const      mZeroInitializer;
    llvm::Constant * const      mOneInitializer;
    llvm::Constant *            mPrintRegisterFunction;
};

}
#endif // IDISA_BUILDER_H
