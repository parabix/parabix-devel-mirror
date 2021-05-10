#ifndef IDISA_BUILDER_H
#define IDISA_BUILDER_H

/*
 *  Copyright (c) 2018 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */
#include <codegen/CBuilder.h>
#include <llvm/IR/DerivedTypes.h>
namespace llvm { class Constant; }
namespace llvm { class LoadInst; }
namespace llvm { class Module; }
namespace llvm { class Value; }
namespace llvm { class StringRef; }

namespace IDISA {


inline bool isStreamTy(llvm::Type * t) {
    return t->isVectorTy() && (llvm::cast<llvm::VectorType>(t)->getNumElements() == 0);
}

inline bool isStreamSetTy(llvm::Type * t) {
    return t->isArrayTy() && (isStreamTy(t->getArrayElementType()));
}

inline unsigned getNumOfStreams (llvm::Type * t) {
    if (isStreamTy(t)) return 1;
    assert(isStreamSetTy(t));
    return t->getArrayNumElements();
}

inline unsigned getStreamFieldWidth (llvm::Type * t) {
    if (isStreamTy(t)) return t->getScalarSizeInBits();
    assert(isStreamSetTy(t));
    return t->getArrayElementType()->getScalarSizeInBits();
}

unsigned getVectorBitWidth(llvm::Value * vec);

class IDISA_Builder : public CBuilder {

public:

    IDISA_Builder(llvm::LLVMContext & C, unsigned nativeVectorWidth, unsigned vectorWidth, unsigned laneWidth);

    virtual ~IDISA_Builder();

    virtual std::string getBuilderUniqueName() = 0;  // A name uniquely identifying builder/bitBlockWidth/stride.

    llvm::Value * bitCast(llvm::Value * a) {
        return fwCast(mLaneWidth, a);
    }

    unsigned getBitBlockWidth() const {
        return mBitBlockWidth;
    }

    llvm::Constant * allZeroes() const {
        return mZeroInitializer;
    }

    llvm::Constant * allOnes() const {
        return mOneInitializer;
    }

    llvm::Constant * getConstantVectorSequence(unsigned fw, unsigned first, unsigned last, unsigned by = 1);

    llvm::Value * CreateHalfVectorHigh(llvm::Value *);

    llvm::Value * CreateHalfVectorLow(llvm::Value *);

    llvm::Value * CreateDoubleVector(llvm::Value * lo, llvm::Value * hi);

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

    llvm::CallInst * createCall(llvm::Value *callee, llvm::ArrayRef<llvm::Value *> args, const llvm::Twine &Name = "") {
        #if LLVM_VERSION_MAJOR >= 11
            auto *calleePtrType = llvm::cast<llvm::PointerType>(callee->getType());
            auto *calleeType = llvm::cast<llvm::FunctionType>(calleePtrType->getElementType());
            return CreateCall(calleeType, callee, args, Name);
        #else
            return CreateCall(callee, args, Name);
        #endif
    }

    llvm::VectorType * fwVectorType(const unsigned fw);

    llvm::Constant * simd_himask(unsigned fw);
    llvm::Constant * simd_lomask(unsigned fw);

    llvm::Value * simd_select_hi(unsigned fw, llvm::Value * a);
    llvm::Value * simd_select_lo(unsigned fw, llvm::Value * a);

    virtual llvm::Value * simd_fill(unsigned fw, llvm::Value * a);

    virtual llvm::Value * simd_add(unsigned fw, llvm::Value * a, llvm::Value * b);
    virtual llvm::Value * simd_sub(unsigned fw, llvm::Value * a, llvm::Value * b);
    virtual llvm::Value * simd_mult(unsigned fw, llvm::Value * a, llvm::Value * b);
    virtual llvm::Value * simd_eq(unsigned fw, llvm::Value * a, llvm::Value * b);
    virtual llvm::Value * simd_ne(unsigned fw, llvm::Value * a, llvm::Value * b);
    virtual llvm::Value * simd_gt(unsigned fw, llvm::Value * a, llvm::Value * b);
    virtual llvm::Value * simd_ge(unsigned fw, llvm::Value * a, llvm::Value * b);
    virtual llvm::Value * simd_lt(unsigned fw, llvm::Value * a, llvm::Value * b);
    virtual llvm::Value * simd_le(unsigned fw, llvm::Value * a, llvm::Value * b);
    virtual llvm::Value * simd_ugt(unsigned fw, llvm::Value * a, llvm::Value * b);
    virtual llvm::Value * simd_ult(unsigned fw, llvm::Value * a, llvm::Value * b);
    virtual llvm::Value * simd_ule(unsigned fw, llvm::Value * a, llvm::Value * b);
    virtual llvm::Value * simd_uge(unsigned fw, llvm::Value * a, llvm::Value * b);
    virtual llvm::Value * simd_max(unsigned fw, llvm::Value * a, llvm::Value * b);
    virtual llvm::Value * simd_umax(unsigned fw, llvm::Value * a, llvm::Value * b);
    virtual llvm::Value * simd_min(unsigned fw, llvm::Value * a, llvm::Value * b);
    virtual llvm::Value * simd_umin(unsigned fw, llvm::Value * a, llvm::Value * b);
    virtual llvm::Value * simd_if(unsigned fw, llvm::Value * cond, llvm::Value * a, llvm::Value * b);
    //
    // Return a logic expression in terms of bitwise And, Or and Not for an
    // arbitrary two-operand boolean function corresponding to a 4-bit truth table mask.
    // The 4-bit mask dcba specifies the two-operand function fn defined by
    // the following table.
    //  bit_1  bit_0   fn
    //    0      0     a
    //    0      1     b
    //    1      0     c
    //    1      1     d
    llvm::Value * simd_binary(unsigned char mask, llvm::Value * bit_1, llvm::Value * bit_0);

    //
    // Return a logic expression in terms of bitwise And, Or and Not for an
    // arbitrary three-operand boolean function corresponding to an 8-bit truth table mask.
    // The 8-bit mask hgfedcba specifies the three-operand function fn defined by
    // the following table.
    //  bit_2  bit_1  bit_0   fn
    //    0      0      0     a
    //    0      0      1     b
    //    0      1      0     c
    //    0      1      1     d
    //    1      0      0     e
    //    1      0      1     f
    //    1      1      0     g
    //    1      1      1     h
    virtual llvm::Value * simd_ternary(unsigned char mask, llvm::Value * bit_2, llvm::Value * bit_1, llvm::Value * bit_0);

    virtual llvm::Value * simd_slli(unsigned fw, llvm::Value * a, unsigned shift);
    virtual llvm::Value * simd_srli(unsigned fw, llvm::Value * a, unsigned shift);
    virtual llvm::Value * simd_srai(unsigned fw, llvm::Value * a, unsigned shift);
    virtual llvm::Value * simd_sllv(unsigned fw, llvm::Value * a, llvm::Value * shifts);
    virtual llvm::Value * simd_srlv(unsigned fw, llvm::Value * a, llvm::Value * shifts);

    virtual llvm::Value * simd_pext(unsigned fw, llvm::Value * v, llvm::Value * extract_mask);
    virtual llvm::Value * simd_pdep(unsigned fw, llvm::Value * v, llvm::Value * deposit_mask);
    virtual llvm::Value * simd_any(unsigned fw, llvm::Value * a);
    virtual llvm::Value * simd_popcount(unsigned fw, llvm::Value * a);
    virtual llvm::Value * hsimd_partial_sum(unsigned fw, llvm::Value * a);
    virtual llvm::Value * simd_cttz(unsigned fw, llvm::Value * a);

    virtual llvm::Value * simd_bitreverse(unsigned fw, llvm::Value * a);

    virtual llvm::Value * esimd_mergeh(unsigned fw, llvm::Value * a, llvm::Value * b);
    virtual llvm::Value * esimd_mergel(unsigned fw, llvm::Value * a, llvm::Value * b);
    virtual llvm::Value * esimd_bitspread(unsigned fw, llvm::Value * bitmask);

    virtual llvm::Value * hsimd_packh(unsigned fw, llvm::Value * a, llvm::Value * b);
    virtual llvm::Value * hsimd_packl(unsigned fw, llvm::Value * a, llvm::Value * b);
    // Pack signed values with signed/unsigned saturation.
    virtual llvm::Value * hsimd_packss(unsigned fw, llvm::Value * a, llvm::Value * b);
    virtual llvm::Value * hsimd_packus(unsigned fw, llvm::Value * a, llvm::Value * b);
    virtual llvm::Value * hsimd_packh_in_lanes(unsigned lanes, unsigned fw, llvm::Value * a, llvm::Value * b);
    virtual llvm::Value * hsimd_packl_in_lanes(unsigned lanes, unsigned fw, llvm::Value * a, llvm::Value * b);

    virtual llvm::Value * hsimd_signmask(unsigned fw, llvm::Value * a);

    virtual llvm::Value * mvmd_extract(unsigned fw, llvm::Value * a, unsigned fieldIndex);
    virtual llvm::Value * mvmd_insert(unsigned fw, llvm::Value * blk, llvm::Value * elt, unsigned fieldIndex);

    virtual llvm::Value * mvmd_sll(unsigned fw, llvm::Value * value, llvm::Value * shift, const bool safe = false);
    virtual llvm::Value * mvmd_srl(unsigned fw, llvm::Value * value, llvm::Value * shift, const bool safe = false);
    virtual llvm::Value * mvmd_slli(unsigned fw, llvm::Value * a, unsigned shift);
    virtual llvm::Value * mvmd_srli(unsigned fw, llvm::Value * a, unsigned shift);
    virtual llvm::Value * mvmd_dslli(unsigned fw, llvm::Value * a, llvm::Value * b, unsigned shift);
    virtual llvm::Value * mvmd_dsll(unsigned fw, llvm::Value * a, llvm::Value * b, llvm::Value * shift);
    virtual llvm::Value * mvmd_shuffle(unsigned fw, llvm::Value * data_table, llvm::Value * index_vector);
    virtual llvm::Value * mvmd_shuffle2(unsigned fw, llvm::Value * table0, llvm::Value * table1, llvm::Value * index_vector);
    virtual llvm::Value * mvmd_compress(unsigned fw, llvm::Value * a, llvm::Value * select_mask);


    virtual llvm::Value * bitblock_any(llvm::Value * a);
    // full add producing {carryout, sum}
    virtual std::pair<llvm::Value *, llvm::Value *> bitblock_add_with_carry(llvm::Value * a, llvm::Value * b, llvm::Value * carryin);
    virtual std::pair<llvm::Value *, llvm::Value *> bitblock_subtract_with_borrow(llvm::Value * a, llvm::Value * b, llvm::Value * borrowin);
    virtual std::pair<llvm::Value *, llvm::Value *> bitblock_subtract_with_propagate(llvm::Value * a, llvm::Value * b, llvm::Value * propagateIn);
    // full shift producing {shiftout, shifted}
    virtual std::pair<llvm::Value *, llvm::Value *> bitblock_advance(llvm::Value * a, llvm::Value * shiftin, unsigned shift);
    virtual std::pair<llvm::Value *, llvm::Value *> bitblock_indexed_advance(llvm::Value * a, llvm::Value * index_strm, llvm::Value * shiftin, unsigned shift);

    virtual llvm::Value * bitblock_mask_from(llvm::Value * const position, const bool safe = false);
    virtual llvm::Value * bitblock_mask_to(llvm::Value * const position, const bool safe = false);
    virtual llvm::Value * bitblock_set_bit(llvm::Value * const position, const bool safe = false);

    // returns a scalar with the popcount of this block
    llvm::Value * bitblock_popcount(llvm::Value * const to_count);

    virtual void CreateBaseFunctions() {}

    llvm::Value * simd_and(llvm::Value * a, llvm::Value * b, llvm::StringRef s = llvm::StringRef());
    llvm::Value * simd_or(llvm::Value * a, llvm::Value * b, llvm::StringRef s = llvm::StringRef());
    llvm::Value * simd_xor(llvm::Value * a, llvm::Value * b, llvm::StringRef s = llvm::StringRef());
    llvm::Value * simd_not(llvm::Value * a, llvm::StringRef s = llvm::StringRef());
    llvm::Value * fwCast(unsigned fw, llvm::Value * a);

    inline llvm::VectorType * getBitBlockType() const {
        return mBitBlockType;
    }

    static llvm::VectorType * LLVM_READNONE getStreamTy(llvm::LLVMContext & C, const unsigned FieldWidth = 1) {
        return llvm::VectorType::get(llvm::IntegerType::getIntNTy(C, FieldWidth), 0);
    }

    static llvm::ArrayType * LLVM_READNONE getStreamSetTy(llvm::LLVMContext & C, const unsigned NumElements = 1, const unsigned FieldWidth = 1) {
        return llvm::ArrayType::get(getStreamTy(C, FieldWidth), NumElements);
    }

    llvm::VectorType * getStreamTy(const unsigned FieldWidth = 1) {
        return getStreamTy(getContext(), FieldWidth);
    }

    llvm::ArrayType * getStreamSetTy(const unsigned NumElements = 1, const unsigned FieldWidth = 1) {
        return getStreamSetTy(getContext(), NumElements, FieldWidth);
    }

    llvm::CallInst * CallPrintRegister(llvm::StringRef regName, llvm::Value * const value, const STD_FD fd = STD_FD::STD_ERR);

protected:
    LLVM_ATTRIBUTE_NORETURN void UnsupportedFieldWidthError(const unsigned FieldWidth, std::string op_name);

    llvm::Constant * bit_interleave_byteshuffle_table(unsigned fw);  // support function for merge using shuffles.

    const unsigned              mNativeBitBlockWidth;
    const unsigned              mBitBlockWidth;
    const unsigned              mLaneWidth;
    llvm::VectorType * const    mBitBlockType;
    llvm::Constant * const      mZeroInitializer;
    llvm::Constant * const      mOneInitializer;
    llvm::Constant *            mPrintRegisterFunction;
};

}
#endif // IDISA_BUILDER_H
