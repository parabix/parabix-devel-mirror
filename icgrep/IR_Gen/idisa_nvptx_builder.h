#ifndef IDISA_NVPTX_BUILDER_H
#define IDISA_NVPTX_BUILDER_H

/*
 *  Copyright (c) 2015 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
*/

#include <IR_Gen/idisa_i64_builder.h>

namespace IDISA {

class IDISA_NVPTX20_Builder : public IDISA_I64_Builder {
public:
    static const unsigned NativeBitBlockWidth = 4096;
    IDISA_NVPTX20_Builder(llvm::LLVMContext & C, unsigned vectorWidth, unsigned laneWidth)
    : IDISA_Builder(C, NativeBitBlockWidth, vectorWidth, laneWidth)
    , IDISA_I64_Builder(C, laneWidth, laneWidth)
    , groupThreads(vectorWidth / laneWidth)
    , barrierFunc(nullptr)
    , tidFunc(nullptr)
    , mLongAdvanceFunc(nullptr)
    , mLongAddFunc(nullptr)
    , carry(nullptr)
    , bubble(nullptr) {
        assert ((vectorWidth % laneWidth) == 0);
    }

    ~IDISA_NVPTX20_Builder() {}

    virtual std::string getBuilderUniqueName() override;

    unsigned getGroupThreads() const;

    void CreateBaseFunctions() override;
    
    llvm::Value * bitblock_any(llvm::Value * a) override;
    std::pair<llvm::Value *, llvm::Value *> bitblock_add_with_carry(llvm::Value * a, llvm::Value * b, llvm::Value * carryin) override;
    virtual std::pair<llvm::Value *, llvm::Value *> bitblock_advance(llvm::Value * a, llvm::Value * shiftin, unsigned shift) override;
    llvm::Value * bitblock_mask_from(llvm::Value * pos) override;
    llvm::Value * bitblock_set_bit(llvm::Value * pos) override;

    llvm::Value * getEOFMask(llvm::Value * remainingBytes);

    llvm::Value * Advance(const unsigned index, const unsigned shiftAmount, llvm::Value * const value);
    llvm::Value * LongAdd(llvm::Value * const valA, llvm::Value * const valB, llvm::Value * carryIn);

    llvm::LoadInst * CreateAtomicLoadAcquire(llvm::Value * ptr) override;
    llvm::StoreInst * CreateAtomicStoreRelease(llvm::Value * val, llvm::Value * ptr) override;

    bool supportsIndirectBr() const final {
        return false;
    }

    #ifdef HAS_ADDRESS_SANITIZER
    llvm::LoadInst * CreateLoad(llvm::Value *Ptr, const char *Name) override;

    llvm::LoadInst * CreateLoad(llvm::Value *Ptr, const llvm::Twine &Name = "") override;

    llvm::LoadInst * CreateLoad(llvm::Type *Ty, llvm::Value *Ptr, const llvm::Twine &Name = "") override;

    llvm::LoadInst * CreateLoad(llvm::Value *Ptr, bool isVolatile, const llvm::Twine &Name = "") override;

    llvm::StoreInst * CreateStore(llvm::Value *Val, llvm::Value *Ptr, bool isVolatile = false) override;
    #endif

private:

    void CreateGlobals();
    void CreateBuiltinFunctions();
    void CreateLongAdvanceFunc();
    void CreateLongAddFunc();
    void CreateBallotFunc();

private:
    const unsigned              groupThreads;
    llvm::Function *            barrierFunc;
    llvm::Function *            tidFunc;
    llvm::Function *            mLongAdvanceFunc;
    llvm::Function *            mLongAddFunc;
    llvm::GlobalVariable*       carry;
    llvm::GlobalVariable*       bubble;
};

#if 0

class IDISA_NVPTX35_Builder : public IDISA_NVPTX20_Builder {
    IDISA_NVPTX35_Builder(Module * m, int groupSize) : IDISA_NVPTX30_Builder(m, groupSize) {}
    
    std::pair<llvm::Value *, llvm::Value *> bitblock_advance(llvm::Value * a, llvm::Value * shiftin, unsigned shift) override;

    ~IDISA_NVPTX35_Builder() {};
    virtual std::string getBuilderUniqueName() override;
};
#endif    
    
}
#endif // IDISA_NVPTX_BUILDER_H
