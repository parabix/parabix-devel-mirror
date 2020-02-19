/*
 *  Copyright (c) 2016 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 */

#ifndef PABLO_KERNEL_H
#define PABLO_KERNEL_H

#include <kernel/core/kernel.h>
#include <pablo/pabloAST.h>
#include <pablo/symbol_generator.h>
#include <util/slab_allocator.h>
#include <llvm/ADT/StringRef.h>

namespace llvm { class Type; }
namespace llvm { class VectorType; }

namespace pablo { class Integer; }
namespace pablo { class Ones; }
namespace pablo { class PabloBlock; }
namespace pablo { class PabloCompiler; }
namespace pablo { class String; }
namespace pablo { class Var; }
namespace pablo { class Extract; }
namespace pablo { class Zeroes; }

namespace pablo {

class PabloKernel : public kernel::BlockOrientedKernel, public PabloAST {

    friend class PabloCompiler;
    friend class PabloBlock;
    friend class CarryManager;
    friend class CarryPackManager;

public:

    using KernelBuilder = kernel::KernelBuilder;

    using Allocator = SlabAllocator<PabloAST *>;

    template <typename T, unsigned n>
    using Vec = llvm::SmallVector<T, n>;

    static inline bool classof(const PabloAST * e) {
        return e->getClassTypeId()  == PabloAST::ClassTypeId::Kernel;
    }
    static inline bool classof(const PabloKernel *) {
        return true;
    }
    static inline bool classof(const void *) {
        return false;
    }

    virtual ~PabloKernel();

    PabloBlock * getEntryScope() const {
        return mEntryScope;
    }

    PabloBlock * setEntryScope(PabloBlock * entryBlock) {
        assert (entryBlock);
        std::swap(mEntryScope, entryBlock);
        return entryBlock;
    }

    Var * getInputStreamVar(const std::string & name);

    std::vector<PabloAST *> getInputStreamSet(const std::string & name);

    Var * getInput(const unsigned index) {
        assert (index < mInputs.size() && mInputs[index]);
        return mInputs[index];
    }

    const Var * getInput(const unsigned index) const {
        assert (index < mInputs.size() && mInputs[index]);
        return mInputs[index];
    }

    unsigned getNumOfInputs() const {
        return mInputs.size();
    }

    Var * getOutputStreamVar(const std::string & name);

    Var * getOutputScalarVar(const std::string & name);

    Var * getOutput(const unsigned index) {
        assert (index < mOutputs.size() && mOutputs[index]);
        return mOutputs[index];
    }

    const Var * getOutput(const unsigned index) const {
        assert (index < mOutputs.size() && mOutputs[index]);
        return mOutputs[index];
    }

    unsigned getNumOfOutputs() const {
        return mOutputs.size();
    }

    Var * getVariable(const unsigned index) {
        assert (index < mVariables.size() && mVariables[index]);
        return mVariables[index];
    }

    unsigned getNumOfVariables() const {
        return mVariables.size();
    }

    Zeroes * getNullValue(llvm::Type * const type);

    Ones * getAllOnesValue(llvm::Type * const type);

    inline SymbolGenerator * getSymbolTable() const {
        return mSymbolTable.get();
    }

    void * operator new (std::size_t size) noexcept {
        return std::malloc(size);
    }

    void operator delete(void* ptr) noexcept {
        std::free(ptr);
    }

    String * makeName(const llvm::StringRef prefix) const;

    Integer * getInteger(const int64_t value, unsigned intWidth = 64) const;

    llvm::StructType * getCarryDataTy() const {
        return mCarryDataTy;
    }

    llvm::LLVMContext & getContext() const {
        assert (mContext);
        return *mContext;
    }

protected:

    PabloKernel(BuilderRef builder,
                std::string && kernelName,
                std::vector<kernel::Binding> stream_inputs = {},
                std::vector<kernel::Binding> stream_outputs = {},
                std::vector<kernel::Binding> scalar_parameters = {},
                std::vector<kernel::Binding> scalar_outputs = {});

    virtual void generatePabloMethod() = 0;

    llvm::IntegerType * getSizeTy() const {
        assert (mSizeTy); return mSizeTy;
    }

    llvm::VectorType * getStreamTy() const {
        assert (mStreamTy); return mStreamTy;
    }

    llvm::IntegerType * getInt1Ty() const;

    void setCarryDataTy(llvm::StructType * const carryDataTy) {
        mCarryDataTy = carryDataTy;
    }

    Var * makeVariable(const String * const name, llvm::Type * const type);

    Extract * makeExtract(Var * const array, PabloAST * const index);

    // A custom method for preparing kernel declarations is needed,
    // so that the carry data requirements may be accommodated before
    // finalizing the KernelStateType.
    void addInternalProperties(BuilderRef b) final;

    bool requiresExplicitPartialFinalStride() const override;

    std::unique_ptr<kernel::KernelCompiler> instantiateKernelCompiler(BuilderRef b) const noexcept;

private:

    void generateDoBlockMethod(BuilderRef b) final;

    // The default method for Pablo final block processing sets the
    // EOFmark bit and then calls the standard DoBlock function.
    // This may be overridden for specialized processing.
    void generateFinalBlockMethod(BuilderRef b, llvm::Value * remainingBytes) final;

    void generateFinalizeMethod(BuilderRef b) final;

private:

    Allocator                        mAllocator;
    mutable PabloCompiler *          mPabloCompiler = nullptr;
    std::unique_ptr<SymbolGenerator> mSymbolTable;
    PabloBlock *                     mEntryScope = nullptr;
    llvm::IntegerType *              mSizeTy = nullptr;
    llvm::VectorType *               mStreamTy = nullptr;
    llvm::StructType *               mCarryDataTy = nullptr;
    llvm::LLVMContext *              mContext = nullptr;

    Vec<Var *, 16>                   mInputs;
    Vec<Var *, 16>                   mOutputs;
    Vec<PabloAST *, 16>              mConstants;
    Vec<Var *, 64>                   mVariables;
    Vec<Var *, 16>                   mScalarOutputVars;
};

}

#endif // PABLO_KERNEL_H
