/*
 *  Copyright (c) 2016 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 */

#ifndef PABLO_KERNEL_H
#define PABLO_KERNEL_H

#include <kernels/kernel.h>
#include <pablo/pabloAST.h>
#include <pablo/symbol_generator.h>
#include <util/slab_allocator.h>
#include <llvm/ADT/StringRef.h>
#include <boost/container/flat_map.hpp>

namespace IDISA { class IDISA_Builder; }
namespace llvm { class Type; }
namespace pablo { class Integer; }
namespace pablo { class Ones; }
namespace pablo { class PabloBlock; }
namespace pablo { class PabloCompiler; }
namespace pablo { class String; }
namespace pablo { class Var; }
namespace pablo { class Zeroes; }

namespace pablo {

class PabloKernel : public kernel::BlockOrientedKernel, public PabloAST {

    friend class PabloCompiler;
    friend class PabloBlock;
    friend class CarryManager;

public:

    using Allocator = SlabAllocator<PabloAST *>;

    static inline bool classof(const PabloAST * e) {
        return e->getClassTypeId()  == PabloAST::ClassTypeId::Kernel;
    }
    static inline bool classof(const PabloKernel *) {
        return true;
    }
    static inline bool classof(const void *) {
        return false;
    }

    PabloKernel(IDISA::IDISA_Builder * builder, std::string kernelName,                      
                std::vector<Binding> stream_inputs = {},
                std::vector<Binding> stream_outputs = {},
                std::vector<Binding> scalar_parameters = {},
                std::vector<Binding> scalar_outputs = {});

    virtual ~PabloKernel();

    PabloBlock * getEntryBlock() {
        return mEntryBlock;
    }

    const PabloBlock * getEntryBlock() const {
        return mEntryBlock;
    }

    PabloBlock * setEntryBlock(PabloBlock * entryBlock) {
        assert (entryBlock);
        std::swap(mEntryBlock, entryBlock);
        return entryBlock;
    }
    
    Var * getInputStreamVar(const std::string & name);

    Var * getInput(const unsigned index) {
        return mInputs[index];
    }

    const Var * getInput(const unsigned index) const {
        return mInputs[index];
    }

    Var * addInput(const std::string & name, llvm::Type * const type);

    unsigned getNumOfInputs() const {
        return mInputs.size();
    }
    
    Var * getOutputStreamVar(const std::string & name);
    
    Var * getOutputScalarVar(const std::string & name);

    Var * getOutput(const unsigned index) {
        return mOutputs[index];
    }

    const Var * getOutput(const unsigned index) const {
        return mOutputs[index];
    }

    Var * addOutput(const std::string & name, llvm::Type * const type);

    unsigned getNumOfOutputs() const {
        return mOutputs.size();
    }

    Var * makeVariable(String * name, llvm::Type * const type);

    Var * getVariable(const unsigned index) {
        return mVariables[index];
    }

    unsigned getNumOfVariables() const {
        return mVariables.size();
    }

    Zeroes * getNullValue(llvm::Type * const type);

    Ones * getAllOnesValue(llvm::Type * const type);

    inline SymbolGenerator * getSymbolTable() const {
        return mSymbolTable;
    }

    void * operator new (std::size_t size) noexcept {
        return std::malloc(size);
    }

    void operator delete(void* ptr) noexcept {
        std::free(ptr);
    }

protected:

    // A custom method for preparing kernel declarations is needed,
    // so that the carry data requirements may be accommodated before
    // finalizing the KernelStateType.
    void prepareKernel()  override;

    void generateDoBlockMethod() override final;

    // The default method for Pablo final block processing sets the
    // EOFmark bit and then calls the standard DoBlock function.
    // This may be overridden for specialized processing.
    virtual void generateFinalBlockMethod(llvm::Value * remainingBytes) override final;

    inline String * makeName(const llvm::StringRef & prefix) const {
        return mSymbolTable->makeString(prefix, iBuilder);
    }

    inline Integer * getInteger(const int64_t value) const {
        return mSymbolTable->getInteger(value, iBuilder);
    }

private:

    Allocator                       mAllocator;
    PabloCompiler * const           mPabloCompiler;
    SymbolGenerator *               mSymbolTable;
    PabloBlock *                    mEntryBlock;

    std::vector<Var *>              mInputs;
    std::vector<Var *>              mOutputs;
    std::vector<PabloAST *>         mConstants;
    std::vector<Var *>              mVariables;
    boost::container::flat_map<std::string, Var *> mScalarOutputNameMap;
};

}

#endif // PABLO_KERNEL_H
