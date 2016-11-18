/*
 *  Copyright (c) 2016 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 */

#ifndef PABLO_KERNEL_H
#define PABLO_KERNEL_H

#include <kernels/kernel.h>
#include <pablo/symbol_generator.h>

namespace IDISA {
class IDISA_Builder;
}

namespace pablo {
    
class PabloCompiler;
class PabloBlock;
class PabloAST;
class CarryManager;
class Var;
class Zeroes;
class Ones;
class String;
class Integer;

class PabloKernel : public kernel::KernelBuilder {

    friend class PabloCompiler;
    friend class PabloBlock;
    friend class CarryManager;

public:

    PabloKernel(IDISA::IDISA_Builder * builder, const std::string & kernelName);
    // At present only population count accumulator are supported,
    // using the pablo.Count operation.
    
    ~PabloKernel();

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

    Var * getInput(const unsigned index) {
        return mInputs[index];
    }

    const Var * getInput(const unsigned index) const {
        return mInputs[index];
    }

    Var * addInput(const std::string name, Type * const type);

    unsigned getNumOfInputs() const {
        return mInputs.size();
    }

    Var * getOutput(const unsigned index) {
        return mOutputs[index];
    }

    const Var * getOutput(const unsigned index) const {
        return mOutputs[index];
    }

    Var * addOutput(const std::string name, Type * const type);

    unsigned getNumOfOutputs() const {
        return mOutputs.size();
    }

    Var * makeVariable(PabloAST * name, Type * const type);

    Var * getVariable(const unsigned index) {
        return mVariables[index];
    }

    unsigned getNumOfVariables() const {
        return mVariables.size();
    }

    Zeroes * getNullValue(Type * const type);

    Ones * getAllOnesValue(Type * const type);

    inline SymbolGenerator * getSymbolTable() const {
        return mSymbolTable;
    }

protected:

    // A custom method for preparing kernel declarations is needed,
    // so that the carry data requirements may be accommodated before
    // finalizing the KernelStateType.
    void prepareKernel() override;

    void generateDoBlockMethod() override;
    
    // The default method for Pablo final block processing sets the
    // EOFmark bit and then calls the standard DoBlock function.
    // This may be overridden for specialized processing.
    virtual void generateFinalBlockMethod() override;

    inline String * getName(const std::string & name) const {
        return mSymbolTable->get(name);
    }

    inline String * makeName(const std::string & prefix) const {
        return mSymbolTable->make(prefix);
    }

    inline Integer * getInteger(const int64_t value) const {
        return mSymbolTable->getInteger(value, iBuilder);
    }

private:

    PabloCompiler * const           mPabloCompiler;
    SymbolGenerator *               mSymbolTable;
    PabloBlock *                    mEntryBlock;
    std::vector<Var *>              mInputs;
    std::vector<Var *>              mOutputs;
    std::vector<PabloAST *>         mConstants;
    std::vector<Var *>              mVariables;

};

}

#endif // PABLO_KERNEL_H
