#ifndef FUNCTION_H
#define FUNCTION_H

#include <pablo/pabloAST.h>
#include <pablo/symbol_generator.h>
#include <pablo/pe_var.h>
#include <pablo/ps_assign.h>
#include <pablo/pe_count.h>
#include <pablo/pe_zeroes.h>
#include <pablo/pe_ones.h>

namespace pablo {

class Prototype : public PabloAST {
    friend class PabloBlock;
public:

    static inline bool classof(const PabloAST * e) {
        return e->getClassTypeId() == ClassTypeId::Prototype;
    }

    static inline bool classof(void *) {
        return false;
    }

    static Prototype * Create(std::string name, const unsigned numOfParameters, const unsigned numOfResults, void * functionPtr = nullptr);

    const String * getName() const {
        return mName;
    }

    unsigned getNumOfParameters() const {
        return mNumOfParameters;
    }

    unsigned getNumOfResults() const {
        return mNumOfResults;
    }

    void * getFunctionPtr() const {
        return mFunctionPtr;
    }
protected:
    Prototype(const PabloAST::ClassTypeId type, std::string && name, const unsigned numOfParameters, const unsigned numOfResults, void * functionPtr);
protected:
    const String * const    mName;
    const unsigned          mNumOfParameters;
    const unsigned          mNumOfResults;
    void *                  mFunctionPtr;
};

inline Prototype * Prototype::Create(std::string name, const unsigned numOfParameters, const unsigned numOfResults, void * functionPtr) {
    return new Prototype(PabloAST::ClassTypeId::Prototype, std::move(name), numOfParameters, numOfResults, functionPtr);
}

class PabloFunction : public Prototype {
    friend class PabloBlock;
    using Allocator = SlabAllocator<PabloAST *>;
public:

    static inline bool classof(const PabloAST * e) {
        switch (e->getClassTypeId()) {
            case ClassTypeId::Function:
            case ClassTypeId::Prototype:
                return true;
            default:
                return false;
        }        
    }

    static inline bool classof(void *) {
        return false;
    }

    static PabloFunction * Create(std::string name, const unsigned numOfParameters, const unsigned numOfResults);
    
    virtual bool operator==(const PabloAST & other) const {
        return &other == this;
    }

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

    Var * getParameter(const unsigned index) {
        if (LLVM_LIKELY(index < getNumOfParameters()))
            return cast<Var>(mParameters[index]);
        else throwInvalidParameterIndex(index);
    }

    const Var * getParameter(const unsigned index) const {
        if (LLVM_LIKELY(index < getNumOfParameters()))
            return cast<Var>(mParameters[index]);
        else throwInvalidParameterIndex(index);
    }

    void setParameter(const unsigned index, Var * value) {
        if (LLVM_LIKELY(index < getNumOfParameters()))
            mParameters[index] = value;
        else throwInvalidParameterIndex(index);
    }

    Assign * getResult(const unsigned index) {
        if (LLVM_LIKELY(index < getNumOfResults()))
            return cast<Assign>(mResults[index]);
        else throwInvalidResultIndex(index);
    }

    const Assign * getResult(const unsigned index) const {
        if (LLVM_LIKELY(index < getNumOfResults()))
            return cast<Assign>(mResults[index]);
        else throwInvalidResultIndex(index);
    }

    void setResult(const unsigned index, Assign * value) {        
        if (LLVM_LIKELY(index < getNumOfResults())) {
            if (LLVM_LIKELY(mResults[index] != value)) {
                if (LLVM_UNLIKELY(mResults[index] != nullptr)) {
                    mResults[index]->removeUser(this);
                }
                mResults[index] = value;
                value->addUser(this);
            }
        }
        else throwInvalidResultIndex(index);
    }

    void setResultCount(Count * value) {
        value->addUser(this);
    }
    
    void setFunctionPtr(void * functionPtr) {
        mFunctionPtr = functionPtr;
    }

    Zeroes * getNullValue(const PabloType * const type);

    Ones * getAllOnesValue(const PabloType * const type);

    void operator delete (void*);

    virtual ~PabloFunction() { }

    inline SymbolGenerator * getSymbolTable() const {
        return mSymbolTable;
    }

protected:

    __attribute__((noreturn)) void throwInvalidParameterIndex(const unsigned index) const;

    __attribute__((noreturn)) void throwInvalidResultIndex(const unsigned index) const;

    PabloFunction(std::string && name, const unsigned numOfParameters, const unsigned numOfResults);
private:
    SymbolGenerator *                           mSymbolTable;
    const PabloType *                           mBitStreamType;
    PabloBlock *                                mEntryBlock;
    std::vector<PabloAST *, Allocator>          mParameters;
    std::vector<PabloAST *, Allocator>          mResults;
    std::vector<PabloAST *, Allocator>          mConstants;
};

inline PabloFunction * PabloFunction::Create(std::string name, const unsigned numOfParameters, const unsigned numOfResults) {
    return new PabloFunction(std::move(name), numOfParameters, numOfResults);
}
    
}

#endif // FUNCTION_H
