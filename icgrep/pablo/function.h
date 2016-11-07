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

protected:
    Prototype(const PabloAST::ClassTypeId type, std::string && name, const unsigned numOfParameters, const unsigned numOfResults);
protected:
    const String * const    mName;
    unsigned                mNumOfParameters;
    unsigned                mNumOfResults;
};

inline Prototype * Prototype::Create(std::string name, const unsigned numOfParameters, const unsigned numOfResults, void *) {
    return new Prototype(PabloAST::ClassTypeId::Prototype, std::move(name), numOfParameters, numOfResults);
}

class PabloFunction : public Prototype {
    friend class PabloBlock;
    friend class Branch;
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

    static PabloFunction * Create(std::string name);
    
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
        return static_cast<Var *>(mParameters[index]);
    }

    const Var * getParameter(const unsigned index) const {
        return static_cast<Var *>(mParameters[index]);
    }

    Var * addParameter(const std::string name, Type * const type);

    Var * getResult(const unsigned index) {
        return static_cast<Var *>(mResults[index]);
    }

    const Var * getResult(const unsigned index) const {
        return static_cast<Var *>(mResults[index]);
    }

    Var * addResult(const std::string name, Type * const type);

    Var * makeVariable(PabloAST * name, Type * const type);

    Var * getVariable(const unsigned index) {
        return static_cast<Var *>(mVariables[index]);
    }

    unsigned getNumOfVariables() {
        return mVariables.size();
    }

    Zeroes * getNullValue(Type * const type);

    Ones * getAllOnesValue(Type * const type);

    void operator delete (void*);

    virtual ~PabloFunction() { }

    inline SymbolGenerator * getSymbolTable() const {
        return mSymbolTable;
    }

protected:

    __attribute__((noreturn)) void throwInvalidParameterIndex(const unsigned index) const;

    __attribute__((noreturn)) void throwInvalidResultIndex(const unsigned index) const;

    PabloFunction(std::string && name);
private:
    SymbolGenerator *                           mSymbolTable;
    PabloBlock *                                mEntryBlock;
    std::vector<PabloAST *, Allocator>          mParameters;
    std::vector<PabloAST *, Allocator>          mResults;
    std::vector<PabloAST *, Allocator>          mConstants;
    std::vector<PabloAST *, Allocator>          mVariables;
};

inline PabloFunction * PabloFunction::Create(std::string name) {
    return new PabloFunction(std::move(name));
}
    
}

#endif // FUNCTION_H
