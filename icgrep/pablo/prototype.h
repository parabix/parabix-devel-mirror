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

//    static Prototype * Create(std::string name, const unsigned numOfParameters, const unsigned numOfResults, void * functionPtr = nullptr);

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
    Prototype(const PabloAST::ClassTypeId type, std::string && name, const unsigned numOfParameters, const unsigned numOfResults, Allocator & allocator);
protected:
    const String * const    mName;
    unsigned                mNumOfParameters;
    unsigned                mNumOfResults;
};

//inline Prototype * Prototype::Create(std::string name, const unsigned numOfParameters, const unsigned numOfResults, void *) {
//    return new Prototype(PabloAST::ClassTypeId::Prototype, std::move(name), numOfParameters, numOfResults);
//}

}

#endif // FUNCTION_H
