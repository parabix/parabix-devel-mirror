#include "function.h"
#include <pablo/codegenstate.h>
#include <cstring>

namespace pablo {

Prototype::Prototype(const PabloAST::ClassTypeId type, std::string && name, const unsigned numOfParameters, const unsigned numOfResults, void * functionPtr)
: PabloAST(type)
, mName(GlobalSymbolGenerator.get(name, false))
, mNumOfParameters(numOfParameters)
, mNumOfResults(numOfResults)
, mFunctionPtr(functionPtr) {

}

PabloFunction::PabloFunction(std::string && name, const unsigned numOfParameters, const unsigned numOfResults)
: Prototype(ClassTypeId::Function, std::move(name), numOfParameters, numOfResults, nullptr)
, mSymbolTable(new SymbolGenerator())
, mEntryBlock(PabloBlock::Create(*this))
, mParameters(reinterpret_cast<Var **>(mAllocator.allocate(sizeof(Var *) * numOfParameters)))
, mResults(reinterpret_cast<Assign **>(mAllocator.allocate(sizeof(Assign *) * numOfResults))) {
    std::memset(mParameters, 0, sizeof(Var *) * numOfParameters);
    std::memset(mResults, 0, sizeof(Assign *) * numOfResults);
}

void PabloFunction::throwInvalidParameterIndex(const unsigned index) const {
    throw std::runtime_error(
                "Invalid parameter index " +
                std::to_string(index) + " of " + std::to_string(getNumOfParameters()) +
                " in function " + getName()->to_string());
}

void PabloFunction::throwInvalidResultIndex(const unsigned index) const {
    throw std::runtime_error(
                "Invalid result index " +
                std::to_string(index) + " of " + std::to_string(getNumOfResults()) +
                " in function " + getName()->to_string());
}

void PabloFunction::operator delete(void * ptr) {
    PabloFunction * f = static_cast<PabloFunction *>(ptr);
    delete f->mSymbolTable;
    f->mSymbolTable = nullptr;
}

}
