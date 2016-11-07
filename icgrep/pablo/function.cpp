#include "function.h"
#include <pablo/codegenstate.h>
#include <cstring>

namespace pablo {

Prototype::Prototype(const PabloAST::ClassTypeId type, std::string && name, const unsigned numOfParameters, const unsigned numOfResults)
: PabloAST(type, nullptr, nullptr)
, mName(GlobalSymbolGenerator.get(name))
, mNumOfParameters(numOfParameters)
, mNumOfResults(numOfResults) {

}

PabloFunction::PabloFunction(std::string && name)
: Prototype(ClassTypeId::Function, std::move(name), 0, 0)
, mSymbolTable(new SymbolGenerator())
, mEntryBlock(PabloBlock::Create(*this))
, mConstants(0, nullptr) {

}

Var * PabloFunction::addParameter(const std::string name, Type * const type) {
    Var * param = new Var(mSymbolTable->make(name), type);
    mParameters.push_back(param);
    mNumOfParameters = mParameters.size();
    return param;
}

Var * PabloFunction::addResult(const std::string name, Type * const type) {
    Var * result = new Var(mSymbolTable->make(name), type);
    mResults.push_back(result);
    mNumOfResults = mResults.size();
    return result;
}

Var * PabloFunction::makeVariable(PabloAST * name, Type * const type) {
    Var * const var = new Var(name, type);
    mVariables.push_back(var);
    return var;
}

Zeroes * PabloFunction::getNullValue(Type * type) {
    if (type == nullptr) {
        type = getStreamTy();
    }
    for (PabloAST * constant : mConstants) {
        if (isa<Zeroes>(constant) && constant->getType() == type) {
            return cast<Zeroes>(constant);
        }
    }
    Zeroes * value = new Zeroes(type);
    mConstants.push_back(value);
    return value;
}

Ones * PabloFunction::getAllOnesValue(Type * type) {
    if (type == nullptr) {
        type = getStreamTy();
    }
    for (PabloAST * constant : mConstants) {
        if (isa<Ones>(constant) && constant->getType() == type) {
            return cast<Ones>(constant);
        }
    }
    Ones * value = new Ones(type);
    mConstants.push_back(value);
    return value;
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
