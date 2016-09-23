#include "function.h"
#include <pablo/codegenstate.h>
#include <cstring>

namespace pablo {

Prototype::Prototype(const PabloAST::ClassTypeId type, std::string && name, const unsigned numOfParameters, const unsigned numOfResults, void * functionPtr)
: PabloAST(type, nullptr)
, mName(GlobalSymbolGenerator.get(name, false))
, mNumOfParameters(numOfParameters)
, mNumOfResults(numOfResults)
, mFunctionPtr(functionPtr) {

}

PabloFunction::PabloFunction(std::string && name, const unsigned numOfParameters, const unsigned numOfResults)
: Prototype(ClassTypeId::Function, std::move(name), numOfParameters, numOfResults, nullptr)
, mSymbolTable(new SymbolGenerator())
, mBitStreamType(getPabloType(PabloType::Stream, 1))
, mEntryBlock(PabloBlock::Create(*this))
, mParameters(numOfParameters, nullptr)
, mResults(numOfResults, nullptr)
, mConstants(0, nullptr) {

}

Zeroes * PabloFunction::getNullValue(const PabloType * type) {
    if (type == nullptr) {
        type = mBitStreamType;
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

Ones * PabloFunction::getAllOnesValue(const PabloType * type) {
    if (type == nullptr) {
        type = mBitStreamType;
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
