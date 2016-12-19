#include "prototype.h"
#include <pablo/codegenstate.h>
#include <cstring>

namespace pablo {

Prototype::Prototype(const PabloAST::ClassTypeId type, std::string && name, const unsigned numOfParameters, const unsigned numOfResults, Allocator &allocator)
: PabloAST(type, nullptr, nullptr, allocator)
, mName(GlobalSymbolGenerator.get(name))
, mNumOfParameters(numOfParameters)
, mNumOfResults(numOfResults) {

}

}
