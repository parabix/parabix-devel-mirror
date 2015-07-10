#include "function.h"
#include <pablo/codegenstate.h>

namespace pablo {

PabloFunction::PabloFunction()
: PabloAST(ClassTypeId::Function)
, mEntryBlock(PabloBlock::Create(mSymbolTable))
, mParameters(reinterpret_cast<ParamAllocator &>(mVectorAllocator))
, mResults(reinterpret_cast<ResultAllocator &>(mVectorAllocator)) {

}

}
