#include "streamtype.h"
#include <IR_Gen/idisa_builder.h>  // for IDISA_Builder
#include <llvm/IR/DerivedTypes.h>  // for ArrayType, VectorType

namespace IDISA {

llvm::Type * StreamType::resolveType(IDISA_Builder * const builder) const {
    if (mFieldWidth == 1) return builder->getBitBlockType();
    return llvm::ArrayType::get(builder->getBitBlockType(), mFieldWidth);
//    return llvm::VectorType::get(builder->getIntNTy(mFieldWidth), builder->getBitBlockWidth() / mFieldWidth);
}

}
