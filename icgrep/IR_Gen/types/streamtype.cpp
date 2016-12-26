#include "streamtype.h"
#include "IR_Gen/idisa_builder.h"

namespace IDISA {

llvm::Type * StreamType::resolveType(IDISA_Builder * const builder) {
    if (mFieldWidth == 1) return builder->getBitBlockType();
    return ArrayType::get(builder->getBitBlockType(), mFieldWidth);
//    return llvm::VectorType::get(builder->getIntNTy(mFieldWidth), builder->getBitBlockWidth() / mFieldWidth);
}

}
