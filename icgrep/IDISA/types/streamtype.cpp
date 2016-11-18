#include "streamtype.h"
#include <boost/container/flat_map.hpp>
#include <IDISA/idisa_builder.h>

using namespace boost::container;
using namespace llvm;

namespace IDISA {

static flat_map<std::pair<unsigned, unsigned>, StreamType *> STREAM_TYPES;

StreamType * StreamType::get(llvm::LLVMContext & ctx, unsigned NumElements, unsigned FieldWidth) {
    const auto f = STREAM_TYPES.find(std::make_pair(NumElements, FieldWidth));
    if (LLVM_LIKELY(f != STREAM_TYPES.end())) {
        return f->second;
    } else {
        StreamType * const T = new StreamType(ctx, NumElements, FieldWidth);
        STREAM_TYPES.emplace(std::make_pair(NumElements, FieldWidth), T);
        return T;
    }
}

llvm::Type * StreamType::resolveType(IDISA::IDISA_Builder * const iBuilder) {
    return ArrayType::get(iBuilder->getBitBlockType(), mNumElements);
}

StreamType::StreamType(llvm::LLVMContext & C, unsigned NumElements, unsigned FieldWidth)
: llvm::Type(C, (Type::TypeID)(StreamTyId))
, mNumElements(NumElements)
, mFieldWidth(FieldWidth) {

}



}

