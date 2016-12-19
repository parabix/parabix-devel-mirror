#include "streamtype.h"
#include <boost/container/flat_map.hpp>
#include <IDISA/idisa_builder.h>

using namespace boost::container;
using namespace llvm;

namespace IDISA {

static flat_map<unsigned, StreamType *> STREAM_TYPES;

StreamType * StreamType::get(llvm::LLVMContext & ctx, unsigned FieldWidth) {
    const auto f = STREAM_TYPES.find(FieldWidth);
    if (LLVM_LIKELY(f != STREAM_TYPES.end())) {
        return f->second;
    } else {
        StreamType * const T = new StreamType(ctx, FieldWidth);
        STREAM_TYPES.emplace(FieldWidth, T);
        return T;
    }
}

llvm::Type * StreamType::resolveType(IDISA::IDISA_Builder * const iBuilder) {
    if (mFieldWidth == 1) return iBuilder->getBitBlockType();
    return ArrayType::get(iBuilder->getBitBlockType(), mFieldWidth);
}

StreamType::StreamType(llvm::LLVMContext & C, unsigned FieldWidth)
: llvm::Type(C, (Type::TypeID)(StreamTyId))
, mFieldWidth(FieldWidth) {

}



}

