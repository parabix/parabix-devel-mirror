#include <pablo/type/streamtype.h>
#include <boost/container/flat_map.hpp>
#include <IDISA/idisa_builder.h>

using namespace boost::container;
using namespace llvm;

namespace pablo {

static flat_map<uint64_t, StreamType *> STREAM_TYPES;

StreamType * StreamType::get(llvm::LLVMContext & ctx, const uint64_t FieldWidth) {
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
    return mFieldWidth == 1 ? iBuilder->getBitBlockType() : ArrayType::get(iBuilder->getBitBlockType(), mFieldWidth);
}

StreamType::StreamType(llvm::LLVMContext & C, uint64_t FieldWidth)
: llvm::Type(C, (Type::TypeID)(StreamTyId))
, mFieldWidth(FieldWidth) {

}



}

