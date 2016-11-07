#ifndef STREAMTYPE_H
#define STREAMTYPE_H

#include <llvm/IR/DerivedTypes.h>
#include <llvm/IR/LLVMContext.h>

namespace IDISA {
    class IDISA_Builder;
}

namespace pablo {

class StreamType : public llvm::Type {

    StreamType(llvm::LLVMContext & ctx, uint64_t FieldWidth);

public:

    enum {
        StreamTyId = VectorTyID + 1
    };

    static StreamType * get(llvm::LLVMContext & ctx, uint64_t FieldWidth);

    llvm::Type * resolveType(IDISA::IDISA_Builder * const iBuilder);

    /// Methods for support type inquiry through isa, cast, and dyn_cast.
    static inline bool classof(const llvm::Type * type) {
        return type->getTypeID() == (Type::TypeID)(StreamTyId);
    }
    static inline bool classof(const void *) {
        return false;
    }
private:

    uint64_t mFieldWidth;
    uint64_t mNumElements;

};

}

inline llvm::Type * getStreamTy(const uint64_t FieldWidth = 1, const uint64_t NumElements = 1) {
    llvm::Type * ty = pablo::StreamType::get(llvm::getGlobalContext(), FieldWidth);
    if (NumElements > 1) {
        ty = llvm::ArrayType::get(ty, NumElements);
    }
    return ty;
}

inline llvm::Type * getScalarTy(const uint64_t FieldWidth = 0) {
    return llvm::Type::getIntNTy(llvm::getGlobalContext(), FieldWidth);
}

#endif // STREAMTYPE_H
