#ifndef STREAMTYPE_H
#define STREAMTYPE_H

#include <llvm/IR/DerivedTypes.h>
#include <llvm/IR/LLVMContext.h>

namespace IDISA {

class IDISA_Builder;

class StreamType : public llvm::Type {

    StreamType(llvm::LLVMContext & ctx, unsigned FieldWidth);

public:

    enum {
        StreamTyId = VectorTyID + 1
    };

    static StreamType * get(llvm::LLVMContext & ctx, unsigned FieldWidth);

    unsigned getFieldWidth() const {
        return mFieldWidth;
    }

    StreamType * getStreamElementType() const {
        return get(getContext(), mFieldWidth);
    }

    llvm::Type * resolveType(IDISA_Builder * const iBuilder);

    /// Methods for support type inquiry through isa, cast, and dyn_cast.
    static inline bool classof(const llvm::Type * type) {
        return type->getTypeID() == (Type::TypeID)(StreamTyId);
    }
    static inline bool classof(const void *) {
        return false;
    }
private:
    unsigned mFieldWidth;
};

}

#endif // STREAMTYPE_H
