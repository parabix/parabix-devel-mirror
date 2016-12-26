#ifndef STREAMTYPE_H
#define STREAMTYPE_H

#include <llvm/IR/DerivedTypes.h>
#include <llvm/IR/LLVMContext.h>

namespace IDISA {

class IDISA_Builder;

class StreamType : public llvm::Type {
    friend class IDISA_Builder;
public:
    enum {
        StreamTyId = VectorTyID + 1
    };

    unsigned getFieldWidth() const {
        return mFieldWidth;
    }

    llvm::Type * resolveType(IDISA_Builder * const iBuilder);

    /// Methods for support type inquiry through isa, cast, and dyn_cast.
    static inline bool classof(const llvm::Type * type) {
        return type->getTypeID() == (Type::TypeID)(StreamTyId);
    }
    static inline bool classof(const void *) {
        return false;
    }
protected:
    StreamType(llvm::LLVMContext & ctx, unsigned FieldWidth)
    : llvm::Type(ctx, (Type::TypeID)(StreamTyId))
    , mFieldWidth(FieldWidth) {
    }
private:
    unsigned mFieldWidth;
};

}

#endif // STREAMTYPE_H
