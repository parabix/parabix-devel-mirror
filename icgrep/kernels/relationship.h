#ifndef ABSTRACT_STREAMSET_H
#define ABSTRACT_STREAMSET_H

#include <vector>
#include <memory>
#include <llvm/Support/Compiler.h>
#include <util/not_null.h>
#include <util/slab_allocator.h>

namespace llvm { class Constant; }
namespace llvm { class LLVMContext; }
namespace llvm { class Type; }

namespace kernel {

class Kernel;

// NOTE: Relationships themselves do not store producer/consumer information. When a PipelineKernel is compiled,
// it recalculates the data based on the existence of a relationship. The problem is that internally, a pipeline
// is considered to produce its inputs and consume its outputs whereas a kernel within a pipeline consumes its
// inputs and produces its outputs. However, a PipelineKernel would simply be another kernel if nested within
// another pipeline and it would become more difficult to reason about global buffer requirements if we
// considered them independently. Moreover, maintaining this information only adds additional bookkeeping work
// when the appropriate cached pipeline kernel already exists.

class Relationship {
    friend class Kernel;
    friend class PipelineKernel;
public:

    using Allocator = ProxyAllocator<Relationship *>;

    static inline bool classof(const Relationship *) {
        return true;
    }
    static inline bool classof(const void *) {
        return false;
    }

    enum class ClassTypeId {
        StreamSet
        , Scalar
        , ScalarConstant
    };

    ClassTypeId getClassTypeId() const noexcept {
        return mClassTypeId;
    }

    llvm::Type * getType() const noexcept {
        return mType;
    }

    void* operator new (std::size_t size, Allocator & A) noexcept {
        return A.allocate<uint8_t>(size);
    }

    bool isConstant() const {
        return mClassTypeId == ClassTypeId::ScalarConstant;
    }

protected:

    Relationship(const ClassTypeId typeId, llvm::Type * type) noexcept
    : mClassTypeId(typeId)
    , mType(type) {
    }

protected:
    const ClassTypeId   mClassTypeId;
    llvm::Type * const  mType;
};

class StreamSet : public Relationship {
public:
    static bool classof(const Relationship * e) {
        return e->getClassTypeId() == ClassTypeId::StreamSet;
    }
    static bool classof(const void *) {
        return false;
    }
    LLVM_READNONE unsigned getNumElements() const;

    LLVM_READNONE unsigned getFieldWidth() const;

    StreamSet(llvm::LLVMContext & C, const unsigned NumElements, const unsigned FieldWidth) noexcept;

};

using StreamSets = std::vector<StreamSet *>;

class Scalar : public Relationship {
public:
    static bool classof(const Relationship * e) {
        return e->getClassTypeId() == ClassTypeId::Scalar || e->getClassTypeId() == ClassTypeId::ScalarConstant;;
    }
    static bool classof(const void *) {
        return false;
    }
    unsigned getFieldWidth() const;
    Scalar(not_null<llvm::Type *> type) noexcept;
protected:
    Scalar(const ClassTypeId typeId, llvm::Type *type) noexcept;
};

class ScalarConstant : public Scalar {
public:
    static bool classof(const Relationship * e) {
        return e->getClassTypeId() == ClassTypeId::ScalarConstant;
    }
    static bool classof(const void *) {
        return false;
    }
    llvm::Constant * value() const {
        return mConstant;
    }
    ScalarConstant(not_null<llvm::Constant *> constant) noexcept;
private:
    llvm::Constant * const mConstant;
};

}

#endif // ABSTRACT_STREAMSET_H
