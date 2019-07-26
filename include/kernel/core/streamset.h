/*
 *  Copyright (c) 2016 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 */

#ifndef STREAMSET_H
#define STREAMSET_H

#include <llvm/IR/Type.h>  // for Type
#include <llvm/IR/DerivedTypes.h>  // for Type
#include <kernel/core/ptrwrapper.hpp>

namespace IDISA { class IDISA_Builder; }
namespace llvm { class Value; }
namespace llvm { class Constant; }

namespace kernel {

class Kernel;
class PipelineKernel;
class KernelBuilder;

class StreamSetBuffer {
public:

    enum class BufferKind : unsigned {
        ExternalBuffer
        , StaticBuffer
        , DynamicBuffer
    };

    using BuilderPtr = PtrWrapper<kernel::KernelBuilder>;

    BufferKind getBufferKind() const {
        return mBufferKind;
    }

    llvm::Type * getType() const {
        return mType;
    }

    llvm::Type * getBaseType() const {
        return mBaseType;
    }

    unsigned getAddressSpace() const {
        return mAddressSpace;
    }

    llvm::PointerType * getPointerType() const {
        return getType()->getPointerTo(getAddressSpace());
    }

    bool hasOverflow() const {
        return mOverflow != 0;
    }

    unsigned getOverflow() const {
        return mOverflow;
    }

    bool hasUnderflow() const {
        return mUnderflow != 0;
    }

    unsigned getUnderflow() const {
        return mUnderflow;
    }

    bool isLinear() const {
        return mLinear;
    }

    size_t getUnderflowCapacity(BuilderPtr b) const;

    size_t getOverflowCapacity(BuilderPtr b) const;

    virtual ~StreamSetBuffer() = 0;

    llvm::Value * getHandle() const {
        return mHandle;
    }

    virtual void allocateBuffer(BuilderPtr b) = 0;

    virtual void releaseBuffer(BuilderPtr b) const = 0;

    // The number of items that cam be linearly accessed from a given logical stream position.
    virtual llvm::Value * getLinearlyAccessibleItems(BuilderPtr b, llvm::Value * fromPosition, llvm::Value * totalItems, llvm::Value * overflowItems = nullptr) const = 0;

    virtual llvm::Value * getLinearlyWritableItems(BuilderPtr b, llvm::Value * fromPosition, llvm::Value * consumedItems, llvm::Value * overflowItems = nullptr) const = 0;

    virtual llvm::Type * getHandleType(BuilderPtr b) const = 0;

    llvm::PointerType * getHandlePointerType(BuilderPtr b) const {
        return getHandleType(b)->getPointerTo(getAddressSpace());
    }

    virtual llvm::Value * getStreamBlockPtr(BuilderPtr b, llvm::Value * baseAddress, llvm::Value * streamIndex, llvm::Value * blockIndex) const;

    virtual llvm::Value * getStreamPackPtr(BuilderPtr b, llvm::Value * baseAddress, llvm::Value * streamIndex, llvm::Value * blockIndex, llvm::Value * packIndex) const;

    virtual llvm::Value * getStreamSetCount(BuilderPtr b) const;

    virtual llvm::Value * getBaseAddress(BuilderPtr b) const = 0;

    virtual void setBaseAddress(BuilderPtr b, llvm::Value * addr) const = 0;

    virtual llvm::Value * getOverflowAddress(BuilderPtr b) const = 0;

    virtual void setCapacity(BuilderPtr b, llvm::Value * size) const = 0;

    virtual llvm::Value * getCapacity(BuilderPtr b) const = 0;

    virtual llvm::Value * getRawItemPointer(BuilderPtr b, llvm::Value * streamIndex, llvm::Value * absolutePosition) const;

    virtual llvm::Value * getStreamLogicalBasePtr(BuilderPtr b, llvm::Value * baseAddress, llvm::Value * const streamIndex, llvm::Value * blockIndex) const = 0;

    virtual llvm::Value * reserveCapacity(BuilderPtr b, llvm::Value * produced, llvm::Value * consumed, llvm::Value * required, llvm::Constant * const overflowItems) const = 0;

    void setHandle(BuilderPtr b, llvm::Value * const handle) const;

protected:

    llvm::Value * getHandle(BuilderPtr b) const;

    llvm::Value * addOverflow(BuilderPtr b, llvm::Value * const bufferCapacity, llvm::Value * const overflowItems, llvm::Value * const consumedOffset) const;

    StreamSetBuffer(const BufferKind k, BuilderPtr b, llvm::Type * baseType, const size_t overflowBlocks, const size_t underflowSize, const bool linear, const unsigned AddressSpace);

    static llvm::Type * resolveType(BuilderPtr b, llvm::Type * const streamSetType);

private:

    void assertValidStreamIndex(BuilderPtr b, llvm::Value * streamIndex) const;

protected:

    const BufferKind                mBufferKind;
    // Each StreamSetBuffer object is local to the Kernel (or pipeline) object at (pre-JIT) "compile time" but
    // by sharing the same handle will refer to the same stream set at (post-JIT) run time.
    mutable llvm::Value *           mHandle;
    llvm::Type * const              mType;
    const unsigned                  mOverflow;
    const unsigned                  mUnderflow;
    const unsigned                  mAddressSpace;
    llvm::Type * const              mBaseType;
    const bool                      mLinear;
};

class ExternalBuffer final : public StreamSetBuffer {
public:
    static inline bool classof(const StreamSetBuffer * b) {
        return b->getBufferKind() == BufferKind::ExternalBuffer;
    }

    enum Field {BaseAddress, Capacity};

    ExternalBuffer(BuilderPtr b, llvm::Type * const type, const bool linear, const unsigned AddressSpace);

    void allocateBuffer(BuilderPtr b) override;

    void releaseBuffer(BuilderPtr b) const override;

    llvm::Value * getStreamLogicalBasePtr(BuilderPtr b, llvm::Value * baseAddress, llvm::Value * const streamIndex, llvm::Value * blockIndex) const override;

    llvm::Value * getLinearlyAccessibleItems(BuilderPtr b, llvm::Value * fromPosition, llvm::Value * totalItems, llvm::Value * overflowItems = nullptr) const override;

    llvm::Value * getLinearlyWritableItems(BuilderPtr b, llvm::Value * fromPosition, llvm::Value * consumedItems, llvm::Value * overflowItems = nullptr) const override;

    llvm::Type * getHandleType(BuilderPtr b) const override;

    llvm::Value * getBaseAddress(BuilderPtr b) const override;

    void setCapacity(BuilderPtr b, llvm::Value * capacity) const override;

    llvm::Value * getCapacity(BuilderPtr b) const override;

    llvm::Value * reserveCapacity(BuilderPtr b, llvm::Value * produced, llvm::Value * consumed, llvm::Value * required, llvm::Constant * const overflowItems) const override;

    void setBaseAddress(BuilderPtr b, llvm::Value * addr) const override;

    llvm::Value * getOverflowAddress(BuilderPtr b) const override;

private:

    void assertValidBlockIndex(BuilderPtr b, llvm::Value * blockIndex) const;

};

class InternalBuffer : public StreamSetBuffer {
public:

    static inline bool classof(const StreamSetBuffer * b) {
        return b->getBufferKind() != BufferKind::ExternalBuffer;
    }

    llvm::Value * getStreamBlockPtr(BuilderPtr b, llvm::Value * baseAddress, llvm::Value * streamIndex, llvm::Value * blockIndex) const final;

    llvm::Value * getStreamPackPtr(BuilderPtr b, llvm::Value * baseAddress, llvm::Value * streamIndex, llvm::Value * blockIndex, llvm::Value * packIndex) const final;

    llvm::Value * getStreamLogicalBasePtr(BuilderPtr b, llvm::Value * baseAddress, llvm::Value * const streamIndex, llvm::Value * blockIndex) const final;

    llvm::Value * getRawItemPointer(BuilderPtr b, llvm::Value * streamIndex, llvm::Value * absolutePosition) const final;

    llvm::Value * getLinearlyAccessibleItems(BuilderPtr b, llvm::Value * fromPosition, llvm::Value * const totalItems, llvm::Value * overflowItems = nullptr) const final;

    llvm::Value * getLinearlyWritableItems(BuilderPtr b, llvm::Value * const fromPosition, llvm::Value * const consumedItems, llvm::Value * overflowItems = nullptr) const final;

protected:

    InternalBuffer(const BufferKind k, BuilderPtr b, llvm::Type * baseType,
                   const size_t overflowBlocks, const size_t underflowSize,
                   const bool linear, const unsigned AddressSpace);


    virtual llvm::Value * modByCapacity(BuilderPtr b, llvm::Value * const offset) const = 0;

};

class StaticBuffer final : public InternalBuffer {
public:
    static inline bool classof(const StreamSetBuffer * b) {
        return b->getBufferKind() == BufferKind::StaticBuffer;
    }

    StaticBuffer(BuilderPtr b, llvm::Type * const type,
                 const size_t capacity, const size_t overflowBlocks, const size_t underflowSize,
                 const bool linear, const unsigned AddressSpace);

    void allocateBuffer(BuilderPtr b) override;

    void releaseBuffer(BuilderPtr b) const override;

    llvm::Type * getHandleType(BuilderPtr b) const override;

    llvm::Value * getBaseAddress(BuilderPtr b) const override;

    void setBaseAddress(BuilderPtr b, llvm::Value * addr) const override;

    llvm::Value * getOverflowAddress(BuilderPtr b) const override;

    llvm::Value * getCapacity(BuilderPtr b) const override;

    void setCapacity(BuilderPtr b, llvm::Value * capacity) const override;

    llvm::Value * reserveCapacity(BuilderPtr b, llvm::Value * produced, llvm::Value * consumed, llvm::Value * required, llvm::Constant * const overflowItems) const override;

    size_t getCapacity() const {
        return mCapacity;
    }

private:

    llvm::Value * modByCapacity(BuilderPtr b, llvm::Value * const offset) const final;

private:

    const size_t    mCapacity;

};

class DynamicBuffer final : public InternalBuffer {

    friend class KernelBuilder;

    enum Field {BaseAddress, Capacity, PriorBaseAddress};

public:

    static inline bool classof(const StreamSetBuffer * b) {
        return b->getBufferKind() == BufferKind::DynamicBuffer;
    }

    DynamicBuffer(BuilderPtr b, llvm::Type * type, const size_t initialCapacity,
                  const size_t overflowSize, const size_t underflowSize,
                  const bool linear, const unsigned AddressSpace);

    void allocateBuffer(BuilderPtr b) override;

    void releaseBuffer(BuilderPtr b) const override;

    llvm::Value * reserveCapacity(BuilderPtr b, llvm::Value * produced, llvm::Value * consumed, llvm::Value * required, llvm::Constant * const overflowItems) const override;

    size_t getInitialCapacity() const {
        return mInitialCapacity;
    }

protected:

    llvm::Type * getHandleType(BuilderPtr b) const override;

    llvm::Value * getBaseAddress(BuilderPtr b) const override;

    void setBaseAddress(BuilderPtr b, llvm::Value * addr) const override;

    llvm::Value * getOverflowAddress(BuilderPtr b) const override;

    llvm::Value * getCapacity(BuilderPtr b) const override;

    void setCapacity(BuilderPtr b, llvm::Value * capacity) const override;

private:

    llvm::Value * modByCapacity(BuilderPtr b, llvm::Value * const offset) const final;

private:

    const size_t    mInitialCapacity;

};

using StreamSetBuffers = std::vector<StreamSetBuffer *>;
using OwnedStreamSetBuffers = std::vector<std::unique_ptr<StreamSetBuffer>>;

}
#endif // STREAMSET_H
