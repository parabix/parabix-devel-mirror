/*
 *  Copyright (c) 2016 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 */

#ifndef STREAMSET_H
#define STREAMSET_H

#include <llvm/IR/Type.h>  // for Type
#include <llvm/IR/DerivedTypes.h>  // for Type
namespace IDISA { class IDISA_Builder; }
namespace llvm { class Value; }
namespace llvm { class Constant; }

namespace kernel {

class Kernel;
class PipelineKernel;
class KernelBuilder;

class StreamSetBuffer {
protected:
    using BuilderRef = const std::unique_ptr<kernel::KernelBuilder> &;
public:

    enum class BufferKind : unsigned {
        ExternalBuffer
        , StaticBuffer
        , DynamicBuffer
        , LinearBuffer
    };

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

    size_t getUnderflowCapacity(BuilderRef b) const;

    size_t getOverflowCapacity(BuilderRef b) const;

    virtual ~StreamSetBuffer() = 0;

    llvm::Value * getHandle() const {
        return mHandle;
    }

    virtual void allocateBuffer(BuilderRef b) = 0;

    virtual void releaseBuffer(BuilderRef b) const = 0;

    // The number of items that cam be linearly accessed from a given logical stream position.
    virtual llvm::Value * getLinearlyAccessibleItems(BuilderRef b, llvm::Value * fromPosition, llvm::Value * totalItems, llvm::Value * overflowItems = nullptr) const = 0;

    virtual llvm::Value * getLinearlyWritableItems(BuilderRef b, llvm::Value * fromPosition, llvm::Value * consumedItems, llvm::Value * overflowItems = nullptr) const = 0;

    virtual llvm::Type * getHandleType(BuilderRef b) const = 0;

    llvm::PointerType * getHandlePointerType(BuilderRef b) const {
        return getHandleType(b)->getPointerTo(getAddressSpace());
    }

    virtual llvm::Value * getStreamBlockPtr(IDISA::IDISA_Builder * const b, llvm::Value * baseAddress, llvm::Value * streamIndex, llvm::Value * blockIndex) const;

    virtual llvm::Value * getStreamPackPtr(IDISA::IDISA_Builder * const b, llvm::Value * baseAddress, llvm::Value * streamIndex, llvm::Value * blockIndex, llvm::Value * packIndex) const;

    virtual llvm::Value * getStreamSetCount(IDISA::IDISA_Builder * const b) const;

    virtual llvm::Value * getBaseAddress(IDISA::IDISA_Builder * const b) const = 0;

    virtual void setBaseAddress(IDISA::IDISA_Builder * const b, llvm::Value * addr) const = 0;

    virtual llvm::Value * getOverflowAddress(IDISA::IDISA_Builder * const b) const = 0;

    virtual void setCapacity(IDISA::IDISA_Builder * const b, llvm::Value * size) const = 0;

    virtual llvm::Value * getCapacity(IDISA::IDISA_Builder * const b) const = 0;

    virtual llvm::Value * getRawItemPointer(IDISA::IDISA_Builder * const b, llvm::Value * streamIndex, llvm::Value * absolutePosition) const;

    virtual llvm::Value * getStreamLogicalBasePtr(IDISA::IDISA_Builder * const b, llvm::Value * baseAddress, llvm::Value * const streamIndex, llvm::Value * blockIndex) const = 0;

    virtual llvm::Value * reserveCapacity(BuilderRef b, llvm::Value * produced, llvm::Value * consumed, llvm::Value * required, llvm::Constant * const overflowItems = nullptr) const = 0;

    void setHandle(BuilderRef b, llvm::Value * const handle);

protected:

    llvm::Value * getHandle(IDISA::IDISA_Builder * const b) const;

    llvm::Value * addOverflow(BuilderRef b, llvm::Value * const bufferCapacity, llvm::Value * const overflowItems, llvm::Value * const consumedOffset) const;

    StreamSetBuffer(const BufferKind k, BuilderRef b, llvm::Type * baseType, const size_t overflowBlocks, const size_t underflowSize, unsigned AddressSpace);

    static llvm::Type * resolveType(BuilderRef b, llvm::Type * const streamSetType);

private:

    void assertValidStreamIndex(IDISA::IDISA_Builder * const b, llvm::Value * streamIndex) const;

protected:

    const BufferKind                mBufferKind;
    // Each StreamSetBuffer object is local to the Kernel (or pipeline) object at (pre-JIT) "compile time" but
    // by sharing the same handle will refer to the same stream set at (post-JIT) run time.
    llvm::Value *                   mHandle;
    llvm::Type * const              mType;
    const unsigned                  mOverflow;
    const unsigned                  mUnderflow;
    const unsigned                  mAddressSpace;
    llvm::Type * const              mBaseType;
};

class ExternalBuffer final : public StreamSetBuffer {
public:
    static inline bool classof(const StreamSetBuffer * b) {
        return b->getBufferKind() == BufferKind::ExternalBuffer;
    }

    enum Field {BaseAddress, Capacity};

    ExternalBuffer(BuilderRef b, llvm::Type * const type, const unsigned AddressSpace = 0);

    void allocateBuffer(BuilderRef b) override;

    void releaseBuffer(BuilderRef b) const override;

    llvm::Value * getStreamLogicalBasePtr(IDISA::IDISA_Builder * const b, llvm::Value * baseAddress, llvm::Value * const streamIndex, llvm::Value * blockIndex) const override;

    llvm::Value * getLinearlyAccessibleItems(BuilderRef b, llvm::Value * fromPosition, llvm::Value * totalItems, llvm::Value * overflowItems = nullptr) const override;

    llvm::Value * getLinearlyWritableItems(BuilderRef b, llvm::Value * fromPosition, llvm::Value * consumedItems, llvm::Value * overflowItems = nullptr) const override;

    llvm::Type * getHandleType(BuilderRef b) const override;

    llvm::Value * getBaseAddress(IDISA::IDISA_Builder * const b) const override;

    void setCapacity(IDISA::IDISA_Builder * const b, llvm::Value * capacity) const override;

    llvm::Value * getCapacity(IDISA::IDISA_Builder * const b) const override;

    llvm::Value * reserveCapacity(BuilderRef b, llvm::Value * produced, llvm::Value * consumed, llvm::Value * required, llvm::Constant * const overflowItems = nullptr) const override;

    void setBaseAddress(IDISA::IDISA_Builder * const b, llvm::Value * addr) const override;

    llvm::Value * getOverflowAddress(IDISA::IDISA_Builder * const b) const override;

private:

    void assertValidBlockIndex(IDISA::IDISA_Builder * const b, llvm::Value * blockIndex) const;

};

class StaticBuffer final : public StreamSetBuffer {
public:
    static inline bool classof(const StreamSetBuffer * b) {
        return b->getBufferKind() == BufferKind::StaticBuffer;
    }

    StaticBuffer(BuilderRef b, llvm::Type * const type,
                 const size_t capacity, const size_t overflowBlocks, const size_t underflowSize, const unsigned AddressSpace);

    void allocateBuffer(BuilderRef b) override;

    void releaseBuffer(BuilderRef b) const override;

    llvm::Value * getLinearlyAccessibleItems(BuilderRef b, llvm::Value * fromPosition, llvm::Value * const totalItems, llvm::Value * overflowItems = nullptr) const override;

    llvm::Value * getLinearlyWritableItems(BuilderRef b, llvm::Value * const fromPosition, llvm::Value * const consumedItems, llvm::Value * overflowItems = nullptr) const override;

    llvm::Type * getHandleType(BuilderRef b) const override;

    llvm::Value * getBaseAddress(IDISA::IDISA_Builder * const b) const override;

    void setBaseAddress(IDISA::IDISA_Builder * const b, llvm::Value * addr) const override;

    llvm::Value * getOverflowAddress(IDISA::IDISA_Builder * const b) const override;

    llvm::Value * getCapacity(IDISA::IDISA_Builder * const b) const override;

    void setCapacity(IDISA::IDISA_Builder * const b, llvm::Value * capacity) const override;

    llvm::Value * reserveCapacity(BuilderRef b, llvm::Value * produced, llvm::Value * consumed, llvm::Value * required, llvm::Constant * const overflowItems = nullptr) const override;

    llvm::Value * getStreamBlockPtr(IDISA::IDISA_Builder * const b, llvm::Value * baseAddress, llvm::Value * streamIndex, llvm::Value * blockIndex) const override;

    llvm::Value * getStreamPackPtr(IDISA::IDISA_Builder * const b, llvm::Value * baseAddress, llvm::Value * streamIndex, llvm::Value * blockIndex, llvm::Value * packIndex) const override;

    llvm::Value * getStreamLogicalBasePtr(IDISA::IDISA_Builder * const b, llvm::Value * baseAddress, llvm::Value * const streamIndex, llvm::Value * blockIndex) const override;

    llvm::Value * getRawItemPointer(IDISA::IDISA_Builder * const b, llvm::Value * streamIndex, llvm::Value * absolutePosition) const override;

    size_t getCapacity() const {
        return mCapacity;
    }

private:

    llvm::Value * modByCapacity(IDISA::IDISA_Builder * const b, llvm::Value * const offset) const;

private:

    const size_t    mCapacity;

};

class DynamicBuffer final : public StreamSetBuffer {

    friend class KernelBuilder;

    enum Field {BaseAddress, Capacity, PriorBaseAddress};

public:

    static inline bool classof(const StreamSetBuffer * b) {return b->getBufferKind() == BufferKind::DynamicBuffer;}

    DynamicBuffer(BuilderRef b, llvm::Type * type, const size_t initialCapacity,
                  const size_t overflowSize, const size_t underflowSize, const unsigned AddressSpace);

    void allocateBuffer(BuilderRef b) override;

    void releaseBuffer(BuilderRef b) const override;

    llvm::Value * reserveCapacity(BuilderRef b, llvm::Value * produced, llvm::Value * consumed, llvm::Value * required, llvm::Constant * const overflowItems = nullptr) const override;

    llvm::Value * getLinearlyAccessibleItems(BuilderRef b, llvm::Value * fromPosition, llvm::Value * totalItems, llvm::Value * overflowItems = nullptr) const override;

    llvm::Value * getLinearlyWritableItems(BuilderRef b, llvm::Value * fromPosition, llvm::Value * consumedItems, llvm::Value * overflowItems = nullptr) const override;

    size_t getInitialCapacity() const {
        return mInitialCapacity;
    }

protected:

    llvm::Type * getHandleType(BuilderRef b) const override;

    llvm::Value * getBaseAddress(IDISA::IDISA_Builder * const b) const override;

    void setBaseAddress(IDISA::IDISA_Builder * const b, llvm::Value * addr) const override;

    llvm::Value * getOverflowAddress(IDISA::IDISA_Builder * const b) const override;

    llvm::Value * getCapacity(IDISA::IDISA_Builder * const b) const override;

    void setCapacity(IDISA::IDISA_Builder * const b, llvm::Value * capacity) const override;

    llvm::Value * getStreamBlockPtr(IDISA::IDISA_Builder * const b, llvm::Value * baseAddress, llvm::Value * streamIndex, llvm::Value * blockIndex) const override;

    llvm::Value * getStreamPackPtr(IDISA::IDISA_Builder * const b, llvm::Value * baseAddress, llvm::Value * streamIndex, llvm::Value * blockIndex, llvm::Value * packIndex) const override;

    llvm::Value * getStreamLogicalBasePtr(IDISA::IDISA_Builder * const b, llvm::Value * baseAddress, llvm::Value * const streamIndex, llvm::Value * blockIndex) const override;

    llvm::Value * getRawItemPointer(IDISA::IDISA_Builder * const b, llvm::Value * streamIndex, llvm::Value * absolutePosition) const override;

private:

    llvm::Value * modByCapacity(IDISA::IDISA_Builder * const b, llvm::Value * const offset) const;

private:

    const size_t    mInitialCapacity;

};

class LinearBuffer final : public StreamSetBuffer {

    friend class KernelBuilder;

    enum Field {ReportedAddress, FirstCapacity, FirstBufferAddress, SecondCapacity, SecondBufferAddress};

public:

    static inline bool classof(const StreamSetBuffer * b) {return b->getBufferKind() == BufferKind::LinearBuffer;}

    LinearBuffer(BuilderRef b, llvm::Type * type, const size_t initialCapacity,
                 const size_t overflowSize, const size_t underflowSize, const unsigned AddressSpace);

    void allocateBuffer(BuilderRef b) override;

    void releaseBuffer(BuilderRef b) const override;

    llvm::Value * reserveCapacity(BuilderRef b, llvm::Value * produced, llvm::Value * consumed, llvm::Value * required, llvm::Constant * const overflowItems = nullptr) const override;

    llvm::Value * getLinearlyAccessibleItems(BuilderRef b, llvm::Value * fromPosition, llvm::Value * totalItems, llvm::Value * overflowItems = nullptr) const override;

    llvm::Value * getLinearlyWritableItems(BuilderRef b, llvm::Value * fromPosition, llvm::Value * consumedItems, llvm::Value * overflowItems = nullptr) const override;

    size_t getInitialCapacity() const {
        return mInitialCapacity;
    }

protected:

    llvm::Type * getHandleType(BuilderRef b) const override;

    llvm::Value * getBaseAddress(IDISA::IDISA_Builder * const b) const override;

    void setBaseAddress(IDISA::IDISA_Builder * const b, llvm::Value * addr) const override;

    llvm::Value * getStreamLogicalBasePtr(IDISA::IDISA_Builder * const b, llvm::Value * baseAddress, llvm::Value * const streamIndex, llvm::Value * blockIndex) const override;

    llvm::Value * getOverflowAddress(IDISA::IDISA_Builder * const b) const override;

    llvm::Value * getCapacity(IDISA::IDISA_Builder * const b) const override;

    void setCapacity(IDISA::IDISA_Builder * const b, llvm::Value * capacity) const override;

private:

    const size_t    mInitialCapacity;

};

using StreamSetBuffers = std::vector<StreamSetBuffer *>;
using OwnedStreamSetBuffers = std::vector<std::unique_ptr<StreamSetBuffer>>;

}
#endif // STREAMSET_H
