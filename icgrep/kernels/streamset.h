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
public:

    enum class BufferKind : unsigned {
        ExternalBuffer
        , StaticBuffer
        , DynamicBuffer
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

    uint64_t getNumOfStreams() const {
        uint64_t numStreams = 1;
        if (mBaseType->isArrayTy()) {
            numStreams = mBaseType->getArrayNumElements();
        }
        return numStreams;
    }

    unsigned getAddressSpace() const {
        return mAddressSpace;
    }

    llvm::PointerType * getPointerType() const {
        return getType()->getPointerTo(getAddressSpace());
    }

    virtual bool hasOverflow() const = 0;

    virtual size_t getOverflowCapacity(const std::unique_ptr<kernel::KernelBuilder> & b) const = 0;

    virtual ~StreamSetBuffer() = 0;

    llvm::Value * getHandle() const {
        return mHandle;
    }

    virtual void allocateBuffer(const std::unique_ptr<KernelBuilder> & b) = 0;

    virtual void releaseBuffer(const std::unique_ptr<KernelBuilder> & b) const = 0;

    // The number of items that cam be linearly accessed from a given logical stream position.
    virtual llvm::Value * getLinearlyAccessibleItems(const std::unique_ptr<KernelBuilder> & b, llvm::Value * fromPosition, llvm::Value * totalItems, llvm::Value * overflowItems = nullptr) const = 0;

    virtual llvm::Value * getLinearlyWritableItems(const std::unique_ptr<KernelBuilder> & b, llvm::Value * fromPosition, llvm::Value * consumedItems, llvm::Value * overflowItems = nullptr) const = 0;

    virtual llvm::Type * getHandleType(const std::unique_ptr<kernel::KernelBuilder> & b) const = 0;

    llvm::PointerType * getHandlePointerType(const std::unique_ptr<kernel::KernelBuilder> & b) const {
        return getHandleType(b)->getPointerTo(getAddressSpace());
    }

    virtual llvm::Value * getStreamBlockPtr(IDISA::IDISA_Builder * const b, llvm::Value * streamIndex, llvm::Value * blockIndex) const;

    virtual llvm::Value * getStreamPackPtr(IDISA::IDISA_Builder * const b, llvm::Value * streamIndex, llvm::Value * blockIndex, llvm::Value * packIndex) const;

    virtual llvm::Value * getStreamSetCount(IDISA::IDISA_Builder * const b) const;

    virtual llvm::Value * getBaseAddress(IDISA::IDISA_Builder * const b) const = 0;

    virtual void setBaseAddress(IDISA::IDISA_Builder * const b, llvm::Value * addr) const = 0;

    virtual llvm::Value * getOverflowAddress(IDISA::IDISA_Builder * const b) const = 0;

    virtual void setCapacity(IDISA::IDISA_Builder * const b, llvm::Value * size) const = 0;

    virtual llvm::Value * getCapacity(IDISA::IDISA_Builder * const b) const = 0;

    virtual llvm::Value * getRawItemPointer(IDISA::IDISA_Builder * const b, llvm::Value * absolutePosition) const = 0;

    virtual llvm::Value * getStreamLogicalBasePtr(IDISA::IDISA_Builder * const b, llvm::Value * const streamIndex, llvm::Value * blockIndex) const = 0;

    void setHandle(const std::unique_ptr<KernelBuilder> & b, llvm::Value * const handle);

protected:

    llvm::Value * getHandle(IDISA::IDISA_Builder * const b) const;

    llvm::Value * addOverflow(const std::unique_ptr<KernelBuilder> & b, llvm::Value * const bufferCapacity, llvm::Value * const overflowItems, llvm::Value * const consumedOffset = nullptr) const;

    StreamSetBuffer(const BufferKind k, const std::unique_ptr<KernelBuilder> & b, llvm::Type * baseType, unsigned AddressSpace);

private:

    void assertValidStreamIndex(IDISA::IDISA_Builder * const b, llvm::Value * streamIndex) const;

protected:

    const BufferKind                mBufferKind;
    // Each StreamSetBuffer object is local to the Kernel (or pipeline) object at (pre-JIT) "compile time" but
    // by sharing the same handle will refer to the same stream set at (post-JIT) run time.
    llvm::Value *                   mHandle;
    llvm::Type * const              mType;
    const unsigned                  mAddressSpace;
    llvm::Type * const              mBaseType;
};

class ExternalBuffer final : public StreamSetBuffer {
public:
    static inline bool classof(const StreamSetBuffer * b) {
        return b->getBufferKind() == BufferKind::ExternalBuffer;
    }

    enum Field {BaseAddress, Capacity};

    ExternalBuffer(const std::unique_ptr<KernelBuilder> & b, llvm::Type * const type, const unsigned AddressSpace = 0);

    void allocateBuffer(const std::unique_ptr<KernelBuilder> & b) override;

    void releaseBuffer(const std::unique_ptr<KernelBuilder> & b) const override;

    llvm::Value * getStreamBlockPtr(IDISA::IDISA_Builder * const b, llvm::Value * streamIndex, llvm::Value * blockIndex) const override;

    llvm::Value * getStreamPackPtr(IDISA::IDISA_Builder * const b, llvm::Value * streamIndex, llvm::Value * blockIndex, llvm::Value * packIndex) const override;

    llvm::Value * getStreamLogicalBasePtr(IDISA::IDISA_Builder * const b, llvm::Value * const streamIndex, llvm::Value * blockIndex) const override;

    llvm::Value * getLinearlyAccessibleItems(const std::unique_ptr<KernelBuilder> & b, llvm::Value * fromPosition, llvm::Value * totalItems, llvm::Value * overflowItems = nullptr) const override;

    llvm::Value * getLinearlyWritableItems(const std::unique_ptr<KernelBuilder> & b, llvm::Value * fromPosition, llvm::Value * consumedItems, llvm::Value * overflowItems = nullptr) const override;

    llvm::Type * getHandleType(const std::unique_ptr<kernel::KernelBuilder> & b) const override;

    llvm::Value * getBaseAddress(IDISA::IDISA_Builder * const b) const override;

    void setCapacity(IDISA::IDISA_Builder * const b, llvm::Value * capacity) const override;

    llvm::Value * getCapacity(IDISA::IDISA_Builder * const b) const override;

    void setBaseAddress(IDISA::IDISA_Builder * const b, llvm::Value * addr) const override;

    virtual bool hasOverflow() const override {
        return false;
    }

    virtual size_t getOverflowCapacity(const std::unique_ptr<kernel::KernelBuilder> & b) const override;

    llvm::Value * getOverflowAddress(IDISA::IDISA_Builder * const b) const override;

    llvm::Value * getRawItemPointer(IDISA::IDISA_Builder * const b, llvm::Value * absolutePosition) const override;

private:

    void assertValidBlockIndex(IDISA::IDISA_Builder * const b, llvm::Value * blockIndex) const;

};

class StaticBuffer final : public StreamSetBuffer {
public:
    static inline bool classof(const StreamSetBuffer * b) {
        return b->getBufferKind() == BufferKind::StaticBuffer;
    }

    StaticBuffer(const std::unique_ptr<KernelBuilder> & b, llvm::Type * const type,
                 const size_t capacity, const size_t overflowBlocks = 0, const unsigned AddressSpace = 0);

    void allocateBuffer(const std::unique_ptr<KernelBuilder> & b) override;

    void releaseBuffer(const std::unique_ptr<KernelBuilder> & b) const override;

    llvm::Value * getLinearlyAccessibleItems(const std::unique_ptr<KernelBuilder> & b, llvm::Value * fromPosition, llvm::Value * const totalItems, llvm::Value * overflowItems = nullptr) const override;

    llvm::Value * getLinearlyWritableItems(const std::unique_ptr<KernelBuilder> & b, llvm::Value * const fromPosition, llvm::Value * const consumedItems, llvm::Value * overflowItems = nullptr) const override;

    bool hasOverflow() const override {
        return mOverflow > 0;
    }

    size_t getOverflowCapacity(const std::unique_ptr<kernel::KernelBuilder> & b) const override;

    llvm::Type * getHandleType(const std::unique_ptr<kernel::KernelBuilder> & b) const override;

    llvm::Value * getBaseAddress(IDISA::IDISA_Builder * const b) const override;

    void setBaseAddress(IDISA::IDISA_Builder * const b, llvm::Value * addr) const override;

    llvm::Value * getOverflowAddress(IDISA::IDISA_Builder * const b) const override;

    llvm::Value * getCapacity(IDISA::IDISA_Builder * const b) const override;

    void setCapacity(IDISA::IDISA_Builder * const b, llvm::Value * capacity) const override;

    llvm::Value * getStreamBlockPtr(IDISA::IDISA_Builder * const b, llvm::Value * streamIndex, llvm::Value * blockIndex) const override;

    llvm::Value * getStreamPackPtr(IDISA::IDISA_Builder * const b, llvm::Value * streamIndex, llvm::Value * blockIndex, llvm::Value * packIndex) const override;

    llvm::Value * getStreamLogicalBasePtr(IDISA::IDISA_Builder * const b, llvm::Value * const streamIndex, llvm::Value * blockIndex) const override;

    llvm::Value * getRawItemPointer(IDISA::IDISA_Builder * const b, llvm::Value * absolutePosition) const override;

private:

    llvm::Value * modByCapacity(IDISA::IDISA_Builder * const b, llvm::Value * const offset) const;

private:

    const size_t    mCapacity;
    const size_t    mOverflow;

};

class DynamicBuffer final : public StreamSetBuffer {

    friend class KernelBuilder;

    enum Field {BaseAddress, PriorBaseAddress, Capacity};

public:
    static inline bool classof(const StreamSetBuffer * b) {return b->getBufferKind() == BufferKind::DynamicBuffer;}

    DynamicBuffer(const std::unique_ptr<KernelBuilder> & b, llvm::Type * type, size_t initialCapacity, size_t overflowSize = 0, unsigned AddressSpace = 0);

    void allocateBuffer(const std::unique_ptr<KernelBuilder> & b) override;

    void releaseBuffer(const std::unique_ptr<KernelBuilder> & b) const override;

    // void expandBuffer(const std::unique_ptr<KernelBuilder> & b, llvm::Value * consumed, llvm::Value * produced, llvm::Value * required) const;

    llvm::Value * getLinearlyAccessibleItems(const std::unique_ptr<KernelBuilder> & b, llvm::Value * fromPosition, llvm::Value * totalItems, llvm::Value * overflowItems = nullptr) const override;

    llvm::Value * getLinearlyWritableItems(const std::unique_ptr<KernelBuilder> & b, llvm::Value * fromPosition, llvm::Value * consumedItems, llvm::Value * overflowItems = nullptr) const override;

    bool hasOverflow() const override {
        return mOverflow > 0;
    }

    size_t getOverflowCapacity(const std::unique_ptr<kernel::KernelBuilder> & b) const override;

protected:

    llvm::Type * getHandleType(const std::unique_ptr<kernel::KernelBuilder> & b) const override;

    llvm::Value * getBaseAddress(IDISA::IDISA_Builder * const b) const override;

    void setBaseAddress(IDISA::IDISA_Builder * const b, llvm::Value * addr) const override;

    llvm::Value * getOverflowAddress(IDISA::IDISA_Builder * const b) const override;

    llvm::Value * getCapacity(IDISA::IDISA_Builder * const b) const override;

    void setCapacity(IDISA::IDISA_Builder * const b, llvm::Value * capacity) const override;

    llvm::Value * getStreamBlockPtr(IDISA::IDISA_Builder * const b, llvm::Value * streamIndex, llvm::Value * blockIndex) const override;

    llvm::Value * getStreamPackPtr(IDISA::IDISA_Builder * const b, llvm::Value * streamIndex, llvm::Value * blockIndex, llvm::Value * packIndex) const override;

    llvm::Value * getStreamLogicalBasePtr(IDISA::IDISA_Builder * const b, llvm::Value * const streamIndex, llvm::Value * blockIndex) const override;

    llvm::Value * getRawItemPointer(IDISA::IDISA_Builder * const b, llvm::Value * absolutePosition) const override;

private:

    llvm::Value * getAllocationSize(IDISA::IDISA_Builder * const b, llvm::Value * const requiredItemCapacity, const size_t overflowSize) const;

    llvm::Value * modByCapacity(IDISA::IDISA_Builder * const b, llvm::Value * const offset) const;

private:

    const size_t    mInitialCapacity;
    const size_t    mOverflow;

};

using StreamSetBuffers = std::vector<StreamSetBuffer *>;
using OwnedStreamSetBuffers = std::vector<std::unique_ptr<StreamSetBuffer>>;

}
#endif // STREAMSET_H
