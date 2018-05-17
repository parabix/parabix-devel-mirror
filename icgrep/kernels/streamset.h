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
namespace kernel { class Kernel; }
namespace kernel { class KernelBuilder; }

namespace parabix {
    
class StreamSetBuffer {
    friend class kernel::Kernel;
    friend class kernel::KernelBuilder;
public:

    enum class BufferKind : unsigned {
        ExternalBuffer
        , StaticBuffer
        , DynamicBuffer
    };

    BufferKind getBufferKind() const {
        return mBufferKind;
    }
    
    std::string getUniqueID() const {
        return mUniqueID;
    }

    llvm::Type * getType() const {
        return mType;
    }
    
    llvm::Type * getBaseType() const {
        return mBaseType;
    }

    unsigned getNumOfStreams () const {
        size_t numStreams = 1;
        if (mBaseType->isArrayTy()) {
            numStreams = mBaseType->getArrayNumElements();
        }
        return numStreams;
    }
    
    unsigned getStreamFieldWidth () const {
        if (mBaseType->isArrayTy()) {
            return mBaseType->getArrayElementType()->getScalarSizeInBits();
        }
        return mBaseType->getScalarSizeInBits();
    }
    
    unsigned getAddressSpace() const {
        return mAddressSpace;
    }

    llvm::PointerType * getPointerType() const {
        return getType()->getPointerTo(getAddressSpace());
    }

    virtual bool supportsCopyBack() const {
        return false;
    }

    virtual bool isUnbounded() const {
        return false;
    }

    virtual ~StreamSetBuffer() = 0;

    kernel::Kernel * getProducer() const {
        return mProducer;
    }

    const std::vector<kernel::Kernel *> & getConsumers() const {
        return mConsumers;
    }

    virtual void allocateBuffer(const std::unique_ptr<kernel::KernelBuilder> & kb) = 0;

    virtual void releaseBuffer(const std::unique_ptr<kernel::KernelBuilder> & kb) const = 0;

    llvm::PointerType * getStreamSetPointerType() const {
        return mType->getPointerTo(mAddressSpace);
    }

protected:

    virtual llvm::Value * getStreamBlockPtr(IDISA::IDISA_Builder * const b, llvm::Value * handle, llvm::Value * streamIndex, llvm::Value * blockIndex, const bool readOnly) const;

    virtual llvm::Value * getStreamPackPtr(IDISA::IDISA_Builder * const b, llvm::Value * handle, llvm::Value * streamIndex, llvm::Value * blockIndex, llvm::Value * packIndex, const bool readOnly) const;
    
    virtual llvm::Value * getStreamSetCount(IDISA::IDISA_Builder * const b, llvm::Value * handle) const;

    virtual llvm::Value * getBaseAddress(IDISA::IDISA_Builder * const b, llvm::Value * handle) const = 0;

    virtual void setBaseAddress(IDISA::IDISA_Builder * const b, llvm::Value * handle, llvm::Value * addr) const = 0;

    virtual llvm::Value * getOverflowAddress(IDISA::IDISA_Builder * const b, llvm::Value * handle) const = 0;

    virtual void setCapacity(IDISA::IDISA_Builder * const b, llvm::Value * size, llvm::Value *) const = 0;
    
    virtual llvm::Value * getCapacity(IDISA::IDISA_Builder * const b, llvm::Value * handle) const = 0;

    virtual llvm::Value * getRawItemPointer(IDISA::IDISA_Builder * const b, llvm::Value * handle, llvm::Value * absolutePosition) const = 0;

    // The number of items that cam be linearly accessed from a given logical stream position.
    virtual llvm::Value * getLinearlyAccessibleItems(IDISA::IDISA_Builder * const b, llvm::Value * handle, llvm::Value * fromPos, llvm::Value * avail, bool reverse = false) const = 0;

    virtual llvm::Value * getLinearlyWritableItems(IDISA::IDISA_Builder * const b, llvm::Value * handle, llvm::Value * fromPosition, llvm::Value * consumed, bool reverse = false) const = 0;
    
    void setProducer(kernel::Kernel * const producer) {
        assert (producer);
        mProducer = producer;
    }

    void addConsumer(kernel::Kernel * const consumer) {
        assert (consumer);
        mConsumers.push_back(consumer);
    }

    llvm::Value * getStreamSetHandle() const {
        return mStreamSetHandle;
    }

    StreamSetBuffer(const BufferKind k, const std::unique_ptr<kernel::KernelBuilder> & b, llvm::Type * baseType, unsigned AddressSpace);

protected:
    const BufferKind                 mBufferKind;
    llvm::Type * const               mType;
    const unsigned                   mAddressSpace;
    llvm::Value *                    mStreamSetHandle;
    llvm::Type * const               mBaseType;
    std::string                      mUniqueID;
    kernel::Kernel *                 mProducer;
    std::vector<kernel::Kernel *>    mConsumers;
};   

class ExternalBuffer final : public StreamSetBuffer {
    friend class kernel::KernelBuilder;
public:
    static inline bool classof(const StreamSetBuffer * b) {
        return b->getBufferKind() == BufferKind::ExternalBuffer;
    }

    enum Field {BaseAddress, Capacity};

    ExternalBuffer(const std::unique_ptr<kernel::KernelBuilder> & b, llvm::Type * const type,
                   llvm::Value * const externalAddress = nullptr, const unsigned AddressSpace = 0);

    void allocateBuffer(const std::unique_ptr<kernel::KernelBuilder> & b) override;

    void releaseBuffer(const std::unique_ptr<kernel::KernelBuilder> & kb) const override;

    bool isUnbounded() const override {
        return true;
    }

protected:

    llvm::Value * getBaseAddress(IDISA::IDISA_Builder * const b, llvm::Value * handle) const override;

    void setBaseAddress(IDISA::IDISA_Builder * const b, llvm::Value * handle, llvm::Value * addr) const override;

    llvm::Value * getOverflowAddress(IDISA::IDISA_Builder * const b, llvm::Value * handle) const override;

    void setCapacity(IDISA::IDISA_Builder * const b, llvm::Value * size, llvm::Value * capacity) const override;

    llvm::Value * getCapacity(IDISA::IDISA_Builder * const b, llvm::Value * handle) const override;

    llvm::Value * getRawItemPointer(IDISA::IDISA_Builder * const b, llvm::Value * handle, llvm::Value * absolutePosition) const override;

    llvm::Value * getLinearlyAccessibleItems(IDISA::IDISA_Builder * const b, llvm::Value * handle, llvm::Value * fromPosition, llvm::Value * avail, bool reverse = false) const override;
    
    llvm::Value * getLinearlyWritableItems(IDISA::IDISA_Builder * const b, llvm::Value * handle, llvm::Value * fromPosition, llvm::Value * consumed, bool reverse = false) const override;

private:

    llvm::Value * mExternalAddress;

};

class StaticBuffer final : public StreamSetBuffer {
    friend class kernel::KernelBuilder;
public:
    static inline bool classof(const StreamSetBuffer * b) {
        return b->getBufferKind() == BufferKind::StaticBuffer;
    }
    
    StaticBuffer(const std::unique_ptr<kernel::KernelBuilder> & b, llvm::Type * const type,
                 const size_t capacity, const size_t overflowBlocks = 0, const unsigned AddressSpace = 0);

    void allocateBuffer(const std::unique_ptr<kernel::KernelBuilder> & b) override;

    void releaseBuffer(const std::unique_ptr<kernel::KernelBuilder> & kb) const override;

    bool supportsCopyBack() const override {
        return mOverflow > 0;
    }

protected:

    llvm::Value * getBaseAddress(IDISA::IDISA_Builder * const b, llvm::Value * handle) const override;

    void setBaseAddress(IDISA::IDISA_Builder * const b, llvm::Value * handle, llvm::Value * addr) const override;

    llvm::Value * getOverflowAddress(IDISA::IDISA_Builder * const b, llvm::Value * handle) const override;

    llvm::Value * getCapacity(IDISA::IDISA_Builder * const b, llvm::Value * handle) const override;

    void setCapacity(IDISA::IDISA_Builder * const b, llvm::Value * size, llvm::Value *) const override;

    llvm::Value * getStreamBlockPtr(IDISA::IDISA_Builder * const b, llvm::Value * handle, llvm::Value * streamIndex, llvm::Value * blockIndex, const bool readOnly) const override;

    llvm::Value * getStreamPackPtr(IDISA::IDISA_Builder * const b, llvm::Value * handle, llvm::Value * streamIndex, llvm::Value * blockIndex, llvm::Value * packIndex, const bool readOnly) const override;

    llvm::Value * getRawItemPointer(IDISA::IDISA_Builder * const b, llvm::Value * handle, llvm::Value * absolutePosition) const override;

    llvm::Value * getLinearlyAccessibleItems(IDISA::IDISA_Builder * const b, llvm::Value * handle, llvm::Value * fromPosition, llvm::Value * avail, bool reverse = false) const override;

    llvm::Value * getLinearlyWritableItems(IDISA::IDISA_Builder * const b, llvm::Value * handle, llvm::Value * fromPosition, llvm::Value * consumed, bool reverse = false) const override;

private:

    llvm::Value * modByCapacity(IDISA::IDISA_Builder * const b, llvm::Value * const offset) const;

private:

    const size_t    mCapacity;
    const size_t    mOverflow;

};
        
// Dynamically allocated circular buffers: TODO: add copyback, swizzle support, dynamic allocation, producer, consumer, length
class DynamicBuffer final : public StreamSetBuffer {

    friend class kernel::KernelBuilder;

    enum Field {BaseAddress, PriorBaseAddress, Capacity};

public:
    static inline bool classof(const StreamSetBuffer * b) {return b->getBufferKind() == BufferKind::DynamicBuffer;}
    
    DynamicBuffer(const std::unique_ptr<kernel::KernelBuilder> & b, llvm::Type * type, size_t initialCapacity, size_t overflowBlocks = 0, unsigned AddressSpace = 0);

    void allocateBuffer(const std::unique_ptr<kernel::KernelBuilder> & b) override;

    void releaseBuffer(const std::unique_ptr<kernel::KernelBuilder> & b) const override;
    
    bool supportsCopyBack() const override {
        return mOverflow > 0;
    }

protected:

    llvm::Value * getBaseAddress(IDISA::IDISA_Builder * const b, llvm::Value * handle) const override;

    void setBaseAddress(IDISA::IDISA_Builder * const b, llvm::Value * handle, llvm::Value * addr) const override;

    llvm::Value * getOverflowAddress(IDISA::IDISA_Builder * const b, llvm::Value * handle) const override;
    
    llvm::Value * getCapacity(IDISA::IDISA_Builder * const b, llvm::Value * handle) const override;
    
    void setCapacity(IDISA::IDISA_Builder * const b, llvm::Value * size, llvm::Value *) const override;

    llvm::Value * getBlockAddress(IDISA::IDISA_Builder * const b, llvm::Value * handle, llvm::Value * blockIndex) const;

    llvm::Value * getRawItemPointer(IDISA::IDISA_Builder * const b, llvm::Value * handle, llvm::Value * absolutePosition) const override;

    llvm::Value * getLinearlyAccessibleItems(IDISA::IDISA_Builder * const b, llvm::Value * handle, llvm::Value * fromPosition, llvm::Value * avail, bool reverse = false) const override;

    llvm::Value * getLinearlyWritableItems(IDISA::IDISA_Builder * const b, llvm::Value * handle, llvm::Value * fromPosition, llvm::Value * consumed, bool reverse = false) const override;

private:

    llvm::Value * getAllocationSize(IDISA::IDISA_Builder * const b, llvm::Value * const handle, llvm::Value * const requiredItemCapacity) const;

private:

    const size_t    mInitialCapacity;
    const size_t    mOverflow;

};


}
#endif // STREAMSET_H
