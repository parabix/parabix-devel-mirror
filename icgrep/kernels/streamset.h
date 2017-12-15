/*
 *  Copyright (c) 2016 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 */

#ifndef STREAMSET_H
#define STREAMSET_H

#include <llvm/IR/Type.h>  // for Type
namespace IDISA { class IDISA_Builder; }
namespace llvm { class Value; }
namespace kernel { class Kernel; }
namespace kernel { class KernelBuilder; }

namespace parabix {
    
class StreamSetBuffer {
    friend class kernel::Kernel;
    friend class kernel::KernelBuilder;
public:

    enum class BufferKind : unsigned {
        SourceBuffer
        , ExternalBuffer
        , CircularBuffer
        , CircularCopybackBuffer
        , SwizzledCopybackBuffer
        , ExpandableBuffer
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

    unsigned getAddressSpace() const {
        return mAddressSpace;
    }

    llvm::PointerType * getPointerType() const {
        return getType()->getPointerTo(getAddressSpace());
    }

    size_t getBufferBlocks() const {
        return mBufferBlocks;
    }

    llvm::Value * getStreamSetHandle() const {
        return mStreamSetBufferPtr;
    }
    
    virtual llvm::Type * getStreamSetBlockType() const;
    
    virtual void allocateBuffer(const std::unique_ptr<kernel::KernelBuilder> & kb);

    virtual void releaseBuffer(const std::unique_ptr<kernel::KernelBuilder> & kb) const;

    virtual llvm::Value * getStreamBlockPtr(IDISA::IDISA_Builder * const b, llvm::Value * handle, llvm::Value * addr, llvm::Value * streamIndex, llvm::Value * blockIndex, const bool readOnly) const;

    virtual llvm::Value * getStreamPackPtr(IDISA::IDISA_Builder * const b, llvm::Value * handle, llvm::Value * addr, llvm::Value * streamIndex, llvm::Value * blockIndex, llvm::Value * packIndex, const bool readOnly) const;
    
    virtual llvm::Value * getStreamSetCount(IDISA::IDISA_Builder * const b, llvm::Value * handle) const;

    virtual llvm::Value * getRawItemPointer(IDISA::IDISA_Builder * const b, llvm::Value * handle, llvm::Value * absolutePosition) const;

    virtual void setBaseAddress(IDISA::IDISA_Builder * const b, llvm::Value * addr, llvm::Value *) const;

    virtual void setBufferedSize(IDISA::IDISA_Builder * const b, llvm::Value * size, llvm::Value *) const;
    
    virtual llvm::Value * getBufferedSize(IDISA::IDISA_Builder * const b, llvm::Value * handle) const;
    
    virtual void setCapacity(IDISA::IDISA_Builder * const b, llvm::Value * size, llvm::Value *) const;
    
    virtual llvm::Value * getCapacity(IDISA::IDISA_Builder * const b, llvm::Value * handle) const;
    
    // The number of items that cam be linearly accessed from a given logical stream position.
    virtual llvm::Value * getLinearlyAccessibleItems(IDISA::IDISA_Builder * const b, llvm::Value * handle, llvm::Value * fromPos, llvm::Value * avail, bool reverse = false) const;

    virtual llvm::Value * getLinearlyCopyableItems(IDISA::IDISA_Builder * const b, llvm::Value * handle, llvm::Value * fromPos, llvm::Value * avail, bool reverse = false) const {
        return getLinearlyAccessibleItems(b, handle, fromPos, avail, reverse);
    }
    
    void createBlockCopy(IDISA::IDISA_Builder * const b, llvm::Value * targetBlockPtr, llvm::Value * sourceBlockPtr, llvm::Value * blocksToCopy) const;

    virtual void createBlockAlignedCopy(IDISA::IDISA_Builder * const b, llvm::Value * targetBlockPtr, llvm::Value * sourceBlockPtr, llvm::Value * itemsToCopy, const unsigned alignment = 1) const;

    virtual llvm::Value * getLinearlyWritableItems(IDISA::IDISA_Builder * const b, llvm::Value * handle, llvm::Value * fromPosition, llvm::Value * consumed, bool reverse = false) const;
    
    bool supportsCopyBack() const {
        return mOverflowBlocks != 0;
    }

    size_t overflowSize() const {
        return mOverflowBlocks;
    }

    virtual void genCopyBackLogic(IDISA::IDISA_Builder * const b, llvm::Value * handle, llvm::Value * priorProduced, llvm::Value * newProduced, const std::string) const;
    
    virtual ~StreamSetBuffer() = 0;

    kernel::Kernel * getProducer() const {
        return mProducer;
    }

    const std::vector<kernel::Kernel *> & getConsumers() const {
        return mConsumers;
    }



protected:

    StreamSetBuffer(BufferKind k, llvm::Type * baseType, llvm::Type * resolvedType, unsigned BufferBlocks, unsigned OverflowBlocks, unsigned AddressSpace);

    virtual llvm::Value * getBlockAddress(IDISA::IDISA_Builder * const b, llvm::Value * handle, llvm::Value * blockNo) const;

    bool isCapacityGuaranteed(const llvm::Value * const index, const size_t capacity) const;

    llvm::Value * modBufferSize(IDISA::IDISA_Builder * const b, llvm::Value * const offset) const;

    virtual llvm::Value * getBaseAddress(IDISA::IDISA_Builder * const b, llvm::Value * handle) const;

    void setProducer(kernel::Kernel * const producer) {
        assert (producer);
        mProducer = producer;
    }

    void addConsumer(kernel::Kernel * const consumer) {
        assert (consumer);
        mConsumers.push_back(consumer);
    }

protected:
    const BufferKind                 mBufferKind;
    llvm::Type * const               mType;
    const size_t                     mBufferBlocks;
    size_t                           mOverflowBlocks;    /* Number of data blocks of additional space at the end of the buffer for writing only. */
    const unsigned                   mAddressSpace;
    llvm::Value *                    mStreamSetBufferPtr;
    llvm::Type * const               mBaseType;
    std::string                      mUniqueID;
    kernel::Kernel *                 mProducer;
    std::vector<kernel::Kernel *>    mConsumers;
};   

class SourceBuffer final : public StreamSetBuffer {
public:
    static inline bool classof(const StreamSetBuffer * b) {
        return b->getBufferKind() == BufferKind::SourceBuffer;
    }

    SourceBuffer(const std::unique_ptr<kernel::KernelBuilder> & b, llvm::Type * type, unsigned MemoryAddressSpace = 0, unsigned StructAddressSpace = 0);

    void setBaseAddress(IDISA::IDISA_Builder * const b, llvm::Value * handle, llvm::Value * addr) const override;

    void setBufferedSize(IDISA::IDISA_Builder * const b, llvm::Value * handle, llvm::Value * size) const override;

    void setCapacity(IDISA::IDISA_Builder * const b, llvm::Value * handle, llvm::Value * c) const override;

    llvm::Value * getBufferedSize(IDISA::IDISA_Builder * const b, llvm::Value * handle) const override;
    
    llvm::Value * getCapacity(IDISA::IDISA_Builder * const b, llvm::Value * handle) const override;
    
    llvm::Value * getLinearlyAccessibleItems(IDISA::IDISA_Builder * const b, llvm::Value * handle, llvm::Value * fromPosition, llvm::Value * avail, bool reverse = false) const override;

    llvm::Value * getLinearlyWritableItems(IDISA::IDISA_Builder * const b, llvm::Value * handle, llvm::Value * fromPosition, llvm::Value * consumed, bool reverse = false) const override;

    llvm::Type * getStreamSetBlockType() const override;

    void allocateBuffer(const std::unique_ptr<kernel::KernelBuilder> & kb) override;

    void releaseBuffer(const std::unique_ptr<kernel::KernelBuilder> & kb) const override;

protected:
    
    enum class Field {BaseAddress, BufferedSize, Capacity};

    llvm::Value * getBaseAddress(IDISA::IDISA_Builder * const b, llvm::Value * handle) const override;

};

class ExternalBuffer final : public StreamSetBuffer {
public:
    static inline bool classof(const StreamSetBuffer * b) {
        return b->getBufferKind() == BufferKind::ExternalBuffer;
    }

    ExternalBuffer(const std::unique_ptr<kernel::KernelBuilder> & b, llvm::Type * type, llvm::Value * addr, unsigned AddressSpace = 0);

    llvm::Value * getLinearlyAccessibleItems(IDISA::IDISA_Builder * const b, llvm::Value * handle, llvm::Value * fromPosition, llvm::Value * avail, bool reverse = false) const override;
    
    llvm::Value * getLinearlyWritableItems(IDISA::IDISA_Builder * const b, llvm::Value * handle, llvm::Value * fromPosition, llvm::Value * consumed, bool reverse = false) const override;

    void allocateBuffer(const std::unique_ptr<kernel::KernelBuilder> & b) override;

    void releaseBuffer(const std::unique_ptr<kernel::KernelBuilder> & kb) const override;

    llvm::Value * getCapacity(IDISA::IDISA_Builder * const b, llvm::Value * handle) const override;
};

class CircularBuffer : public StreamSetBuffer {
public:
    static inline bool classof(const StreamSetBuffer * b) {
        return b->getBufferKind() == BufferKind::CircularBuffer;
    }
    
    CircularBuffer(const std::unique_ptr<kernel::KernelBuilder> & b, llvm::Type * type, size_t bufferBlocks, unsigned AddressSpace = 0);

    llvm::Value * getRawItemPointer(IDISA::IDISA_Builder * const b, llvm::Value * handle, llvm::Value * absolutePosition) const final;

    llvm::Value * getLinearlyCopyableItems(IDISA::IDISA_Builder * const b, llvm::Value * handle, llvm::Value * fromPos, llvm::Value * avail, bool reverse = false) const final;

protected:

    CircularBuffer(const BufferKind kind, const std::unique_ptr<kernel::KernelBuilder> & b, llvm::Type * type, size_t bufferBlocks, size_t overflowBlocks, unsigned AddressSpace);

    llvm::Value * getBlockAddress(IDISA::IDISA_Builder * const b, llvm::Value * handle, llvm::Value * blockIndex) const final;
};
    

//
//  A CircularCopybackBuffer operates as a circular buffer buffer with an overflow area
//  for temporary use by the kernel that writes to it.   If the kernel uses the overflow
//  area, it must perform the doCopyBack action before releasing the buffer for use by
//  subsequent kernels.
//  Kernels that read from a CircularCopybackBuffer must not access the overflow area.
//
class CircularCopybackBuffer final : public CircularBuffer {
public:
    static inline bool classof(const StreamSetBuffer * b) {return b->getBufferKind() == BufferKind::CircularCopybackBuffer;}
    
    CircularCopybackBuffer(const std::unique_ptr<kernel::KernelBuilder> & b, llvm::Type * type, size_t bufferBlocks, size_t overflowBlocks, unsigned AddressSpace = 0);
    
    llvm::Value * getLinearlyWritableItems(IDISA::IDISA_Builder * const b, llvm::Value * handle, llvm::Value * fromPosition, llvm::Value * consumed, bool reverse = false) const override;
    
    void allocateBuffer(const std::unique_ptr<kernel::KernelBuilder> & b) override;

    void genCopyBackLogic(IDISA::IDISA_Builder * const b, llvm::Value * handle, llvm::Value * priorProduced, llvm::Value * newProduced, const std::string) const override;

};
    
class SwizzledCopybackBuffer final : public StreamSetBuffer {
public:
    static inline bool classof(const StreamSetBuffer * b) {return b->getBufferKind() == BufferKind::SwizzledCopybackBuffer;}
    
    SwizzledCopybackBuffer(const std::unique_ptr<kernel::KernelBuilder> & b, llvm::Type * type, size_t bufferBlocks, size_t overflowBlocks, unsigned fieldwidth = 64, unsigned AddressSpace = 0);
       
    void createBlockAlignedCopy(IDISA::IDISA_Builder * const b, llvm::Value * targetBlockPtr, llvm::Value * sourceBlockPtr, llvm::Value * itemsToCopy, const unsigned alignment = 1) const override;

    llvm::Value * getLinearlyWritableItems(IDISA::IDISA_Builder * const b, llvm::Value * handle, llvm::Value * fromPosition, llvm::Value * consumed, bool reverse = false) const override;
    
    void allocateBuffer(const std::unique_ptr<kernel::KernelBuilder> & b) override;

    void genCopyBackLogic(IDISA::IDISA_Builder * const b, llvm::Value * handle, llvm::Value * priorProduced, llvm::Value * newProduced, const std::string) const override;

protected:
    llvm::Value * getBlockAddress(IDISA::IDISA_Builder * const b, llvm::Value * handle, llvm::Value * blockIndex) const override;
private:
    unsigned mFieldWidth;
    
};

// ExpandableBuffers do not allow access to the base stream set but will automatically increase the number of streams
// within their set whenever the index exceeds its capacity
//
class ExpandableBuffer final : public StreamSetBuffer {
public:
    static inline bool classof(const StreamSetBuffer * b) {return b->getBufferKind() == BufferKind::ExpandableBuffer;}

    ExpandableBuffer(const std::unique_ptr<kernel::KernelBuilder> & b, llvm::Type * type, size_t bufferBlocks, unsigned AddressSpace = 0);

    llvm::Value * getStreamBlockPtr(IDISA::IDISA_Builder * const b, llvm::Value * handle, llvm::Value * addr, llvm::Value * streamIndex, llvm::Value * blockIndex, const bool readOnly) const override;

    llvm::Value * getStreamPackPtr(IDISA::IDISA_Builder * const b, llvm::Value * handle, llvm::Value * addr, llvm::Value * streamIndex, llvm::Value * blockIndex, llvm::Value * packIndex, const bool readOnly) const override;

    llvm::Value * getLinearlyAccessibleItems(IDISA::IDISA_Builder * const b, llvm::Value * handle, llvm::Value * fromPosition, llvm::Value * avail, bool reverse = false) const override;
    
    llvm::Value * getStreamSetCount(IDISA::IDISA_Builder * const b, llvm::Value * handle) const override;

    void allocateBuffer(const std::unique_ptr<kernel::KernelBuilder> & b) override;

    void releaseBuffer(const std::unique_ptr<kernel::KernelBuilder> & kb) const override;

protected:

    llvm::Value * getBaseAddress(IDISA::IDISA_Builder * const b, llvm::Value * handle) const override;

    llvm::Value * getBlockAddress(IDISA::IDISA_Builder * const b, llvm::Value * blockIndex, llvm::Value *) const override;

private:

    std::pair<llvm::Value *, llvm::Value *> getInternalStreamBuffer(IDISA::IDISA_Builder * const b, llvm::Value * handle, llvm::Value * streamIndex, llvm::Value * blockIndex, const bool readOnly) const;

private:

    const uint64_t  mInitialCapacity;

};
    
// Dynamically allocated circular buffers: TODO: add copyback, swizzle support, dynamic allocation, producer, consumer, length
class DynamicBuffer: public StreamSetBuffer {
public:
    static inline bool classof(const StreamSetBuffer * b) {return b->getBufferKind() == BufferKind::DynamicBuffer;}
    
    DynamicBuffer(const std::unique_ptr<kernel::KernelBuilder> & b, llvm::Type * type, size_t initialCapacity, size_t overflowBlocks = 0, unsigned swizzleFactor = 1, unsigned addrSpace = 0);
    
    llvm::Value * getLinearlyAccessibleItems(IDISA::IDISA_Builder * const b, llvm::Value * handle, llvm::Value * fromPosition, llvm::Value * avail, bool reverse = false) const override;
    
    llvm::Value * getLinearlyWritableItems(IDISA::IDISA_Builder * const b, llvm::Value * handle, llvm::Value * fromPosition, llvm::Value * consumed, bool reverse = false) const override;
    
    void allocateBuffer(const std::unique_ptr<kernel::KernelBuilder> & b) override;

    void releaseBuffer(const std::unique_ptr<kernel::KernelBuilder> & kb) const override;

    llvm::Value * getRawItemPointer(IDISA::IDISA_Builder * const b, llvm::Value * handle, llvm::Value * absolutePosition) const override;
    
    llvm::Value * getBufferedSize(IDISA::IDISA_Builder * const b, llvm::Value * handle) const override;
    
    void doubleCapacity(IDISA::IDISA_Builder * const b, llvm::Value * handle);

    void genCopyBackLogic(IDISA::IDISA_Builder * const b, llvm::Value * handle, llvm::Value * priorProduced, llvm::Value * newProduced, const std::string) const override;

protected:
    llvm::Value * getBaseAddress(IDISA::IDISA_Builder * const b, llvm::Value * handle) const override;
    
    llvm::Value * getBlockAddress(IDISA::IDISA_Builder * const b, llvm::Value * handle, llvm::Value * blockIndex) const override;

    
private:
    /* Static data */
    llvm::Type * mBufferStructType;      /* The type of the buffer struct. */
    unsigned   mSwizzleFactor;     /* Number of streams swizzled together per block.  Must be a small power of 2. Default: 1. */
    
    /* Dynamic data fields stored in the buffer struct */
    
    enum class Field {BaseAddress, PriorBaseAddress, AllocatedCapacity, WorkingBlocks, Length, ProducedPosition, ConsumedPosition, FieldCount};
    
    /* BaseAddress - the physical base address of the memory area for stream set data.
     PriorBaseAddress - the physical base address of the previous memory area for stream set data
     (the immediately prior memory area is preserved in case any users in other threads
     are accessing it).
     WorkingBlocks - the physical size of the buffer for use in reading and writing data.
     AllocatedCapacity - physical size available for expansion in place
     Length - actual final length of stream set or -1 for unknown
     ProducedPosition - the total number of items ever generated and stored in the buffer
     ConsumedPosition - the number of buffer items that are known to have been fully processed by all users
     */
    
};


}
#endif // STREAMSET_H
