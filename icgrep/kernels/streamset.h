/*
 *  Copyright (c) 2016 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 */

#ifndef STREAMSET_H
#define STREAMSET_H

#include <llvm/IR/Type.h>  // for Type
namespace IDISA { class IDISA_Builder; }
namespace llvm { class PointerType; }
namespace llvm { class Value; }
namespace kernel { class KernelBuilder; }

namespace parabix {
    
class StreamSetBuffer {
    friend class kernel::KernelBuilder;

public:

    enum class BufferKind : unsigned {BlockBuffer, ExternalFileBuffer, CircularBuffer, CircularCopybackBuffer, ExpandableBuffer};

    BufferKind getBufferKind() const {
        return mBufferKind;
    }

    llvm::Type * getType() const {
        return mStreamSetType;
    }

    llvm::Type * getBaseType() const {
        return mBaseStreamSetType;
    }

    llvm::PointerType * getPointerType() const {
        return getType()->getPointerTo(mAddressSpace);
    }

    size_t getBufferBlocks() const {
        return mBufferBlocks;
    }

    llvm::Value * getStreamSetBasePtr() const {
        return mStreamSetBufferPtr;
    }

    virtual void allocateBuffer();

    virtual llvm::Value * getStream(llvm::Value * self, llvm::Value * blockNo, llvm::Value * index) const;

    virtual llvm::Value * getStream(llvm::Value * self, llvm::Value * blockNo, llvm::Value * index1, llvm::Value * index2) const;
    
    virtual llvm::Value * getStreamView(llvm::Type * type, llvm::Value * self, llvm::Value * blockNo, llvm::Value * index) const;

    // The number of items that cam be linearly accessed from a given logical stream position.
    virtual llvm::Value * getLinearlyAccessibleItems(llvm::Value * fromPosition) const;
    
protected:

    StreamSetBuffer(BufferKind k, IDISA::IDISA_Builder * b, llvm::Type * type, unsigned blocks, unsigned AddressSpace);

    // Get the buffer pointer for a given block of the stream.
    virtual llvm::Value * getStreamSetPtr(llvm::Value * self, llvm::Value * blockNo) const = 0;

    llvm::Type * resolveStreamSetBufferType(llvm::Type * type) const;

protected:
    const BufferKind                mBufferKind;
    IDISA::IDISA_Builder * const    iBuilder;
    llvm::Type * const              mStreamSetType;
    const size_t                    mBufferBlocks;
    const unsigned                  mAddressSpace;
    llvm::Value *                   mStreamSetBufferPtr;
    llvm::Type * const              mBaseStreamSetType;
};   

class SingleBlockBuffer : public StreamSetBuffer {
public:
    static inline bool classof(const StreamSetBuffer * b) {
        return b->getBufferKind() == BufferKind::BlockBuffer;
    }   

    SingleBlockBuffer(IDISA::IDISA_Builder * b, llvm::Type * type);

protected:
    llvm::Value * getStreamSetPtr(llvm::Value * self, llvm::Value * blockNo) const override;
};

class ExternalFileBuffer : public StreamSetBuffer {
public:
    static inline bool classof(const StreamSetBuffer * b) {
        return b->getBufferKind() == BufferKind::ExternalFileBuffer;
    }
    
    ExternalFileBuffer(IDISA::IDISA_Builder * b, llvm::Type * type, unsigned AddressSpace = 0);

    void setStreamSetBuffer(llvm::Value * ptr, llvm::Value * fileSize);

    void setEmptyBuffer(llvm::Value * buffer_ptr);

    // Can't allocate - raise an error. */
    void allocateBuffer() override;

    llvm::Value * getLinearlyAccessibleItems(llvm::Value * fromPosition) const override;
    
protected:
    llvm::Value * getStreamSetPtr(llvm::Value * self, llvm::Value * blockNo) const override;
};
    
class CircularBuffer : public StreamSetBuffer {
public:
    static inline bool classof(const StreamSetBuffer * b) {
        return b->getBufferKind() == BufferKind::CircularBuffer;
    }
    
    CircularBuffer(IDISA::IDISA_Builder * b, llvm::Type * type, size_t bufferBlocks, unsigned AddressSpace = 0);

protected:
    llvm::Value * getStreamSetPtr(llvm::Value * bufferBasePtr, llvm::Value * blockNo) const override;
};
    

//
//  A CircularCopybackBuffer operates as a circular buffer buffer with an overflow area
//  for temporary use by the kernel that writes to it.   If the kernel uses the overflow
//  area, it must perform the doCopyBack action before releasing the buffer for use by
//  subsequent kernels.
//  Kernels that read from a CircularCopybackBuffer must not access the overflow area.
//
class CircularCopybackBuffer : public StreamSetBuffer {
public:
    static inline bool classof(const StreamSetBuffer * b) {return b->getBufferKind() == BufferKind::CircularCopybackBuffer;}
    
    CircularCopybackBuffer(IDISA::IDISA_Builder * b, llvm::Type * type, size_t bufferBlocks, size_t overflowBlocks, unsigned AddressSpace = 0);

    void allocateBuffer() override;
    
    // Generate copyback code for the given number of overflowItems.
    void createCopyBack(llvm::Value * self, llvm::Value * overflowItems) const;
    
    
protected:
    llvm::Value * getStreamSetPtr(llvm::Value * bufferBasePtr, llvm::Value * blockNo) const override;
private:
    size_t mOverflowBlocks;

};

// ExpandableBuffers do not allow access to the base stream set but will automatically increase the number of streams
// within their set whenever the index exceeds its capacity
//
class ExpandableBuffer : public StreamSetBuffer {
public:
    static inline bool classof(const StreamSetBuffer * b) {return b->getBufferKind() == BufferKind::ExpandableBuffer;}

    ExpandableBuffer(IDISA::IDISA_Builder * b, llvm::Type * type, size_t bufferBlocks, unsigned AddressSpace = 0);

    llvm::Value * getStream(llvm::Value * self, llvm::Value * blockNo, llvm::Value * index) const override;

    llvm::Value * getStream(llvm::Value * self, llvm::Value * blockNo, llvm::Value * index1, llvm::Value * index2) const override;

    llvm::Value * getStreamView(llvm::Type * type, llvm::Value * self, llvm::Value * blockNo, llvm::Value * index) const override;

    llvm::Value * getLinearlyAccessibleItems(llvm::Value * fromPosition) const override;
    
protected:

    llvm::Value * getStreamSetPtr(llvm::Value * self, llvm::Value * blockNo) const override;
};

}
#endif // STREAMSET_H
