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
    
// Stream Set Structs hold information about the current state of a stream set buffer.

llvm::Value * getProducerPosPtr(IDISA::IDISA_Builder * b, llvm::Value * self);
llvm::Value * getConsumerPosPtr(IDISA::IDISA_Builder * b, llvm::Value * self);
llvm::Value * getEndOfInputPtr(IDISA::IDISA_Builder * b, llvm::Value * self);
llvm::Value * getStreamSetBufferPtr(IDISA::IDISA_Builder * b, llvm::Value * self);

class StreamSetBuffer {
    friend class kernel::KernelBuilder;

public:

    enum class BufferKind : unsigned {BlockBuffer, ExternalFileBuffer, CircularBuffer, LinearCopybackBuffer, ExpandableBuffer};

    inline BufferKind getBufferKind() const {
        return mBufferKind;
    }

    inline llvm::Type * getBufferStreamSetType() const {
        return mStreamSetType;
    }

    llvm::PointerType * getStreamBufferPointerType() const {
        return mStreamSetType->getPointerTo(mAddrSpace);
    }

    llvm::PointerType * getStreamSetStructPointerType() const {
        return mStreamSetStructType->getPointerTo();
    }

    size_t getBufferSize() const { return mBufferBlocks; }

    llvm::Value * getStreamSetBasePtr() const { return mStreamSetBufferPtr; }

    llvm::Value * getStreamSetStructPtr() const { return mStreamSetStructPtr; }

    virtual void allocateBuffer();

    virtual llvm::Value * getStream(llvm::Value * self, llvm::Value * blockNo, llvm::Value * index) const;

    virtual llvm::Value * getStream(llvm::Value * self, llvm::Value * blockNo, llvm::Value * index1, llvm::Value * index2) const;
    
    virtual llvm::Value * getStreamView(llvm::Value * self, llvm::Value * blockNo, llvm::Value * index) const;

    virtual llvm::Value * getStreamView(llvm::Type * type, llvm::Value * self, llvm::Value * blockNo, llvm::Value * index) const;

    llvm::Value * getProducerPosPtr(llvm::Value * self) const;

    void setProducerPos(llvm::Value * self, llvm::Value * pos) const;

    llvm::Value * getConsumerPosPtr(llvm::Value * self) const;

    virtual void setConsumerPos(llvm::Value * self, llvm::Value * pos) const;

    llvm::Value * getEndOfInputPtr(llvm::Value * self) const;

    void setEndOfInput(llvm::Value * self) const;
    
    llvm::Type * resolveStreamTypes(llvm::Type * type);
    
protected:

    StreamSetBuffer(BufferKind k, IDISA::IDISA_Builder * b, llvm::Type * type, unsigned blocks, unsigned AddressSpace);

    // Get the buffer pointer for a given block of the stream.
    virtual llvm::Value * getStreamSetPtr(llvm::Value * self, llvm::Value * blockNo) const = 0;

protected:
    const BufferKind                mBufferKind;
    IDISA::IDISA_Builder * const    iBuilder;
    llvm::Type * const              mStreamSetType;
    const size_t                    mBufferBlocks;
    const int                       mAddrSpace;
    llvm::Value *                   mStreamSetBufferPtr;
    llvm::Value *                   mStreamSetStructPtr;
    llvm::Type * const              mStreamSetStructType;
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
    
// Linear buffers extending from the current ConsumerPos forward.   Within the buffer, the
// offset of the block containing the current consumer position is always zero.
//
class LinearCopybackBuffer : public StreamSetBuffer {
public:
    static inline bool classof(const StreamSetBuffer * b) {return b->getBufferKind() == BufferKind::LinearCopybackBuffer;}
    
    LinearCopybackBuffer(IDISA::IDISA_Builder * b, llvm::Type * type, size_t bufferBlocks, unsigned AddressSpace = 0);

    // Reset the buffer to contain data starting at the base block of new_consumer_pos,
    // copying back any data beyond that position. 
    void setConsumerPos(llvm::Value * self, llvm::Value * newConsumerPos) const override;

protected:
    llvm::Value * getStreamSetPtr(llvm::Value * self, llvm::Value * blockNo) const override;
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

    llvm::Value * getStreamView(llvm::Value * self, llvm::Value * blockNo, llvm::Value * index) const override;

    llvm::Value * getStreamView(llvm::Type * type, llvm::Value * self, llvm::Value * blockNo, llvm::Value * index) const override;

protected:

    llvm::Value * getStreamSetPtr(llvm::Value * self, llvm::Value * blockNo) const override;
};

}
#endif // STREAMSET_H
