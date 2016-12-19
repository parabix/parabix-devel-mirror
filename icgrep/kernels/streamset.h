/*
 *  Copyright (c) 2016 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 */

#ifndef STREAMSET_H
#define STREAMSET_H

#include <string>
#include <vector>
#include <IDISA/idisa_builder.h>
#include <llvm/IR/Type.h>

namespace parabix {
    
enum FieldType {i1 = 1, i2 = 2, i4 = 4, i8 = 8, i16 = 16, i32 = 32, i64 = 64, i128 = 128, i256 = 256};

// Stream Set Structs hold information about the current state of a stream set buffer.

llvm::Value * getProducerPosPtr(IDISA::IDISA_Builder * b, Value * bufferStructPtr);
llvm::Value * getConsumerPosPtr(IDISA::IDISA_Builder * b, Value * bufferStructPtr);
llvm::Value * getEndOfInputPtr(IDISA::IDISA_Builder * b, Value * bufferStructPtr);
llvm::Value * getStreamSetBufferPtr(IDISA::IDISA_Builder * b, Value * bufferStructPtr);

class StreamSetBuffer {
public:

    enum class BufferKind : unsigned {BlockBuffer, ExternalFileBuffer, CircularBuffer, LinearCopybackBuffer};

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

    // Get the buffer pointer for a given block of the stream.
    virtual llvm::Value * getStreamSetBlockPointer(llvm::Value * bufferStructPtr, llvm::Value * blockNo) = 0;
    
    llvm::Value * getProducerPosPtr(Value * bufferStructPtr);

    void setProducerPos(Value * bufferStructPtr, Value * pos);

    llvm::Value * getConsumerPosPtr(Value * bufferStructPtr);

    virtual void setConsumerPos(Value * bufferStructPtr, Value * pos);

    llvm::Value * getEndOfInputPtr(Value * bufferStructPtr);

    void setEndOfInput(Value * bufferStructPtr);
    
    llvm::Type * resolveStreamTypes(llvm::Type * type) {
        if (auto ty = dyn_cast<ArrayType>(type)) {
            unsigned numElems = ty->getNumElements();
            auto elemTy = ty->getElementType();
            if (isa<IDISA::StreamType>(elemTy)) {
                return ArrayType::get(cast<IDISA::StreamType>(elemTy)->resolveType(iBuilder), numElems);
            }
        }
        else if (auto ty = dyn_cast<IDISA::StreamType>(type)) {
            return ty->resolveType(iBuilder);
        }
        return type;
    }
    
protected:
    StreamSetBuffer(BufferKind k, IDISA::IDISA_Builder * b, llvm::Type * type, unsigned blocks, unsigned AddressSpace = 0)
    : mBufferKind(k)
    , iBuilder(b)
    , mStreamSetType(resolveStreamTypes(type))
    , mBufferBlocks(blocks)
    , mAddrSpace(AddressSpace)
    , mStreamSetBufferPtr(nullptr)
    , mStreamSetStructPtr(nullptr)
    , mStreamSetStructType(StructType::get(b->getContext(),
                            {{b->getSizeTy(),
                              b->getSizeTy(),
                              b->getInt1Ty(),
                              PointerType::get(mStreamSetType, AddressSpace)}})) {

    }
protected:
    const BufferKind        mBufferKind;
    IDISA::IDISA_Builder *  iBuilder;
    llvm::Type * const      mStreamSetType;
    size_t                  mBufferBlocks;
    int                     mAddrSpace;
    llvm::Value *           mStreamSetBufferPtr;
    llvm::Value *           mStreamSetStructPtr;
    llvm::Type * const      mStreamSetStructType;
};   

class SingleBlockBuffer : public StreamSetBuffer {
public:
    static inline bool classof(const StreamSetBuffer * b) {
        return b->getBufferKind() == BufferKind::BlockBuffer;
    }   
    SingleBlockBuffer(IDISA::IDISA_Builder * b, llvm::Type * type)
    : StreamSetBuffer(BufferKind::BlockBuffer, b, type, 1, 0) {

    }
    llvm::Value * getStreamSetBlockPointer(llvm::Value * bufferBasePtr, llvm::Value * blockNo) override;
};

class ExternalFileBuffer : public StreamSetBuffer {
public:
    static inline bool classof(const StreamSetBuffer * b) {
        return b->getBufferKind() == BufferKind::ExternalFileBuffer;
    }
    
    ExternalFileBuffer(IDISA::IDISA_Builder * b, llvm::Type * type, unsigned AddressSpace = 0)
    : StreamSetBuffer(BufferKind::ExternalFileBuffer, b, type, 0, AddressSpace) {

    }

    void setStreamSetBuffer(llvm::Value * ptr, llvm::Value * fileSize);
    void setEmptyBuffer(llvm::Value * buffer_ptr);

    // Can't allocate - raise an error. */
    void allocateBuffer() override;

    llvm::Value * getStreamSetBlockPointer(llvm::Value * bufferStructPtr, llvm::Value * blockNo) override;
};
    
class CircularBuffer : public StreamSetBuffer {
public:
    static inline bool classof(const StreamSetBuffer * b) {
        return b->getBufferKind() == BufferKind::CircularBuffer;
    }
  
    CircularBuffer(IDISA::IDISA_Builder * b, llvm::Type * type, size_t bufferBlocks, unsigned AddressSpace = 0)
    : StreamSetBuffer(BufferKind::CircularBuffer, b, type, bufferBlocks, AddressSpace) {

    }

    llvm::Value * getStreamSetBlockPointer(llvm::Value * bufferBasePtr, llvm::Value * blockNo) override;
};
    
// Linear buffers extending from the current ConsumerPos forward.   Within the buffer, the
// offset of the block containing the current consumer position is always zero.
//
class LinearCopybackBuffer : public StreamSetBuffer {
public:
    static inline bool classof(const StreamSetBuffer * b) {return b->getBufferKind() == BufferKind::LinearCopybackBuffer;}
    
    LinearCopybackBuffer(IDISA::IDISA_Builder * b, llvm::Type * type, size_t bufferBlocks, unsigned AddressSpace = 0) :
        StreamSetBuffer(BufferKind::LinearCopybackBuffer, b, type, bufferBlocks, AddressSpace) {}
    
    llvm::Value * getStreamSetBlockPointer(llvm::Value * bufferStructPtr, llvm::Value * blockNo) override;
    
    // Reset the buffer to contain data starting at the base block of new_consumer_pos,
    // copying back any data beyond that position. 
    void setConsumerPos(Value * bufferStructPtr, Value * new_consumer_pos) override;
};

}
#endif // STREAMSET_H
