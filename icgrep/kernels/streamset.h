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

class StreamSetType {
public:
    StreamSetType(int count, int width) : mStreamCount(count), mFieldWidth(width) {}
    int StreamCount() { return mStreamCount;}
    int StreamFieldWidth() { return mFieldWidth;}
    bool operator== (StreamSetType& other) {return (mStreamCount == other.mStreamCount) && (mFieldWidth == other.mFieldWidth);}
    
    llvm::Type * getStreamSetBlockType(IDISA::IDISA_Builder * iBuilder);
    
private:
    int mStreamCount;
    int mFieldWidth;
};

    
// Stream Set Structs hold information about the current state
// of a stream set buffer.
    
llvm::Value * getProducerPosPtr(IDISA::IDISA_Builder * b, Value * bufferStructPtr);    
llvm::Value * getConsumerPosPtr(IDISA::IDISA_Builder * b, Value * bufferStructPtr);
llvm::Value * hasEndOfInputPtr(IDISA::IDISA_Builder * b, Value * bufferStructPtr);    
llvm::Value * getStreamSetBufferPtr(IDISA::IDISA_Builder * b, Value * bufferStructPtr);    
    
class StreamSetBuffer {
public:
    enum class BufferKind : unsigned {BlockBuffer, ExternalFileBuffer, CircularBuffer, LinearCopybackBuffer};
    inline BufferKind getBufferKind() const {return mBufferKind;}
    inline StreamSetType& getBufferStreamSetType() {return mStreamSetType;}

    llvm::PointerType * getStreamBufferPointerType();

    size_t getBufferSize() { return mBufferBlocks;}
    
    virtual llvm::Value * allocateBuffer();
    
    llvm::Value * getStreamSetBasePtr() {return mStreamSetBufferPtr;}
    
    // Get the buffer pointer for a given block of the stream.
    virtual llvm::Value * getStreamSetBlockPointer(llvm::Value * bufferStructPtr, llvm::Value * blockNo) = 0;
    
    llvm::Value * getProducerPosPtr(Value * bufferStructPtr);

    void setProducerPos(Value * bufferStructPtr, Value * pos);

    llvm::Value * getConsumerPosPtr(Value * bufferStructPtr);

    virtual void setConsumerPos(Value * bufferStructPtr, Value * pos);

    llvm::Value * hasEndOfInputPtr(Value * bufferStructPtr);

    void setEndOfInput(Value * bufferStructPtr);
    
    llvm::Value * getStreamSetBufferPtrPtr(Value * bufferStructPtr);

    virtual llvm::PointerType * getStreamSetStructPointerType();

    virtual llvm::Value * getStreamSetStructPtr();
    
protected:
    StreamSetBuffer(BufferKind k, IDISA::IDISA_Builder * b, StreamSetType ss_type, unsigned blocks, unsigned AddressSpace = 0) :
        mBufferKind(k), iBuilder(b), mStreamSetType(ss_type), mBufferBlocks(blocks), mAddrSpace(AddressSpace), mStreamSetBufferPtr(nullptr) {
            mStreamSetStructType =
                StructType::get(iBuilder->getContext(),
                                std::vector<Type *>({iBuilder->getSizeTy(),
                                                    iBuilder->getSizeTy(),
                                                    iBuilder->getInt1Ty(),
                                                    PointerType::get(mStreamSetType.getStreamSetBlockType(iBuilder), AddressSpace)}));
    }
    
    const BufferKind       mBufferKind;
    IDISA::IDISA_Builder * iBuilder;
    StreamSetType mStreamSetType;
    size_t mBufferBlocks;
    unsigned mAddrSpace;
    llvm::Value * mStreamSetBufferPtr;
    llvm::Value * mStreamSetStructPtr;
    llvm::Type * mStreamSetStructType;
};   
    

class SingleBlockBuffer : public StreamSetBuffer {
public:
    static inline bool classof(const StreamSetBuffer * b) {return b->getBufferKind() == BufferKind::BlockBuffer;}
    
    SingleBlockBuffer(IDISA::IDISA_Builder * b, StreamSetType ss_type) :
    StreamSetBuffer(BufferKind::BlockBuffer, b, ss_type, 1, 0) {}
    llvm::Value * getStreamSetBlockPointer(llvm::Value * bufferStructPtr, llvm::Value * blockNo) override;
};
    
class ExternalFileBuffer : public StreamSetBuffer {
public:
    static inline bool classof(const StreamSetBuffer * b) {return b->getBufferKind() == BufferKind::ExternalFileBuffer;}
    
    ExternalFileBuffer(IDISA::IDISA_Builder * b, StreamSetType ss_type, unsigned AddressSpace = 0) :
        StreamSetBuffer(BufferKind::ExternalFileBuffer, b, ss_type, 0, AddressSpace) {}

    void setStreamSetBuffer(llvm::Value * ptr, llvm::Value * fileSize);
    void setEmptyBuffer(llvm::Value * buffer_ptr);
    
    // Can't allocate - raise an error. */
    llvm::Value * allocateBuffer() override;
    llvm::Value * getStreamSetBlockPointer(llvm::Value * bufferStructPtr, llvm::Value * blockNo) override;

};
    
class CircularBuffer : public StreamSetBuffer {
public:
    static inline bool classof(const StreamSetBuffer * b) {return b->getBufferKind() == BufferKind::CircularBuffer;}
  
    CircularBuffer(IDISA::IDISA_Builder * b, StreamSetType ss_type, size_t bufferBlocks, unsigned AddressSpace = 0) :
        StreamSetBuffer(BufferKind::CircularBuffer, b, ss_type, bufferBlocks, AddressSpace) {
            if (((bufferBlocks - 1) & bufferBlocks) != 0) {
                throw std::runtime_error("CircularStreamSetBuffer: number of blocks must be a power of 2!");
            }
        }
    llvm::Value * getStreamSetBlockPointer(llvm::Value * bufferStructPtr, llvm::Value * blockNo) override;
};
    
// Linear buffers extending from the current ConsumerPos forward.   Within the buffer, the
// offset of the block containing the current consumer position is always zero.
//
class LinearCopybackBuffer : public StreamSetBuffer {
public:
    static inline bool classof(const StreamSetBuffer * b) {return b->getBufferKind() == BufferKind::LinearCopybackBuffer;}
    
    LinearCopybackBuffer(IDISA::IDISA_Builder * b, StreamSetType ss_type, size_t bufferBlocks, unsigned AddressSpace = 0) :
        StreamSetBuffer(BufferKind::LinearCopybackBuffer, b, ss_type, bufferBlocks, AddressSpace) {}
    
    llvm::Value * getStreamSetBlockPointer(llvm::Value * bufferStructPtr, llvm::Value * blockNo) override;
    
    // Reset the buffer to contain data starting at the base block of new_consumer_pos,
    // copying back any data beyond that position. 
    void setConsumerPos(Value * bufferStructPtr, Value * new_consumer_pos) override;
};
    
    

}
#endif // STREAMSET_H
