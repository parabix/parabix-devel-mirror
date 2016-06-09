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
    
namespace kernel {

class StreamSetType {
public:
    StreamSetType(int count, int width) : mStreamCount(count), mFieldWidth(width) {}
    int StreamCount() { return mStreamCount;}
    int StreamFieldWidth() { return mFieldWidth;}
    
    llvm::Type * getStreamSetBlockType(IDISA::IDISA_Builder * iBuilder);
    
private:
    int mStreamCount;
    int mFieldWidth;
};

class StreamSetBuffer {
public:
    StreamSetBuffer(IDISA::IDISA_Builder * b, StreamSetType ss_type, unsigned SegmentSize) :
    iBuilder(b), mStreamSetType(ss_type), mSegmentSize(SegmentSize), mStreamSetBufferPtr(nullptr) {}

    llvm::Type * getStreamSetBlockType();
    
    llvm::Type * getStreamSetBufferType();
    
    void setStreamSetBuffer(llvm::Value * ptr) {mStreamSetBufferPtr = ptr;}
    
    llvm::Value * allocateBuffer();
    
    llvm::Value * getBlockPointer(llvm::Value * blockNo);
private:
    IDISA::IDISA_Builder * iBuilder;
    StreamSetType mStreamSetType;
    unsigned mSegmentSize;
    llvm::Value * mStreamSetBufferPtr;
    

};
}
#endif // STREAMSET_H
