/*
 *  Copyright (c) 2016 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 */

#include "streamset.h"
#include <IR_Gen/idisa_builder.h>  // for IDISA_Builder
#include <IR_Gen/types/streamtype.h>
#include <assert.h>                // for assert
#include <llvm/IR/Type.h>          // for Type
#include <stdexcept>               // for runtime_error
#include <llvm/IR/BasicBlock.h>    // for BasicBlock
#include <llvm/IR/Constants.h>     // for ConstantInt
#include <llvm/IR/DataLayout.h>    // for DataLayout
#include <llvm/IR/DerivedTypes.h>  // for IntegerType (ptr only), PointerType
#include <llvm/IR/Module.h>        // for Module
#include <llvm/IR/Value.h>         // for Value
namespace llvm { class Constant; }
namespace llvm { class Function; }

using namespace parabix;
using namespace llvm;
using namespace IDISA;

enum SS_struct_index {iProducer_pos = 0, iConsumer_pos = 1, iEnd_of_input = 2, iBuffer_ptr = 3};

Value * StreamSetBuffer::getProducerPosPtr(Value * self) const {
    return iBuilder->CreateGEP(self, {iBuilder->getInt32(0), iBuilder->getInt32(iProducer_pos)});
}

void StreamSetBuffer::setProducerPos(Value * self, Value * pos) const {
    iBuilder->CreateStore(pos, getProducerPosPtr(self));
}

Value * StreamSetBuffer::getConsumerPosPtr(Value * self) const {
    return iBuilder->CreateGEP(self, {iBuilder->getInt32(0), iBuilder->getInt32(iConsumer_pos)});
}

void StreamSetBuffer::setConsumerPos(Value * self, Value * pos) const {
    iBuilder->CreateStore(pos, getConsumerPosPtr(self));
}

Value * StreamSetBuffer::getEndOfInputPtr(Value * self) const {
    return iBuilder->CreateGEP(self, {iBuilder->getInt32(0), iBuilder->getInt32(iEnd_of_input)});
}

void StreamSetBuffer::setEndOfInput(Value * self) const {
    iBuilder->CreateStore(ConstantInt::get(iBuilder->getInt1Ty(), 1), getEndOfInputPtr(self));
}

Type * StreamSetBuffer::resolveStreamTypes(Type * type) {
    if (auto ty = dyn_cast<ArrayType>(type)) {
        unsigned numElems = ty->getNumElements();
        auto elemTy = ty->getElementType();
        if (isa<StreamType>(elemTy)) {
            return ArrayType::get(cast<StreamType>(elemTy)->resolveType(iBuilder), numElems);
        }
    }
    else if (auto ty = dyn_cast<StreamType>(type)) {
        return ty->resolveType(iBuilder);
    }
    return type;
}

void StreamSetBuffer::allocateBuffer() {
    Type * const sizeTy = iBuilder->getSizeTy();
    Type * const int1ty = iBuilder->getInt1Ty();
    mStreamSetBufferPtr = iBuilder->CreateCacheAlignedAlloca(mStreamSetType, iBuilder->getSize(mBufferBlocks));
    mStreamSetStructPtr = iBuilder->CreateCacheAlignedAlloca(mStreamSetStructType);
    iBuilder->CreateStore(ConstantInt::get(sizeTy, 0), iBuilder->CreateGEP(mStreamSetStructPtr, {iBuilder->getInt32(0), iBuilder->getInt32(iProducer_pos)}));
    iBuilder->CreateStore(ConstantInt::get(sizeTy, 0), iBuilder->CreateGEP(mStreamSetStructPtr, {iBuilder->getInt32(0), iBuilder->getInt32(iConsumer_pos)}));
    iBuilder->CreateStore(ConstantInt::get(int1ty, 0), iBuilder->CreateGEP(mStreamSetStructPtr, {iBuilder->getInt32(0), iBuilder->getInt32(iEnd_of_input)}));
    iBuilder->CreateStore(mStreamSetBufferPtr, iBuilder->CreateGEP(mStreamSetStructPtr, {iBuilder->getInt32(0), iBuilder->getInt32(iBuffer_ptr)}));
}


Value * StreamSetBuffer::getStream(Value * self, Value * blockNo, Value * index) const {
    return iBuilder->CreateGEP(getStreamSetPtr(self, blockNo), {iBuilder->getInt32(0), index});
}

Value * StreamSetBuffer::getStream(Value * self, Value * blockNo, Value * index1, Value * index2) const {
    return iBuilder->CreateGEP(getStreamSetPtr(self, blockNo), {iBuilder->getInt32(0), index1, index2});
}

Value * StreamSetBuffer::getStreamView(llvm::Value * self, Value * blockNo, llvm::Value * index) const {
    return iBuilder->CreateGEP(getStreamSetPtr(self, blockNo), index, "view");
}

Value * StreamSetBuffer::getStreamView(llvm::Type * type, llvm::Value * self, Value * blockNo, llvm::Value * index) const {
    return iBuilder->CreateGEP(iBuilder->CreatePointerCast(getStreamSetPtr(self, blockNo), type), index, "view");
}

// Single Block Buffer

// For a single block buffer, the block pointer is always the buffer base pointer.
Value * SingleBlockBuffer::getStreamSetPtr(Value * self, Value *) const {
    return iBuilder->CreateLoad(iBuilder->CreateGEP(self, {iBuilder->getInt32(0), iBuilder->getInt32(iBuffer_ptr)}), "sb");
}

// External File Buffer
void ExternalFileBuffer::setStreamSetBuffer(Value * ptr, Value * fileSize) {
    
    Type * const size_ty = iBuilder->getSizeTy();
    Type * const int1ty = iBuilder->getInt1Ty();
    
    PointerType * t = getStreamBufferPointerType();    
    mStreamSetBufferPtr = iBuilder->CreatePointerBitCastOrAddrSpaceCast(ptr, t);
    
    mStreamSetStructPtr = iBuilder->CreateCacheAlignedAlloca(mStreamSetStructType);
    iBuilder->CreateStore(fileSize, iBuilder->CreateGEP(mStreamSetStructPtr, {iBuilder->getInt32(0), iBuilder->getInt32(iProducer_pos)}));
    iBuilder->CreateStore(ConstantInt::get(size_ty, 0), iBuilder->CreateGEP(mStreamSetStructPtr, {iBuilder->getInt32(0), iBuilder->getInt32(iConsumer_pos)}));
    iBuilder->CreateStore(ConstantInt::get(int1ty, 1), iBuilder->CreateGEP(mStreamSetStructPtr, {iBuilder->getInt32(0), iBuilder->getInt32(iEnd_of_input)}));
    iBuilder->CreateStore(mStreamSetBufferPtr, iBuilder->CreateGEP(mStreamSetStructPtr, {iBuilder->getInt32(0), iBuilder->getInt32(iBuffer_ptr)}));
}

void ExternalFileBuffer::setEmptyBuffer(Value * ptr) {
    
    Type * const size_ty = iBuilder->getSizeTy();
    Type * const int1ty = iBuilder->getInt1Ty();
    
    PointerType * t = getStreamBufferPointerType();    
    mStreamSetBufferPtr = iBuilder->CreatePointerBitCastOrAddrSpaceCast(ptr, t);
    
    mStreamSetStructPtr = iBuilder->CreateCacheAlignedAlloca(mStreamSetStructType);
    iBuilder->CreateStore(ConstantInt::get(size_ty, 0), iBuilder->CreateGEP(mStreamSetStructPtr, {iBuilder->getInt32(0), iBuilder->getInt32(iProducer_pos)}));
    iBuilder->CreateStore(ConstantInt::get(size_ty, 0), iBuilder->CreateGEP(mStreamSetStructPtr, {iBuilder->getInt32(0), iBuilder->getInt32(iConsumer_pos)}));
    iBuilder->CreateStore(ConstantInt::get(int1ty,0), iBuilder->CreateGEP(mStreamSetStructPtr, {iBuilder->getInt32(0), iBuilder->getInt32(iEnd_of_input)}));
    iBuilder->CreateStore(mStreamSetBufferPtr, iBuilder->CreateGEP(mStreamSetStructPtr, {iBuilder->getInt32(0), iBuilder->getInt32(iBuffer_ptr)}));
}

void ExternalFileBuffer::allocateBuffer() {
    throw std::runtime_error("External buffers cannot be allocated.");
}

Value * ExternalFileBuffer::getStreamSetPtr(Value * self, Value * blockNo) const {
    Value * handle = iBuilder->CreateGEP(self, {iBuilder->getInt32(0), iBuilder->getInt32(iBuffer_ptr)}, "ef");
    Value * bufPtr = iBuilder->CreateLoad(handle);
    return iBuilder->CreateGEP(bufPtr, blockNo);
}

// Circular Buffer

Value * CircularBuffer::getStreamSetPtr(Value * self, Value * blockNo) const {
    assert (blockNo->getType()->isIntegerTy());

    Value * handle = iBuilder->CreateGEP(self, {iBuilder->getInt32(0), iBuilder->getInt32(iBuffer_ptr)}, "cb");
    Value * bufPtr = iBuilder->CreateLoad(handle);
    Value * offset = nullptr;
    if (mBufferBlocks == 1) {
        offset = ConstantInt::getNullValue(iBuilder->getSizeTy());
    } else if ((mBufferBlocks & (mBufferBlocks - 1)) == 0) { // is power of 2
        offset = iBuilder->CreateAnd(blockNo, ConstantInt::get(blockNo->getType(), mBufferBlocks - 1));
    } else {
        offset = iBuilder->CreateURem(blockNo, ConstantInt::get(blockNo->getType(), mBufferBlocks));
    }
    return iBuilder->CreateGEP(bufPtr, offset);
}

// Linear Copyback Buffer

Value * LinearCopybackBuffer::getStreamSetPtr(Value * self, Value * blockNo) const {
    Value * consumerPos_ptr = iBuilder->CreateGEP(self, {iBuilder->getInt32(0), iBuilder->getInt32(iConsumer_pos)}, "lcb.c");
    Value * consumerPos = iBuilder->CreateLoad(consumerPos_ptr);
    Value * consumerBlock = iBuilder->CreateUDiv(consumerPos, iBuilder->getSize(iBuilder->getStride()));
    consumerBlock = iBuilder->CreateZExtOrTrunc(consumerBlock, blockNo->getType());
    Value * handle = iBuilder->CreateGEP(self, {iBuilder->getInt32(0), iBuilder->getInt32(iBuffer_ptr)}, "lcb.p");
    Value * bufPtr = iBuilder->CreateLoad(handle);
    return iBuilder->CreateGEP(bufPtr, iBuilder->CreateSub(blockNo, consumerBlock));
}

void LinearCopybackBuffer::setConsumerPos(Value * self, Value * newConsumerPos) const {
    Type * const i8 = iBuilder->getInt8Ty();
    Type * const i8_ptr = i8->getPointerTo(mAddrSpace);
    IntegerType * const sizeTy = iBuilder->getSizeTy();

    Module * const M = iBuilder->getModule();

    Function * const current = iBuilder->GetInsertBlock()->getParent();
    BasicBlock * const copyBackBody = BasicBlock::Create(M->getContext(), "copy_back", current, 0);
    BasicBlock * const setConsumerPosExit = BasicBlock::Create(M->getContext(), "setConsumerPos_done", current, 0);
    Constant * const blockWidth = ConstantInt::get(sizeTy, iBuilder->getStride());

    Constant * const one = ConstantInt::get(sizeTy, 1);

    Value * const consumerPosPtr = getConsumerPosPtr(self);
    Value * const consumerPos = iBuilder->CreateLoad(consumerPosPtr);

    // Ensure that the new consumer position is no less than the current position.
    newConsumerPos = iBuilder->CreateSelect(iBuilder->CreateICmpULT(newConsumerPos, consumerPos), consumerPos, newConsumerPos);
    Value * producerPos = iBuilder->CreateLoad(iBuilder->CreateGEP(self, {iBuilder->getInt32(0), iBuilder->getInt32(iProducer_pos)}));

    // Ensure that the new consumer position is no greater than the current producer position.
    Value * new_pos_lt_producer_pos = iBuilder->CreateICmpULT(newConsumerPos, producerPos);
    newConsumerPos = iBuilder->CreateSelect(new_pos_lt_producer_pos, newConsumerPos, producerPos);

    // Now, the new_consumer_pos is at most = to the producer_pos; if =, we're done.
    iBuilder->CreateCondBr(new_pos_lt_producer_pos, copyBackBody, setConsumerPosExit);
    iBuilder->SetInsertPoint(copyBackBody);
    
    Value * new_consumer_block = iBuilder->CreateUDiv(newConsumerPos, blockWidth);
    Value * lastProducerBlock = iBuilder->CreateUDiv(iBuilder->CreateSub(producerPos, one), blockWidth);
    Value * copyBlocks = iBuilder->CreateAdd(iBuilder->CreateSub(lastProducerBlock, new_consumer_block), one);

    DataLayout dl(iBuilder->getModule());

    Constant * blockBytes = ConstantInt::get(sizeTy, dl.getTypeAllocSize(mStreamSetType) * iBuilder->getStride());

    Value * copyLength = iBuilder->CreateMul(copyBlocks, blockBytes);

    // Must copy back one full block for each of the streams in the stream set.
    Value * handle = iBuilder->CreateGEP(self, {iBuilder->getInt32(0), iBuilder->getInt32(iBuffer_ptr)});
    Value * bufferPtr = iBuilder->CreateLoad(handle);
    Value * const consumerBlock = iBuilder->CreateUDiv(consumerPos, blockWidth);
    Value * copyFrom = iBuilder->CreateGEP(bufferPtr, iBuilder->CreateSub(new_consumer_block, consumerBlock));
    unsigned alignment = iBuilder->getBitBlockWidth() / 8;
    iBuilder->CreateMemMove(iBuilder->CreateBitCast(bufferPtr, i8_ptr), iBuilder->CreateBitCast(copyFrom, i8_ptr), copyLength, alignment);
    iBuilder->CreateBr(setConsumerPosExit);
    // Copy back done, store the new consumer position.
    iBuilder->SetInsertPoint(setConsumerPosExit);

    iBuilder->CreateStore(newConsumerPos, consumerPosPtr);
}

// Expandable Buffer

Value * ExpandableBuffer::getStreamSetPtr(Value * self, Value * blockNo) const {
    return nullptr;
}

llvm::Value * ExpandableBuffer::getStream(llvm::Value * self, llvm::Value * blockNo, llvm::Value * index) const {
    return nullptr;
}

llvm::Value * ExpandableBuffer::getStream(llvm::Value * self, llvm::Value * blockNo, Value *index1, Value *index2) const {
    return nullptr;
}

llvm::Value * ExpandableBuffer::getStreamView(llvm::Value * self, llvm::Value * blockNo, llvm::Value * index) const {
    return nullptr;
}

llvm::Value * ExpandableBuffer::getStreamView(llvm::Type * type, llvm::Value * self, llvm::Value * blockNo, llvm::Value * index) const {
    return nullptr;
}


// Constructors

SingleBlockBuffer::SingleBlockBuffer(IDISA::IDISA_Builder * b, llvm::Type * type)
: StreamSetBuffer(BufferKind::BlockBuffer, b, type, 1, 0) {

}

ExternalFileBuffer::ExternalFileBuffer(IDISA::IDISA_Builder * b, llvm::Type * type, unsigned AddressSpace)
: StreamSetBuffer(BufferKind::ExternalFileBuffer, b, type, 0, AddressSpace) {

}

CircularBuffer::CircularBuffer(IDISA::IDISA_Builder * b, llvm::Type * type, size_t bufferBlocks, unsigned AddressSpace)
: StreamSetBuffer(BufferKind::CircularBuffer, b, type, bufferBlocks, AddressSpace) {

}

LinearCopybackBuffer::LinearCopybackBuffer(IDISA::IDISA_Builder * b, llvm::Type * type, size_t bufferBlocks, unsigned AddressSpace)
: StreamSetBuffer(BufferKind::LinearCopybackBuffer, b, type, bufferBlocks, AddressSpace) {

}

ExpandableBuffer::ExpandableBuffer(IDISA::IDISA_Builder * b, llvm::Type * type, size_t bufferBlocks, unsigned AddressSpace)
: StreamSetBuffer(BufferKind::ExpandableBuffer, b, type, bufferBlocks, AddressSpace) {

}

StreamSetBuffer::StreamSetBuffer(BufferKind k, IDISA::IDISA_Builder * b, Type * type, unsigned blocks, unsigned AddressSpace)
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
