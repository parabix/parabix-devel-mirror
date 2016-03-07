/*
 *  Copyright (c) 2016 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 */

#include "kernel.h"
#include <pablo/function.h>
#include <IDISA/idisa_builder.h>

using namespace llvm;
using namespace pablo;

inline bool isPowerOfTwo(const unsigned x) {
    return (x != 0) && (x & (x - 1)) == 0;
}

// sets name & sets internal state to the kernel superclass state
KernelBuilder::KernelBuilder(std::string name, Module * m, IDISA::IDISA_Builder * b)
: mMod(m)
, iBuilder(b)
, mKernelName(name)
, mBitBlockType(b->getBitBlockType())
, mBlockSize(b->getBitBlockWidth())
, mBlocksPerSegment(1)
, mCircularBufferModulo(1)
, mSegmentIndex(0)
, mStartIndex(0) {
    addInternalStateType(b->getInt64Ty());
    addInternalStateType(b->getInt64Ty());
    addInternalStateType(b->getInt64Ty());
    addInternalStateType(b->getInt64Ty());
}

unsigned KernelBuilder::addInternalStateType(Type * type){
    unsigned idx = mStates.size();
    mStates.push_back(type);
    return idx;
}
void KernelBuilder::addOutputStream(const unsigned fields){
    if (fields == 1){
        mOutputStreams.push_back(mBitBlockType);
    }
    else {
        mOutputStreams.push_back(ArrayType::get(mBitBlockType, fields));
    }

}
void KernelBuilder::addOutputAccum(Type * t){
    mOutputAccums.push_back(t);

}
void KernelBuilder::addInputStream(const unsigned fields, std::string name){
    if (name.empty())
        mInputStreamNames.push_back(mKernelName + "_inputstream_" + std::to_string(mInputStreams.size()));
    else
        mInputStreamNames.push_back(name);

    if (fields == 1){
        mInputStreams.push_back(mBitBlockType);
    } else {
        mInputStreams.push_back(ArrayType::get(mBitBlockType, fields));
    }
}
void KernelBuilder::addInputScalar(Type * t, std::string name){
    if (name.empty())
        mInputScalarNames.push_back(mKernelName + "_inputscalar_" + std::to_string(mInputScalars.size()));
    else
        mInputScalarNames.push_back(name);

    mInputScalars.push_back(t);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief prepareFunction
 ** ------------------------------------------------------------------------------------------------------------- */
Function * KernelBuilder::prepareFunction() {    
    if (mCircularBufferModulo > 1) {
        mStartIndex = addInternalStateType(iBuilder->getInt32Ty());
    }
    const unsigned capacity = mBlocksPerSegment + mCircularBufferModulo - 1;
    mInputStreamType = PointerType::get(ArrayType::get(StructType::get(mMod->getContext(), mInputStreams), capacity), 0);
    mInputScalarType = PointerType::get(StructType::get(mMod->getContext(), mInputScalars), 0);
    Type * outputStreamType = ArrayType::get(StructType::get(mMod->getContext(), mOutputStreams), capacity);
    Type * outputAccumType = StructType::get(mMod->getContext(), mOutputAccums);
    Type * stateType = StructType::create(mMod->getContext(), mStates, mKernelName);
    mKernelStructType = StructType::create(mMod->getContext(),std::vector<Type *>({stateType, outputStreamType, outputAccumType}), "KernelStruct_"+ mKernelName);

    FunctionType * functionType = FunctionType::get(Type::getVoidTy(mMod->getContext()),
        std::vector<Type *>({PointerType::get(mKernelStructType, 0), mInputStreamType}), false);

    mFunction = Function::Create(functionType, GlobalValue::ExternalLinkage, mKernelName + "_DoBlock", mMod);
    mFunction->setCallingConv(CallingConv::C);

    Function::arg_iterator args = mFunction->arg_begin();
    mKernelParam = args++;
    mKernelParam->setName("this");

    mInputParam = args++;
    mInputParam->setName("input_stream");

    iBuilder->SetInsertPoint(BasicBlock::Create(mMod->getContext(), "entry", mFunction, 0));

    mSegmentIndex = 0;

    return mFunction;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief finalize
 ** ------------------------------------------------------------------------------------------------------------- */
void KernelBuilder::finalize() {
    Type * const int64Ty = iBuilder->getInt64Ty();

    // Finish the actual function
    if (mCircularBufferModulo > 1) {
        Value * startIdx = getInternalState(mStartIndex);
        Value * value = iBuilder->CreateAdd(iBuilder->CreateBlockAlignedLoad(startIdx), iBuilder->getInt32(1));
        iBuilder->CreateBlockAlignedStore(value, startIdx);
    }
    iBuilder->CreateRetVoid();


    // Generate the zero initializer
    Function * initializer = cast<Function>(mMod->getOrInsertFunction(mKernelName + "_Init", Type::getVoidTy(mMod->getContext()), PointerType::get(mKernelStructType, 0), nullptr));
    initializer->setCallingConv(CallingConv::C);
    Function::arg_iterator args = initializer->arg_begin();

    mKernelParam = args++;
    mKernelParam->setName("this");

    iBuilder->SetInsertPoint(BasicBlock::Create(mMod->getContext(), "entry", initializer, 0));

    for (unsigned i = 0; i < mStates.size(); ++i) {
        Value * const gep = getInternalState(i);
        Type * const type = gep->getType();
        if (type->isIntegerTy() || type->isArrayTy() || type->isVectorTy()) {
            setInternalState(i, Constant::getNullValue(type));
        } else {
            Value * gep_next = iBuilder->CreateGEP(gep, iBuilder->getInt32(1));
            Value * get_int = iBuilder->CreatePtrToInt(gep, int64Ty);
            Value * get_next_int = iBuilder->CreatePtrToInt(gep_next, int64Ty);
            Value * state_size = iBuilder->CreateSub(get_next_int, get_int);
            iBuilder->CreateMemSet(gep, iBuilder->getInt8(0), state_size, 4);
        }
    }

    iBuilder->CreateRetVoid();

    // and then the constructor
    mConstructor = cast<Function>(mMod->getOrInsertFunction(mKernelName+"_Create_Default", Type::getVoidTy(mMod->getContext()), PointerType::get(mKernelStructType, 0), int64Ty, int64Ty, nullptr));
    mConstructor->setCallingConv(CallingConv::C);
    args = mConstructor->arg_begin();

    mKernelParam = args++;
    mKernelParam->setName("this");

    Value* block_size_param = args++;
    block_size_param->setName("block_size");
    Value* seg_size_param = args++;
    seg_size_param->setName("seg_size");
    iBuilder->SetInsertPoint(BasicBlock::Create(mMod->getContext(), "entry", mConstructor, 0));
    iBuilder->CreateStore(block_size_param, getInternalState(0));
    iBuilder->CreateStore(seg_size_param, getInternalState(1));
    iBuilder->CreateCall(initializer, mKernelParam);
    iBuilder->CreateRetVoid();
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief generateKernelInstance
 ** ------------------------------------------------------------------------------------------------------------- */
Value * KernelBuilder::generateKernelInstance() {
    mKernelStruct = iBuilder->CreateAlloca(mKernelStructType);
    iBuilder->CreateCall3(mConstructor, mKernelStruct,
        ConstantInt::get(iBuilder->getIntNTy(64), mBlockSize),
        ConstantInt::get(iBuilder->getIntNTy(64), (mBlocksPerSegment + mCircularBufferModulo) * mBlockSize));
    return mKernelStruct;

}

Value * KernelBuilder::getInputStream(const unsigned index, const unsigned streamOffset) {
    Value * const indices[] = {iBuilder->getInt32(0), getOffset(streamOffset), iBuilder->getInt32(index)};
    return iBuilder->CreateGEP(mInputParam, indices);
}

Value * KernelBuilder::getKernelState(const unsigned index, const unsigned streamOffset) {
    Value * const indices[] = {iBuilder->getInt32(0), iBuilder->getInt32(0), getOffset(streamOffset), iBuilder->getInt32(index)};
    return iBuilder->CreateGEP(mKernelParam, indices);
}

Value * KernelBuilder::getOutputStream(const unsigned index, const unsigned streamOffset) {
    Value * const indices[] = {iBuilder->getInt32(0), iBuilder->getInt32(1), getOffset(streamOffset), iBuilder->getInt32(index)};
    return iBuilder->CreateGEP(mKernelParam, indices);
}

Value * KernelBuilder::getOutputScalar(const unsigned index, const unsigned streamOffset) {
    Value * const indices[] = {iBuilder->getInt32(0), iBuilder->getInt32(2), getOffset(streamOffset), iBuilder->getInt32(index)};
    return iBuilder->CreateGEP(mKernelParam, indices);
}

Value * KernelBuilder::getInternalState(const unsigned index){
    Value* indices[] = {iBuilder->getInt64(0), iBuilder->getInt32(0), iBuilder->getInt32(index)};
    return iBuilder->CreateGEP(mKernelParam, indices);
}

void KernelBuilder::setInternalState(const unsigned index, Value * const value) {
    iBuilder->CreateBlockAlignedStore(value, getInternalState(index));
}

void KernelBuilder::generateInitCall(){
    iBuilder->CreateCall(mInitFunction, mKernelStruct);
}

void KernelBuilder::generateDoBlockCall(Value * inputStreams){
    iBuilder->CreateCall2(mFunction, mKernelStruct, inputStreams);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief offset
 *
 * Compute the index of the given offset value.
 ** ------------------------------------------------------------------------------------------------------------- */
Value * KernelBuilder::getOffset(const unsigned offset) {
    Value * index = iBuilder->getInt32(mSegmentIndex + offset);
    if (mStartIndex) {
        index = iBuilder->CreateAdd(iBuilder->CreateBlockAlignedLoad(getInternalState(mStartIndex)), index);
        const unsigned capacity = (mBlocksPerSegment + mCircularBufferModulo);
        if (isPowerOfTwo(capacity)) {
            index = iBuilder->CreateAnd(index, ConstantInt::get(index->getType(), capacity - 1));
        } else {
            index = iBuilder->CreateURem(index, ConstantInt::get(index->getType(), capacity));
        }
        // TODO: generate branch / phi node when it's sufficiently unlikely that we'll wrap around.
    }
    return index;
}

