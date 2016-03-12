/*
 *  Copyright (c) 2016 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 */

#include "kernel.h"
#include <pablo/function.h>
#include <IDISA/idisa_builder.h>
#include <llvm/Support/CommandLine.h>

using namespace llvm;
using namespace pablo;

static cl::opt<unsigned> SegmentSize("segment-size", cl::desc("Segment Size"), cl::value_desc("positive integer"), cl::init(1));

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
, mBlocksPerSegment(SegmentSize)
, mCircularBufferModulo(1)
, mSegmentIndex(0)
, mBlockIndex(0) {
    assert (mBlocksPerSegment > 0);
    mBlockIndex = addInternalState(b->getInt64Ty(), "BlockNo");
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief addInternalState
 ** ------------------------------------------------------------------------------------------------------------- */
unsigned KernelBuilder::addInternalState(Type * const type) {
    assert (type);
    const unsigned index = mStates.size();
    mStates.push_back(type);
    return index;
}

unsigned KernelBuilder::addInternalState(llvm::Type * const type, std::string name) {
    if (LLVM_UNLIKELY(mStateNameMap.count(name) != 0)) {
        throw std::runtime_error("Kernel already contains internal state " + name);
    }
    const unsigned index = addInternalState(type);
    mStateNameMap.emplace(name, index);
    return index;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief addOutputStream
 ** ------------------------------------------------------------------------------------------------------------- */
void KernelBuilder::addOutputStream(const unsigned fields) {
    assert (fields > 0);
    mOutputStreams.push_back((fields == 1) ? mBitBlockType : ArrayType::get(mBitBlockType, fields));
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief addOutputScalar
 ** ------------------------------------------------------------------------------------------------------------- */
void KernelBuilder::addOutputScalar(Type * const type) {
    assert (type);
    mOutputScalar.push_back(type);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief addInputStream
 ** ------------------------------------------------------------------------------------------------------------- */
void KernelBuilder::addInputStream(const unsigned fields, std::string name) {
    assert (fields > 0 && !name.empty());
    mInputStreamNames.push_back(name);
    if (fields == 1){
        mInputStreams.push_back(mBitBlockType);
    } else {
        mInputStreams.push_back(ArrayType::get(mBitBlockType, fields));
    }
}

void KernelBuilder::addInputStream(const unsigned fields) {
    addInputStream(fields, std::move(mKernelName + "_inputstream_" + std::to_string(mInputStreams.size())));
}


/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getInputStream
 ** ------------------------------------------------------------------------------------------------------------- */
Value * KernelBuilder::getInputStream(const unsigned index, const unsigned streamOffset) {
    Value * const indices[] = {getOffset(streamOffset), iBuilder->getInt32(index)};
    return iBuilder->CreateGEP(mInputParam, indices);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getInputScalar
 ** ------------------------------------------------------------------------------------------------------------- */
Value * KernelBuilder::getInputScalar(const unsigned) {
    throw std::runtime_error("currently not supported!");
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getOutputStream
 ** ------------------------------------------------------------------------------------------------------------- */
Value * KernelBuilder::getOutputStream(const unsigned index, const unsigned streamOffset) {
    Value * const indices[] = {iBuilder->getInt32(0), iBuilder->getInt32(1), getOffset(streamOffset), iBuilder->getInt32(index)};
    return iBuilder->CreateGEP(mKernelParam, indices);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getOutputScalar
 ** ------------------------------------------------------------------------------------------------------------- */
Value * KernelBuilder::getOutputScalar(const unsigned) {
    throw std::runtime_error("currently not supported!");
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getInternalState
 ** ------------------------------------------------------------------------------------------------------------- */
Value * KernelBuilder::getInternalState(const unsigned index, Value * const inputStruct) {
    Value* indices[] = {iBuilder->getInt64(0), iBuilder->getInt32(0), iBuilder->getInt32(index)};
    return iBuilder->CreateGEP(inputStruct ? inputStruct : mKernelParam, indices);
}

Value * KernelBuilder::getInternalState(const std::string & name, Value * const inputStruct) {
    const auto f = mStateNameMap.find(name);
    if (LLVM_UNLIKELY(f == mStateNameMap.end())) {
        throw std::runtime_error("Kernel does not contain internal state " + name);
    }
    return getInternalState(f->second, inputStruct);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief setInternalState
 ** ------------------------------------------------------------------------------------------------------------- */
void KernelBuilder::setInternalState(const unsigned index, Value * const value) {
    Value * ptr = getInternalState(index);
    assert (ptr->getType()->getPointerElementType() == value->getType());
    if (value->getType() == iBuilder->getBitBlockType()) {
        iBuilder->CreateBlockAlignedStore(value, ptr);
    } else {
        iBuilder->CreateStore(value, ptr);
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief addInputScalar
 ** ------------------------------------------------------------------------------------------------------------- */
void KernelBuilder::addInputScalar(Type * const type, std::string name) {
    assert (type && !name.empty());
    mInputScalarNames.push_back(name);
    mInputScalars.push_back(type);
}

void KernelBuilder::addInputScalar(Type * const type) {
    addInputScalar(type, std::move(mKernelName + "_inputscalar_" + std::to_string(mInputScalars.size())));
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief prepareFunction
 ** ------------------------------------------------------------------------------------------------------------- */
Function * KernelBuilder::prepareFunction() {    
    if (mCircularBufferModulo > 1) {
        mBlockIndex = addInternalState(iBuilder->getInt32Ty());
    }
    const unsigned capacity = mBlocksPerSegment + mCircularBufferModulo - 1;

    mInputStreamType = PointerType::get(StructType::get(mMod->getContext(), mInputStreams), 0);
    mInputScalarType = PointerType::get(StructType::get(mMod->getContext(), mInputScalars), 0);
    Type * outputStreamType = ArrayType::get(StructType::get(mMod->getContext(), mOutputStreams), capacity);
    Type * outputAccumType = StructType::get(mMod->getContext(), mOutputScalar);
    Type * internalStateType = StructType::create(mMod->getContext(), mStates, mKernelName);
    mKernelStructType = StructType::create(mMod->getContext(),std::vector<Type *>({internalStateType, outputStreamType, outputAccumType}), "KernelStruct_"+ mKernelName);

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

    // Finish the actual function
    Value * startIdx = getInternalState(mBlockIndex);
    Value * value = iBuilder->CreateBlockAlignedLoad(startIdx);
    value = iBuilder->CreateAdd(value, ConstantInt::get(value->getType(), 1));
    iBuilder->CreateBlockAlignedStore(value, startIdx);
    iBuilder->CreateRetVoid();

    // Generate the zero initializer
    Function * initializer = cast<Function>(mMod->getOrInsertFunction(mKernelName + "_Init", Type::getVoidTy(mMod->getContext()), PointerType::get(mKernelStructType, 0), nullptr));
    initializer->setCallingConv(CallingConv::C);
    Function::arg_iterator args = initializer->arg_begin();
    mKernelParam = args++;
    mKernelParam->setName("this");

    iBuilder->SetInsertPoint(BasicBlock::Create(mMod->getContext(), "entry", initializer, 0));

    Type * const int64Ty = iBuilder->getInt64Ty(); // TODO: should call getIntPtrTy() instead but we don't have the data layout here.
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

    iBuilder->SetInsertPoint(BasicBlock::Create(mMod->getContext(), "entry", mConstructor, 0));
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
        ConstantInt::get(iBuilder->getIntNTy(64), (mBlocksPerSegment + mCircularBufferModulo - 1) * mBlockSize));
    return mKernelStruct;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief generateDoBlockCall
 ** ------------------------------------------------------------------------------------------------------------- */
void KernelBuilder::generateInitCall() {
    assert (mInitFunction && mKernelStruct);
    iBuilder->CreateCall(mInitFunction, mKernelStruct);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief generateDoBlockCall
 ** ------------------------------------------------------------------------------------------------------------- */
void KernelBuilder::generateDoBlockCall(Value * inputStreams) {
    assert (mFunction && mKernelStruct);
    iBuilder->CreateCall2(mFunction, mKernelStruct, iBuilder->CreatePointerCast(inputStreams, mInputStreamType));
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief offset
 *
 * Compute the stream index of the given offset value.
 ** ------------------------------------------------------------------------------------------------------------- */
Value * KernelBuilder::getOffset(const unsigned value) {
    const unsigned adjustedOffset = (mSegmentIndex + value);
    Value * offset = iBuilder->getInt32(adjustedOffset);
    if (mBlockIndex) {
        Value * index = iBuilder->CreateBlockAlignedLoad(getInternalState(mBlockIndex));
        if (adjustedOffset) {
            index = iBuilder->CreateAdd(index, offset);
        }
        const unsigned bufferSize = (mBlocksPerSegment + mCircularBufferModulo - 1); assert (bufferSize > 1);
        if (isPowerOfTwo(bufferSize)) {
            index = iBuilder->CreateAnd(index, ConstantInt::get(index->getType(), bufferSize - 1));
        } else {
            index = iBuilder->CreateURem(index, ConstantInt::get(index->getType(), bufferSize));
        }
        // TODO: generate branch / phi node when it's sufficiently unlikely that we'll wrap around.
        offset = index;
    }
    return offset;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief setLongestLookaheadAmount
 ** ------------------------------------------------------------------------------------------------------------- */
void KernelBuilder::setLongestLookaheadAmount(const unsigned bits) {
    const unsigned blockWidth = iBuilder->getBitBlockWidth();
    const unsigned lookaheadBlocks = (bits + blockWidth - 1) / blockWidth;
    mCircularBufferModulo = (lookaheadBlocks + 1);
}
