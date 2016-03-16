/*
 *  Copyright (c) 2016 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 */

#include "kernel.h"
#include <pablo/function.h>
#include <IDISA/idisa_builder.h>
#include <llvm/Support/CommandLine.h>
#include <kernels/instance.h>

using namespace llvm;
using namespace pablo;

static cl::opt<unsigned> SegmentSize("segment-size", cl::desc("Segment Size"), cl::value_desc("positive integer"), cl::init(1));

inline bool isPowerOfTwo(const unsigned x) {
    return (x != 0) && (x & (x - 1)) == 0;
}

namespace kernel {

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

SlabAllocator<Instance> Instance::mAllocator; // static allocator declaration; should probably be in a "instance.cpp"

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief addInternalState
 ** ------------------------------------------------------------------------------------------------------------- */
unsigned KernelBuilder::addInternalState(Type * const type) {
    assert (type);
    const unsigned index = mInternalState.size();
    mInternalState.push_back(type);
    return index;
}

unsigned KernelBuilder::addInternalState(llvm::Type * const type, std::string && name) {
    if (LLVM_UNLIKELY(mInternalStateNameMap.count(name) != 0)) {
        throw std::runtime_error("Kernel already contains internal state " + name);
    }
    const unsigned index = addInternalState(type);
    mInternalStateNameMap.emplace(name, index);
    return index;
}


/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getInternalState
 ** ------------------------------------------------------------------------------------------------------------- */
Value * KernelBuilder::getInternalState(Value * const instance, const unsigned index) {
    Value* indices[] = {iBuilder->getInt64(0), iBuilder->getInt32(0), iBuilder->getInt32(index)};
    return iBuilder->CreateGEP(instance ? instance : mKernelParam, indices);
}

Value * KernelBuilder::getInternalState(Value * const instance, const std::string & name) {
    const auto f = mInternalStateNameMap.find(name);
    if (LLVM_UNLIKELY(f == mInternalStateNameMap.end())) {
        throw std::runtime_error("Kernel does not contain internal state " + name);
    }
    return getInternalState(instance, f->second);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief setInternalState
 ** ------------------------------------------------------------------------------------------------------------- */
void KernelBuilder::setInternalState(Value * const instance, const std::string & name, Value * const value) {
    Value * ptr = getInternalState(instance, name);
    assert (ptr->getType()->getPointerElementType() == value->getType());
    if (value->getType() == iBuilder->getBitBlockType()) {
        iBuilder->CreateBlockAlignedStore(value, ptr);
    } else {
        iBuilder->CreateStore(value, ptr);
    }
}

void KernelBuilder::setInternalState(Value * const instance, const unsigned index, Value * const value) {
    Value * ptr = getInternalState(instance, index);
    assert (ptr->getType()->getPointerElementType() == value->getType());
    if (value->getType() == iBuilder->getBitBlockType()) {
        iBuilder->CreateBlockAlignedStore(value, ptr);
    } else {
        iBuilder->CreateStore(value, ptr);
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief addInputStream
 ** ------------------------------------------------------------------------------------------------------------- */
void KernelBuilder::addInputStream(const unsigned fields, std::string && name) {
    assert (fields > 0 && !name.empty());
    mInputStreamName.push_back(name);
    if (fields == 1){
        mInputStream.push_back(mBitBlockType);
    } else {
        mInputStream.push_back(ArrayType::get(mBitBlockType, fields));
    }
}

void KernelBuilder::addInputStream(const unsigned fields) {
    addInputStream(fields, std::move(mKernelName + "_inputstream_" + std::to_string(mInputStream.size())));
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getInputStream
 ** ------------------------------------------------------------------------------------------------------------- */
Value * KernelBuilder::getInputStream(llvm::Value * const instance, const unsigned index, const unsigned streamOffset) {
    assert (instance);
    Value * const indices[] = {getOffset(instance, streamOffset), iBuilder->getInt32(index)};
    return iBuilder->CreateGEP(instance, indices);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief addInputScalar
 ** ------------------------------------------------------------------------------------------------------------- */
void KernelBuilder::addInputScalar(Type * const type, std::string && name) {
    assert (type && !name.empty());
    mInputScalarName.push_back(name);
    mInputScalar.push_back(type);
}

void KernelBuilder::addInputScalar(Type * const type) {
    addInputScalar(type, std::move(mKernelName + "_inputscalar_" + std::to_string(mInputScalar.size())));
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getInputScalar
 ** ------------------------------------------------------------------------------------------------------------- */
Value * KernelBuilder::getInputScalar(Value * const instance, const unsigned) {
    throw std::runtime_error("currently not supported!");
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief addOutputStream
 ** ------------------------------------------------------------------------------------------------------------- */
unsigned KernelBuilder::addOutputStream(const unsigned fields) {
    assert (fields > 0);
    const unsigned index = mOutputStream.size();
    mOutputStream.push_back((fields == 1) ? mBitBlockType : ArrayType::get(mBitBlockType, fields));
    return index;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief addOutputScalar
 ** ------------------------------------------------------------------------------------------------------------- */
unsigned KernelBuilder::addOutputScalar(Type * const type) {
    assert (type);
    const unsigned index = mOutputScalar.size();
    mOutputScalar.push_back(type);
    return index;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getOutputStream
 ** ------------------------------------------------------------------------------------------------------------- */
Value * KernelBuilder::getOutputStream(Value * const instance, const unsigned index, const unsigned streamOffset) {
    assert (instance);
    Value * const indices[] = {getOffset(instance, streamOffset), iBuilder->getInt32(1), iBuilder->getInt32(0), iBuilder->getInt32(index)};
    return iBuilder->CreateGEP(instance, indices);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getOutputStreams
 ** ------------------------------------------------------------------------------------------------------------- */
Value * KernelBuilder::getOutputStreamSet(Value * const instance, const unsigned streamOffset) {
    assert (instance);
    Value * const indices[] = {getOffset(instance, streamOffset), iBuilder->getInt32(1)};
    return iBuilder->CreateGEP(instance, indices);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getOutputScalar
 ** ------------------------------------------------------------------------------------------------------------- */
Value * KernelBuilder::getOutputScalar(Value * const instance, const unsigned) {
    throw std::runtime_error("currently not supported!");
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief prepareFunction
 ** ------------------------------------------------------------------------------------------------------------- */
Function * KernelBuilder::prepareFunction() {    
    if (mCircularBufferModulo > 1) {
        mBlockIndex = addInternalState(iBuilder->getInt32Ty());
    }
    const unsigned capacity = mBlocksPerSegment + mCircularBufferModulo - 1;

    mInputStreamType = PointerType::get(StructType::get(mMod->getContext(), mInputStream), 0);
    mInputScalarType = PointerType::get(StructType::get(mMod->getContext(), mInputScalar), 0);
    Type * outputStreamType = ArrayType::get(StructType::get(mMod->getContext(), mOutputStream), capacity);
    Type * outputAccumType = StructType::get(mMod->getContext(), mOutputScalar);
    Type * internalStateType = StructType::create(mMod->getContext(), mInternalState, mKernelName);
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

    Type * const int64Ty = iBuilder->getInt64Ty(); // TODO: should call getIntPtrTy() instead but we don't have the data layout here.

    // Generate the zero initializer
    mConstructor = cast<Function>(mMod->getOrInsertFunction(mKernelName + "_Constructor", Type::getVoidTy(mMod->getContext()), PointerType::get(mKernelStructType, 0), nullptr));
    mConstructor->setCallingConv(CallingConv::C);
    auto args = mConstructor->arg_begin();
    mKernelParam = args++;
    mKernelParam->setName("this");
    iBuilder->SetInsertPoint(BasicBlock::Create(mMod->getContext(), "entry", mConstructor, 0));
    for (unsigned i = 0; i < mInternalState.size(); ++i) {
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

    iBuilder->ClearInsertionPoint();

    mSegmentIndex = 0;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief instantiate
 *
 * Generate a new instance of this kernel and call the default constructor to initialize it
 ** ------------------------------------------------------------------------------------------------------------- */
Instance * KernelBuilder::instantiate() {
    AllocaInst * const memory = iBuilder->CreateAlloca(mKernelStructType);
    iBuilder->CreateCall(mConstructor, memory);
    return new Instance(this, memory);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief call
 ** ------------------------------------------------------------------------------------------------------------- */
void KernelBuilder::call(llvm::Value * const instance, Value * inputStreams) {
    assert (mFunction && instance && inputStreams);
    iBuilder->CreateCall2(mFunction, instance, iBuilder->CreatePointerCast(inputStreams, mInputStreamType));
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief offset
 *
 * Compute the stream index of the given offset value.
 ** ------------------------------------------------------------------------------------------------------------- */
Value * KernelBuilder::getOffset(Value * const instance, const unsigned value) {
    const unsigned adjustedOffset = (mSegmentIndex + value);
    if (mBlockIndex) {
        Value * offset = iBuilder->CreateBlockAlignedLoad(getBlockNo(instance));
        if (adjustedOffset) {
            offset = iBuilder->CreateAdd(offset, ConstantInt::get(offset->getType(), adjustedOffset));
        }
        const unsigned bufferSize = (mBlocksPerSegment + mCircularBufferModulo - 1); assert (bufferSize > 1);
        if (isPowerOfTwo(bufferSize)) {
            offset = iBuilder->CreateAnd(offset, ConstantInt::get(offset->getType(), bufferSize - 1));
        } else {
            offset = iBuilder->CreateURem(offset, ConstantInt::get(offset->getType(), bufferSize));
        }
        // TODO: generate branch / phi node when it's sufficiently unlikely that we'll wrap around.
        return offset;
    } else {
        return iBuilder->getInt32(adjustedOffset);
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief setLongestLookaheadAmount
 ** ------------------------------------------------------------------------------------------------------------- */
void KernelBuilder::setLongestLookaheadAmount(const unsigned bits) {
    const unsigned blockWidth = iBuilder->getBitBlockWidth();
    const unsigned lookaheadBlocks = (bits + blockWidth - 1) / blockWidth;
    mCircularBufferModulo = (lookaheadBlocks + 1);
}

} // end of namespace kernel
