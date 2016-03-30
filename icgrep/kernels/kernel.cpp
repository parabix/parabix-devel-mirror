/*
 *  Copyright (c) 2016 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 */

#include "kernel.h"
#include <pablo/function.h>
#include <IDISA/idisa_builder.h>
#include <kernels/instance.h>
#include <tuple>
#include <boost/functional/hash_fwd.hpp>
#include <unordered_map>

using namespace llvm;
using namespace pablo;

inline bool isPowerOfTwo(const unsigned x) {
    return (x != 0) && (x & (x - 1)) == 0;
}

namespace kernel {

enum : unsigned {
    INTERNAL_STATE = 0
    , INPUT_STREAM_SET = 1
    , OUTPUT_STREAM_SET = 2
    , OUTPUT_SCALAR_SET = 3
};

// sets name & sets internal state to the kernel superclass state
KernelBuilder::KernelBuilder(std::string name, Module * m, IDISA::IDISA_Builder * b, const unsigned bufferSize)
: mMod(m)
, iBuilder(b)
, mKernelName(name)
, mBitBlockType(b->getBitBlockType())
, mBufferSize(bufferSize)
, mBlockNoIndex(0) {
    assert (mBufferSize > 0);
    mBlockNoIndex = addInternalState(b->getInt64Ty(), "BlockNo");
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
        throw std::runtime_error("Kernel already contains internal state '" + name + "'");
    }
    const unsigned index = addInternalState(type);
    mInternalStateNameMap.emplace(name, index);
    return index;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getInternalState
 ** ------------------------------------------------------------------------------------------------------------- */
Value * KernelBuilder::getInternalState(Value * const instance, const unsigned index) {
    assert (index < mInternalState.size());
    return getInternalState(instance, iBuilder->getInt32(index));
}

Value * KernelBuilder::getInternalState(Value * const instance, disable_implicit_conversion<Value *> index) {
    assert (index->getType()->isIntegerTy());
    return iBuilder->CreateGEP(instance, {iBuilder->getInt64(0), iBuilder->getInt32(INTERNAL_STATE), index});
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
    assert (index < mInternalState.size());
    return setInternalState(instance, iBuilder->getInt32(index), value);
}

void KernelBuilder::setInternalState(Value * const instance, disable_implicit_conversion<Value *> index, Value * const value) {
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
    if (fields == 1) {
        mInputStream.push_back(mBitBlockType);
    } else {
        mInputStream.push_back(ArrayType::get(mBitBlockType, fields));
    }
}

void KernelBuilder::addInputStream(const unsigned fields) {
    addInputStream(fields, std::move(mKernelName + "_InputStream_" + std::to_string(mInputStream.size())));
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getInputStream
 ** ------------------------------------------------------------------------------------------------------------- */
Value * KernelBuilder::getInputStream(Value * const instance, const unsigned index, const unsigned streamOffset) {
    assert (index < mInputStream.size());
    return getInputStream(instance, iBuilder->getInt32(index), streamOffset);
}

Value * KernelBuilder::getInputStream(Value * const instance, disable_implicit_conversion<Value *> index, const unsigned streamOffset) {
    assert (instance && index);
    assert (index->getType()->isIntegerTy());
    Value * const inputStreamSet = iBuilder->CreateLoad(iBuilder->CreateGEP(instance,
        {iBuilder->getInt32(0), iBuilder->getInt32(INPUT_STREAM_SET), iBuilder->getInt32(0)}));
    Value * modFunction = iBuilder->CreateLoad(iBuilder->CreateGEP(instance,
        {iBuilder->getInt32(0), iBuilder->getInt32(INPUT_STREAM_SET), iBuilder->getInt32(1)}));
    Value * offset = iBuilder->CreateLoad(getBlockNo(instance));
    if (streamOffset) {
        offset = iBuilder->CreateAdd(offset, ConstantInt::get(offset->getType(), streamOffset));
    }
    if (LLVM_LIKELY(isa<ConstantInt>(index.get()) || inputStreamSet->getType()->getPointerElementType()->isArrayTy())) {
        return iBuilder->CreateGEP(inputStreamSet, { iBuilder->CreateCall(modFunction, offset), index });
    } else {
        throw std::runtime_error("Cannot access the input stream with a non-constant value unless all input stream types are identical!");
    }
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
    addInputScalar(type, std::move(mKernelName + "_InputScalar_" + std::to_string(mInputScalar.size())));
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getInputScalar
 ** ------------------------------------------------------------------------------------------------------------- */
Value * KernelBuilder::getInputScalar(Value * const instance, const unsigned) {
    assert (instance);
    throw std::runtime_error("currently not supported!");
}

Value * KernelBuilder::getInputScalar(Value * const instance, disable_implicit_conversion<Value *>) {
    assert (instance);
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
    assert (index < mOutputStream.size());
    return getOutputStream(instance, iBuilder->getInt32(index), streamOffset);
}

Value * KernelBuilder::getOutputStream(Value * const instance, disable_implicit_conversion<Value *> index, const unsigned streamOffset) {
    assert (instance && index);
    assert (index->getType()->isIntegerTy());
    if (LLVM_LIKELY(isa<ConstantInt>(index.get()))) {
        return iBuilder->CreateGEP(instance, {iBuilder->getInt32(0), iBuilder->getInt32(OUTPUT_STREAM_SET), getStreamOffset(instance, streamOffset), index});
    } else {
        Value * const outputStreamSet = iBuilder->CreateGEP(instance, {iBuilder->getInt32(0), iBuilder->getInt32(OUTPUT_STREAM_SET)});
        if (LLVM_LIKELY(outputStreamSet->getType()->getPointerElementType()->isArrayTy())) {
            return iBuilder->CreateGEP(outputStreamSet, {getStreamOffset(instance, streamOffset), index});
        }
    }
    throw std::runtime_error("Cannot access the output stream with a non-constant value unless all output stream types are identical!");
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getOutputScalar
 ** ------------------------------------------------------------------------------------------------------------- */
Value * KernelBuilder::getOutputScalar(Value * const instance, const unsigned) {
    throw std::runtime_error("currently not supported!");
}

Value * KernelBuilder::getOutputScalar(Value * const instance, disable_implicit_conversion<Value *> ) {
    throw std::runtime_error("currently not supported!");
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief packDataTypes
 ** ------------------------------------------------------------------------------------------------------------- */
llvm::Type * KernelBuilder::packDataTypes(const std::vector<llvm::Type *> & types) {
    bool canPackIntoArray = !types.empty();
    for (Type * type : types) {
        if (type != types.front()) { // use canLosslesslyBitcastInto ?
            canPackIntoArray = false;
            break;
        }
    }
    if (canPackIntoArray) {
        return ArrayType::get(types.front(), types.size());
    } else {
        return StructType::get(mMod->getContext(), types);
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief prepareFunction
 ** ------------------------------------------------------------------------------------------------------------- */
Function * KernelBuilder::prepareFunction() {

    PointerType * modFunctionType = PointerType::get(FunctionType::get(iBuilder->getInt64Ty(), {iBuilder->getInt64Ty()}, false), 0);
    mInputStreamType = PointerType::get(packDataTypes(mInputStream), 0);
    mInputScalarType = PointerType::get(packDataTypes(mInputScalar), 0);
    mOutputStreamType = packDataTypes(mOutputStream);
    Type * outputScalarType = packDataTypes(mOutputScalar);
    Type * internalStateType = packDataTypes(mInternalState);
    Type * inputStateType = StructType::create(mMod->getContext(), { mInputStreamType, modFunctionType});
    Type * outputBufferType = ArrayType::get(mOutputStreamType, mBufferSize);
    mKernelStateType = StructType::create(mMod->getContext(), {internalStateType, inputStateType, outputBufferType, outputScalarType}, mKernelName);

    FunctionType * const functionType = FunctionType::get(iBuilder->getVoidTy(), {PointerType::get(mKernelStateType, 0)}, false);
    mDoBlock = Function::Create(functionType, GlobalValue::ExternalLinkage, mKernelName + "_DoBlock", mMod);
    mDoBlock->setCallingConv(CallingConv::C);   
    mDoBlock->setDoesNotCapture(1);
    mDoBlock->setDoesNotThrow();

    Function::arg_iterator args = mDoBlock->arg_begin();
    mKernelState = args++;
    mKernelState->setName("this");

    iBuilder->SetInsertPoint(BasicBlock::Create(mMod->getContext(), "entry", mDoBlock, 0));

    return mDoBlock;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief finalize
 ** ------------------------------------------------------------------------------------------------------------- */
void KernelBuilder::finalize() {

    // Finish the actual function
    Value * blockNo = getBlockNo();
    Value * value = iBuilder->CreateLoad(blockNo);
    value = iBuilder->CreateAdd(value, ConstantInt::get(value->getType(), 1));
    iBuilder->CreateStore(value, blockNo);
    iBuilder->CreateRetVoid();

    eliminateRedundantMemoryOperations(mDoBlock);

    // Generate the zero initializer
    PointerType * modFunctionType = PointerType::get(FunctionType::get(iBuilder->getInt64Ty(), {iBuilder->getInt64Ty()}, false), 0);
    FunctionType * constructorType = FunctionType::get(iBuilder->getVoidTy(), {PointerType::get(mKernelStateType, 0), mInputStreamType, modFunctionType}, false);

    mConstructor = Function::Create(constructorType, GlobalValue::ExternalLinkage, mKernelName + "_Constructor", mMod);
    mConstructor->setCallingConv(CallingConv::C);
    mDoBlock->setDoesNotCapture(1);
    mConstructor->addAttribute(AttributeSet::FunctionIndex, Attribute::InlineHint);
    mDoBlock->setDoesNotThrow();

    auto args = mConstructor->arg_begin();
    mKernelState = args++;
    mKernelState->setName("this");
    Value * const inputStream = args++;
    inputStream->setName("inputStream");
    Value * const modFunction = args++;
    modFunction->setName("modFunction");

    iBuilder->SetInsertPoint(BasicBlock::Create(mMod->getContext(), "entry", mConstructor, 0));
    for (unsigned i = 0; i < mInternalState.size(); ++i) {
        Type * const type = mInternalState[i];
        if (type->isSized()) {
            setInternalState(i, Constant::getNullValue(type));
        } else {
            Value * const ptr = getInternalState(i);
            Value * const size = iBuilder->CreatePtrDiff(iBuilder->CreateGEP(ptr, iBuilder->getInt32(1)), ptr);
            iBuilder->CallPrintInt(mKernelName + "_zeroinit_" + std::to_string(i), size);
            iBuilder->CreateMemSet(ptr, iBuilder->getInt8(0), size, 4);
        }
    }

    Value * const input = iBuilder->CreateGEP(mKernelState, {iBuilder->getInt32(0), iBuilder->getInt32(INPUT_STREAM_SET)});
    iBuilder->CreateStore(inputStream, iBuilder->CreateGEP(input, {iBuilder->getInt32(0), iBuilder->getInt32(0)}));
    iBuilder->CreateStore(modFunction, iBuilder->CreateGEP(input, {iBuilder->getInt32(0), iBuilder->getInt32(1)}));
    iBuilder->CreateRetVoid();

//    if (mOutputStreamType->getStructNumElements()) {
//        PointerType * outputStreamType = PointerType::get(mOutputStreamType, 0);
//        FunctionType * type = FunctionType::get(outputStreamType, {outputStreamType, PointerType::get(blockNo->getType(), 0)}, false);
//        mStreamSetFunction = Function::Create(type, Function::ExternalLinkage, mKernelName + "_StreamSet", mMod);
//        auto arg = mStreamSetFunction->arg_begin();
//        Value * stream = arg++;
//        stream->setName("stream");
//        mStreamSetFunction->addAttribute(1, Attribute::NoCapture);
//        mStreamSetFunction->addAttribute(2, Attribute::NoCapture);
//        mStreamSetFunction->addAttribute(AttributeSet::FunctionIndex, Attribute::InlineHint);
//        mStreamSetFunction->addAttribute(AttributeSet::FunctionIndex, Attribute::NoUnwind);
//        Value * offset = arg;
//        BasicBlock * entry = BasicBlock::Create(mMod->getContext(), "entry", mStreamSetFunction);
//        iBuilder->SetInsertPoint(entry);
//        if (mBufferSize != 1) {
//            offset = iBuilder->CreateLoad(offset);
//            if (isPowerOfTwo(mBufferSize)) {
//                offset = iBuilder->CreateAnd(offset, iBuilder->getInt64(mBufferSize - 1));
//            } else if (mBufferSize > 2) {
//                offset = iBuilder->CreateURem(offset, iBuilder->getInt64(mBufferSize));
//            }
//            stream = iBuilder->CreateGEP(stream, offset);
//        }
//        iBuilder->CreateRet(stream);
//    }

    iBuilder->ClearInsertionPoint();
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief eliminateRedundantMemoryOperations
 ** ------------------------------------------------------------------------------------------------------------- */
inline void KernelBuilder::eliminateRedundantMemoryOperations(Function * const function) {


}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief instantiate
 *
 * Generate a new instance of this kernel and call the default constructor to initialize it
 ** ------------------------------------------------------------------------------------------------------------- */
Instance * KernelBuilder::instantiate(std::pair<Value *, unsigned> && inputStream) {
    AllocaInst * const memory = iBuilder->CreateAlloca(mKernelStateType);
    Value * const indices[] = {iBuilder->getInt32(0), iBuilder->getInt32(OUTPUT_STREAM_SET)};
    Value * ptr = iBuilder->CreateGEP(std::get<0>(inputStream), indices);
    iBuilder->CreateCall3(mConstructor, memory, iBuilder->CreatePointerCast(ptr, mInputStreamType), CreateModFunction(std::get<1>(inputStream)));
    return new Instance(this, memory);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief instantiate
 *
 * Generate a new instance of this kernel and call the default constructor to initialize it
 ** ------------------------------------------------------------------------------------------------------------- */
Instance * KernelBuilder::instantiate(llvm::Value * const inputStream) {
    AllocaInst * const memory = iBuilder->CreateAlloca(mKernelStateType);
    iBuilder->CreateCall3(mConstructor, memory, iBuilder->CreatePointerCast(inputStream, mInputStreamType), CreateModFunction(0));
    return new Instance(this, memory);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief instantiate
 *
 * Generate a new instance of this kernel and call the default constructor to initialize it
 ** ------------------------------------------------------------------------------------------------------------- */
Instance * KernelBuilder::instantiate(std::initializer_list<llvm::Value *> inputStreams) {
    if (mInputStreamType->getStructNumElements() != inputStreams.size()) {
        throw std::runtime_error(mKernelName + ".instantiate expected " + std::to_string(inputStreams.size()) +
                                 "elements but was given " + std::to_string(mInputStreamType->getStructNumElements()));
    }
    AllocaInst * const memory = iBuilder->CreateAlloca(mKernelStateType);
    AllocaInst * inputStruct = iBuilder->CreateAlloca(mInputStreamType, 0);
    unsigned i = 0;
    for (Value * inputStream : inputStreams) {
        Value * ptr = iBuilder->CreateGEP(inputStruct, { iBuilder->getInt32(0), iBuilder->getInt32(i++)});
        iBuilder->CreateStore(inputStream, ptr);
    }
    iBuilder->CreateCall3(mConstructor, memory, iBuilder->CreatePointerCast(inputStruct, mInputStreamType), CreateModFunction(0));
    return new Instance(this, memory);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief CreateDoBlockCall
 ** ------------------------------------------------------------------------------------------------------------- */
void KernelBuilder::CreateDoBlockCall(Value * const instance) {
    assert (mDoBlock && instance);
    iBuilder->CreateCall(mDoBlock, instance);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief clearOutputStreamSet
 *
 * Zero out the i + streamOffset stream set memory, where i is the current stream set indicated by the BlockNo.
 ** ------------------------------------------------------------------------------------------------------------- */
void KernelBuilder::clearOutputStreamSet(Value * const instance, const unsigned streamOffset) {
    Value * const indices[] = {iBuilder->getInt32(0), iBuilder->getInt32(OUTPUT_STREAM_SET), getStreamOffset(instance, streamOffset)};
    Value * ptr = iBuilder->CreateGEP(instance, indices);
    unsigned size = 0;
    for (unsigned i = 0; i < mOutputStreamType->getStructNumElements(); ++i) {
        size += mOutputStreamType->getStructElementType(i)->getPrimitiveSizeInBits();
    }
    iBuilder->CreateMemSet(ptr, iBuilder->getInt8(0), size / 8, 4);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief offset
 *
 * Compute the stream index of the given offset value.
 ** ------------------------------------------------------------------------------------------------------------- */
Value * KernelBuilder::getStreamOffset(Value * const instance, const unsigned index) {
    Value * offset = nullptr;
    if (mBufferSize > 1) {
        offset = iBuilder->CreateLoad(getBlockNo(instance));
        if (index) {
            offset = iBuilder->CreateAdd(offset, iBuilder->getInt64(index));
        }
        if (isPowerOfTwo(mBufferSize)) {
            offset = iBuilder->CreateAnd(offset, iBuilder->getInt64(mBufferSize - 1));
        } else {
            offset = iBuilder->CreateURem(offset, iBuilder->getInt64(mBufferSize));
        }
    } else {
        offset = iBuilder->getInt64(index);
    }
    return offset;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief CreateModFunction
 *
 * Generate a "modulo" function that dictates the local offset of a given blockNo
 ** ------------------------------------------------------------------------------------------------------------- */
inline Function * KernelBuilder::CreateModFunction(const unsigned size) {
    const std::string name((size == 0) ? "continuous" : "finite" + std::to_string(size));
    Function * function = mMod->getFunction(name);
    if (function) {
        return function;
    }
    const auto ip = iBuilder->saveIP();
    FunctionType * type = FunctionType::get(iBuilder->getInt64Ty(), {iBuilder->getInt64Ty()}, false);
    function = Function::Create(type, Function::ExternalLinkage, name, mMod);
    Value * offset = function->arg_begin();
    offset->setName("index");
    BasicBlock * entry = BasicBlock::Create(mMod->getContext(), "entry", function);
    iBuilder->SetInsertPoint(entry);
    if (size) {
        if (size == 1) {
            offset = iBuilder->getInt64(0);
        } else if (isPowerOfTwo(size)) {
            offset = iBuilder->CreateAnd(offset, iBuilder->getInt64(size - 1));
        } else {
            offset = iBuilder->CreateURem(offset, iBuilder->getInt64(size));
        }
    }
    iBuilder->CreateRet(offset);
    iBuilder->restoreIP(ip);
    return function;
}

} // end of namespace kernel
