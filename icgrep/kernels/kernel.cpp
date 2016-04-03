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

namespace kernel {

// sets name & sets internal state to the kernel superclass state
KernelBuilder::KernelBuilder(IDISA::IDISA_Builder * builder, std::string && name, const unsigned defaultBufferSize)
: iBuilder(builder)
, mKernelName(name)
, mDefaultBufferSize(defaultBufferSize)
, mBitBlockType(builder->getBitBlockType())
, mBlockNoIndex(0) {
    assert (mDefaultBufferSize > 0);
    mBlockNoIndex = iBuilder->getInt32(addInternalState(builder->getInt64Ty(), "BlockNo"));
}

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
    mInternalStateNameMap.emplace(name, iBuilder->getInt32(index));
    return index;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getInternalState
 ** ------------------------------------------------------------------------------------------------------------- */
Value * KernelBuilder::getInternalState(Value * const kernelState, disable_implicit_conversion<Value *> index) {
    assert (index->getType()->isIntegerTy());    
    assert (kernelState->getType()->getPointerElementType() == mKernelStateType);
    return iBuilder->CreateGEP(kernelState, {iBuilder->getInt32(0), index});
}

Value * KernelBuilder::getInternalState(Value * const kernelState, const std::string & name) {
    const auto f = mInternalStateNameMap.find(name);
    if (LLVM_UNLIKELY(f == mInternalStateNameMap.end())) {
        throw std::runtime_error("Kernel does not contain internal state " + name);
    }
    return getInternalState(kernelState, f->second);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief setInternalState
 ** ------------------------------------------------------------------------------------------------------------- */
void KernelBuilder::setInternalState(Value * const kernelState, const std::string & name, Value * const value) {
    Value * ptr = getInternalState(kernelState, name);
    assert (ptr->getType()->getPointerElementType() == value->getType());
    if (value->getType() == iBuilder->getBitBlockType()) {
        iBuilder->CreateBlockAlignedStore(value, ptr);
    } else {
        iBuilder->CreateStore(value, ptr);
    }
}

void KernelBuilder::setInternalState(Value * const kernelState, disable_implicit_conversion<Value *> index, Value * const value) {
    Value * ptr = getInternalState(kernelState, index);
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
Value * KernelBuilder::getInputStream(Value * const inputStreamSet, disable_implicit_conversion<Value *> index) {
    assert ("Parameters cannot be null!" && (inputStreamSet != nullptr && index != nullptr));
    assert ("Stream index must be an integer!" && index->getType()->isIntegerTy());
    assert ("Illegal input stream set provided!" && inputStreamSet->getType()->getPointerElementType() == mInputStreamType);
    if (LLVM_LIKELY(isa<ConstantInt>(index.get()) || getInputStreamType()->isArrayTy())) {
        return iBuilder->CreateGEP(inputStreamSet, { iBuilder->getInt32(0), index });
    }
    #ifndef NDEBUG
    iBuilder->getModule()->dump();
    #endif
    throw std::runtime_error("Cannot access the input stream with a non-constant value unless all input stream types are identical!");
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
Value * KernelBuilder::getInputScalar(Value * const inputScalarSet, disable_implicit_conversion<Value *>) {
    assert (inputScalarSet);
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
Value * KernelBuilder::getOutputStream(Value * const outputStreamSet, disable_implicit_conversion<Value *> index) {
    assert ("Parameters cannot be null!" && (outputStreamSet != nullptr && index != nullptr));
    assert ("Stream index must be an integer!" && index->getType()->isIntegerTy());
    assert ("Illegal output stream set provided!" && outputStreamSet->getType()->getPointerElementType() == getOutputStreamType());
    if (LLVM_LIKELY(isa<ConstantInt>(index.get()) || getOutputStreamType()->isArrayTy())) {
        return iBuilder->CreateGEP(outputStreamSet, { iBuilder->getInt32(0), index });
    }
    throw std::runtime_error("Cannot access the output stream with a non-constant value unless all output stream types are identical!");
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getOutputScalar
 ** ------------------------------------------------------------------------------------------------------------- */
Value * KernelBuilder::getOutputScalar(Value * const outputScalarSet, disable_implicit_conversion<Value *> ) {
    throw std::runtime_error("currently not supported!");
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief packDataTypes
 ** ------------------------------------------------------------------------------------------------------------- */
Type * KernelBuilder::packDataTypes(const std::vector<llvm::Type *> & types) {
    if (types.empty()) {
        return nullptr;
    }
    for (Type * type : types) {
        if (type != types.front()) { // use canLosslesslyBitcastInto ?
            return StructType::get(iBuilder->getContext(), types);
        }
    }
    return ArrayType::get(types.front(), types.size());
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief prepareFunction
 ** ------------------------------------------------------------------------------------------------------------- */
Function * KernelBuilder::prepareFunction(std::vector<unsigned> && inputStreamOffsets) {

    mKernelStateType = StructType::create(iBuilder->getContext(), mInternalState, mKernelName);
    mInputScalarType = packDataTypes(mInputScalar);
    mInputStreamType = packDataTypes(mInputStream);
    mOutputScalarType = packDataTypes(mInputScalar);
    mOutputStreamType = packDataTypes(mOutputStream);
    mInputStreamOffsets = inputStreamOffsets;

    std::vector<Type *> params;
    params.push_back(mKernelStateType->getPointerTo());
    if (mInputScalarType) {
        params.push_back(mInputScalarType->getPointerTo());
    }
    if (mInputStreamType) {
        for (unsigned i = 0; i < mInputStreamOffsets.size(); ++i) {
            params.push_back(mInputStreamType->getPointerTo());
        }
    }
    if (mOutputScalarType) {
        params.push_back(mOutputScalarType->getPointerTo());
    }
    if (mOutputStreamType) {
        params.push_back(mOutputStreamType->getPointerTo());
    }

    // A pointer value is captured if the function makes a copy of any part of the pointer that outlives
    // the call (e.g., stored in a global or, depending on the context, when returned by the function.)
    // Since this does not occur in either our DoBlock or Constructor, all parameters are marked nocapture.

    FunctionType * const functionType = FunctionType::get(iBuilder->getVoidTy(), params, false);
    mDoBlock = Function::Create(functionType, GlobalValue::ExternalLinkage, mKernelName + "_DoBlock", iBuilder->getModule());
    mDoBlock->setCallingConv(CallingConv::C);
    for (unsigned i = 1; i <= params.size(); ++i) {
        mDoBlock->setDoesNotCapture(i);
    }
    mDoBlock->setDoesNotThrow();
    Function::arg_iterator args = mDoBlock->arg_begin();
    mKernelStateParam = args++;
    mKernelStateParam->setName("this");
    if (mInputScalarType) {
        mInputScalarParam = args++;
        mInputScalarParam->setName("inputScalars");
    }
    if (mInputStreamType) {
        for (const unsigned offset : mInputStreamOffsets) {
            Value * const inputStreamSet = args++;
            inputStreamSet->setName("inputStreamSet" + std::to_string(offset));
            mInputStreamParam.emplace(offset, inputStreamSet);
        }
    }
    if (mOutputScalarType) {
        mOutputScalarParam = args++;
        mOutputScalarParam->setName("outputScalars");
    }
    if (mOutputStreamType) {
        mOutputStreamParam = args;
        mOutputStreamParam->setName("outputStreamSet");
    }
    iBuilder->SetInsertPoint(BasicBlock::Create(iBuilder->getContext(), "entry", mDoBlock, 0));
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

    mKernelStateParam = nullptr;
    mInputScalarParam = nullptr;
    mInputStreamParam.clear();
    mOutputScalarParam = nullptr;
    mOutputStreamParam = nullptr;
    iBuilder->ClearInsertionPoint();
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief instantiate
 *
 * Allocate and zero initialize the memory for this kernel and its output scalars and streams
 ** ------------------------------------------------------------------------------------------------------------- */
Instance * KernelBuilder::instantiate(std::pair<Value *, unsigned> && inputStreamSet, const unsigned outputBufferSize) {
    AllocaInst * const kernelState = iBuilder->CreateAlloca(mKernelStateType);
    iBuilder->CreateStore(Constant::getNullValue(mKernelStateType), kernelState);
    AllocaInst * outputScalars = nullptr;
    if (mOutputScalarType) {
        outputScalars = iBuilder->CreateAlloca(mOutputScalarType);
    }
    AllocaInst * outputStreamSets = nullptr;
    if (mOutputStreamType) {
        outputStreamSets = iBuilder->CreateAlloca(mOutputStreamType, iBuilder->getInt32(outputBufferSize));
    }
    return new Instance(this, kernelState, nullptr, std::get<0>(inputStreamSet), std::get<1>(inputStreamSet), outputScalars, outputStreamSets, outputBufferSize);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief instantiate
 *
 * Generate a new instance of this kernel and call the default constructor to initialize it
 ** ------------------------------------------------------------------------------------------------------------- */
Instance * KernelBuilder::instantiate(std::initializer_list<llvm::Value *> inputStreams) {   
    throw std::runtime_error("Not supported!");
//    AllocaInst * inputStruct = iBuilder->CreateAlloca(mInputStreamType);
//    unsigned i = 0;
//    for (Value * inputStream : inputStreams) {
//        Value * ptr = iBuilder->CreateGEP(inputStruct, { iBuilder->getInt32(0), iBuilder->getInt32(i++)});
//        iBuilder->CreateStore(iBuilder->CreatePointerCast(inputStream, ptr);
//    }
//    return instantiate(std::make_pair(inputStruct, 0));
}

} // end of namespace kernel
