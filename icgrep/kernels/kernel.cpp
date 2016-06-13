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
, mBlockNoIndex(0)
, mKernelStateType(nullptr) {
    assert (mDefaultBufferSize > 0);
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

    unsigned KernelBuilder::addInternalState(llvm::Type * const type, std::string name) {
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
Value * KernelBuilder::getInternalStateInternal(Value * const kernelState, const std::string & name) {
    const auto f = mInternalStateNameMap.find(name);
    if (LLVM_UNLIKELY(f == mInternalStateNameMap.end())) {
        throw std::runtime_error("Kernel does not contain internal state " + name);
    }
    return getInternalStateInternal(kernelState, f->second);
}

Value * KernelBuilder::getInternalStateInternal(Value * const kernelState, disable_implicit_conversion<Value *> index) {
    assert (index->getType()->isIntegerTy());
    assert (kernelState->getType()->getPointerElementType() == mKernelStateType);
    return iBuilder->CreateGEP(kernelState, {iBuilder->getInt32(0), index});
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief setInternalState
 ** ------------------------------------------------------------------------------------------------------------- */
void KernelBuilder::setInternalStateInternal(Value * const kernelState, const std::string & name, Value * const value) {
    Value * ptr = getInternalStateInternal(kernelState, name);
    assert (ptr->getType()->getPointerElementType() == value->getType());
    if (value->getType() == iBuilder->getBitBlockType()) {
        iBuilder->CreateBlockAlignedStore(value, ptr);
    } else {
        iBuilder->CreateStore(value, ptr);
    }
}

void KernelBuilder::setInternalStateInternal(Value * const kernelState, disable_implicit_conversion<Value *> index, Value * const value) {
    Value * ptr = getInternalStateInternal(kernelState, index);
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
    addInputStream(fields, mKernelName + "_InputStream_" + std::to_string(mInputStream.size()));
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getInputStream
 ** ------------------------------------------------------------------------------------------------------------- */
Value * KernelBuilder::getInputStreamInternal(Value * const inputStreamSet, disable_implicit_conversion<Value *> index) {
    assert ("Parameters cannot be null!" && (inputStreamSet != nullptr && index != nullptr));
    assert ("Stream index must be an integer!" && index->getType()->isIntegerTy());
    assert ("Illegal input stream set provided!" && inputStreamSet->getType()->getPointerElementType() == mInputStreamType);
    if (LLVM_LIKELY(isa<ConstantInt>(index.get()) || getInputStreamType()->isArrayTy())) {
        return iBuilder->CreateGEP(inputStreamSet, { iBuilder->getInt32(0), index });
    }
    throw std::runtime_error("Cannot access the input stream with a non-constant value unless all input stream types are identical!");
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
 * @brief getOutputStream
 ** ------------------------------------------------------------------------------------------------------------- */
Value * KernelBuilder::getOutputStreamInternal(Value * const outputStreamSet, disable_implicit_conversion<Value *> index) {
    assert ("Parameters cannot be null!" && (outputStreamSet != nullptr && index != nullptr));
    assert ("Stream index must be an integer!" && index->getType()->isIntegerTy());
    assert ("Illegal output stream set provided!" && outputStreamSet->getType()->getPointerElementType() == getOutputStreamType());
    if (LLVM_LIKELY(isa<ConstantInt>(index.get()) || getOutputStreamType()->isArrayTy())) {
        return iBuilder->CreateGEP(outputStreamSet, { iBuilder->getInt32(0), index });
    }
    throw std::runtime_error("Cannot access the output stream with a non-constant value unless all output stream types are identical!");
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

    mBlockNoIndex = iBuilder->getInt32(addInternalState(iBuilder->getInt64Ty(), "BlockNo"));

    if (!mKernelStateType) {
        mKernelStateType = StructType::create(iBuilder->getContext(), mInternalState, mKernelName);
    }
    mInputStreamType = packDataTypes(mInputStream);
    mOutputStreamType = packDataTypes(mOutputStream);
    mInputStreamOffsets = inputStreamOffsets;

    std::vector<Type *> params;
    params.push_back(mKernelStateType->getPointerTo());
    if (mInputStreamType) {
        for (unsigned i = 0; i < mInputStreamOffsets.size(); ++i) {
            params.push_back(mInputStreamType->getPointerTo());
        }
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
    mKernelStateParam = &*(args++);
    mKernelStateParam->setName("this");
    if (mInputStreamType) {
        for (const unsigned offset : mInputStreamOffsets) {
            Value * const inputStreamSet = &*(args++);
            inputStreamSet->setName("inputStreamSet" + std::to_string(offset));
            mInputStreamParam.emplace(offset, inputStreamSet);
        }
    }
    if (mOutputStreamType) {
        mOutputStreamParam = &*args;
        mOutputStreamParam->setName("outputStreamSet");
    }
    iBuilder->SetInsertPoint(BasicBlock::Create(iBuilder->getContext(), "entry", mDoBlock, 0));
    return mDoBlock;
}

void KernelBuilder::setInstanceParameters(std::vector<ParameterBinding> parms) {
    mInstanceParameters = parms;
    mInstanceParametersOffset = mInternalState.size();
    for (auto binding : mInstanceParameters) {
        addInternalState(binding.parameterType, binding.parameterName);
    }
}


Function *  KernelBuilder::createInitMethod() {
    if (!mKernelStateType) {
        mKernelStateType = StructType::create(iBuilder->getContext(), mInternalState, mKernelName);
    }
    std::vector<Type *> initParameters = {PointerType::getUnqual(mKernelStateType)};
    for (auto binding : mInstanceParameters) {
        initParameters.push_back(binding.parameterType);
    }
    FunctionType * mInitFunctionType = FunctionType::get(iBuilder->getVoidTy(), initParameters, false);
    Function * mInitFunction = Function::Create(mInitFunctionType, GlobalValue::ExternalLinkage, mKernelName + "_Init", iBuilder->getModule());
    iBuilder->SetInsertPoint(BasicBlock::Create(iBuilder->getContext(), "Init_entry", mInitFunction, 0));

    Function::arg_iterator args = mInitFunction->arg_begin();
    Value * self = &*(args++);
    self->setName("self");
    for (auto binding : mInstanceParameters) {
        Value * parm = &*(args++);
        parm->setName(binding.parameterName);
    }

    iBuilder->CreateStore(Constant::getNullValue(mKernelStateType), self);
    args = mInitFunction->arg_begin();
    args++;   // skip self argument.
    for (auto binding : mInstanceParameters) {
        Value * parm = &*(args++);
        setInternalStateInternal(self, binding.parameterName, parm);
    }
    iBuilder->CreateRetVoid();
    return mInitFunction;
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
    mInputStreamParam.clear();
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
    AllocaInst * outputStreamSets = nullptr;
    if (mOutputStreamType) {
        outputStreamSets = iBuilder->CreateAlloca(mOutputStreamType, iBuilder->getInt32(outputBufferSize));
    }
    return new Instance(this, kernelState, std::get<0>(inputStreamSet), std::get<1>(inputStreamSet), outputStreamSets, outputBufferSize);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief instantiate
 *
 * Generate a new instance of this kernel and call the default constructor to initialize it
 ** ------------------------------------------------------------------------------------------------------------- */
Instance * KernelBuilder::instantiate(std::initializer_list<llvm::Value *> inputStreams) {   
    AllocaInst * inputStruct = iBuilder->CreateAlloca(mInputStreamType);
    unsigned i = 0;
    for (Value * inputStream : inputStreams) {
        Value * ptr = iBuilder->CreateGEP(inputStruct, { iBuilder->getInt32(0), iBuilder->getInt32(i++)});
        iBuilder->CreateStore(inputStream, ptr);
    }
    return instantiate(std::make_pair(inputStruct, 0));
}

Value * KernelBuilder::getInputStreamParam(const unsigned streamOffset) const {
    const auto f = mInputStreamParam.find(streamOffset);
    if (LLVM_UNLIKELY(f == mInputStreamParam.end())) {
        throw std::runtime_error("Kernel compilation error: No input stream parameter for stream offset " + std::to_string(streamOffset));
    }
    return f->second;
}
    
llvm::Value * make_New(IDISA::IDISA_Builder * iBuilder, std::string kernel_name, std::vector<Value *> args) {
    Module * m = iBuilder->getModule();
    Type * kernelType = m->getTypeByName(kernel_name);
    if (!kernelType) {
        throw std::runtime_error("Cannot find kernel type " + kernel_name);
    }
    Value * kernelInstance = iBuilder->CreateAlloca(kernelType);
    std::vector<Value *> init_args = {kernelInstance};
    for (auto a : args) {
        init_args.push_back(a);
    }
    //iBuilder->CreateStore(Constant::getNullValue(kernelType), kernelInstance);
    Function * initMethod = m->getFunction(kernel_name + "_Init");
    if (!initMethod) {
        //throw std::runtime_error("Cannot find " + kernel_name + "_Init");
        iBuilder->CreateStore(Constant::getNullValue(kernelType), kernelInstance);
        return kernelInstance;
    }
    iBuilder->CreateCall(initMethod, init_args);
    return kernelInstance;
}
    llvm::Value * make_DoBlock_Call(IDISA::IDISA_Builder * iBuilder, std::string kernel_name, std::vector<Value *> args) {
        Module * m = iBuilder->getModule();
        Function * doBlockMethod = m->getFunction(kernel_name + "_DoBlock");
        if (!doBlockMethod) {
            throw std::runtime_error("Cannot find " + kernel_name + "_DoBlock");
        }
        return iBuilder->CreateCall(doBlockMethod, args);
    }
    
    llvm::Value * make_FinalBlock_Call(IDISA::IDISA_Builder * iBuilder, std::string kernel_name, std::vector<Value *> args) {
        Module * m = iBuilder->getModule();
        Function * finalBlockMethod = m->getFunction(kernel_name + "_FinalBlock");
        if (!finalBlockMethod) {
            throw std::runtime_error("Cannot find " + kernel_name + "_FinalBlock");
        }
        return iBuilder->CreateCall(finalBlockMethod, args);
    }
    
    

} // end of namespace kernel
