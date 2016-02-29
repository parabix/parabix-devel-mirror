/*
 *  Copyright (c) 2016 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 */

#include "kernel.h"
#include <iostream>

// sets name & sets internal state to the kernel superclass state
KernelBuilder::KernelBuilder(std::string name, Module * m, IDISA::IDISA_Builder * b)
: mMod(m)
, iBuilder(b)
, mKernelName(name)
, mPredifinedStates(2)
, mBitBlockType(b->getBitBlockType())
, mBlockSize(b->getBitBlockWidth()){
    mStates = std::vector<Type *>(4, b->getIntNTy(64)); 
    mSegmentBlocks = 1;
    mBufferSize = mSegmentBlocks * mBlockSize;
}

KernelBuilder::~KernelBuilder(){
}

int KernelBuilder::extendKernelInternalStateType(Type * t){
    int idx = mStates.size();
    mStates.push_back(t);
    return idx;
}
void KernelBuilder::addKernelOutputStream(int fw){
    if (fw == 1){
        mOutputStreams.push_back(mBitBlockType);
    }
    else {
        mOutputStreams.push_back(ArrayType::get(mBitBlockType, fw));
    }

}
void KernelBuilder::addKernelOutputAccum(Type * t){
    mOutputAccums.push_back(t);

}
void KernelBuilder::addKernelInputStream(int fw, std::string name = ""){
    if (name=="")
        mInputStreamNames.push_back(mKernelName + "_inputstream_" + std::to_string(mInputStreams.size()));
    else
        mInputStreamNames.push_back(name);

    if (fw == 1){
        mInputStreams.push_back(mBitBlockType);
    }
    else {
        mInputStreams.push_back(ArrayType::get(mBitBlockType, fw));
    }
}
void KernelBuilder::addKernelInputScalar(Type * t, std::string name = ""){
    if (name=="")
        mInputScalarNames.push_back(mKernelName + "_inputscalar_" + std::to_string(mInputScalars.size()));
    else
        mInputScalarNames.push_back(name);

    mInputScalars.push_back(t);
}

void KernelBuilder::PrepareDoBlockFunction(){ 
    mInputStreamType = PointerType::get(ArrayType::get(StructType::get(mMod->getContext(), mInputStreams), mSegmentBlocks), 0);
    mInputScalarType = PointerType::get(StructType::get(mMod->getContext(), mInputScalars), 0);
    Type * outputStreamType = ArrayType::get(StructType::get(mMod->getContext(), mOutputStreams), mSegmentBlocks);
    Type * outputAccumType = StructType::get(mMod->getContext(), mOutputAccums);
    Type * stateType = StructType::create(mMod->getContext(), mStates, mKernelName);
    mKernelStructType = StructType::create(mMod->getContext(),std::vector<Type *>({stateType, outputStreamType, outputAccumType}), "KernelStruct_"+mKernelName);
}

struct Inputs KernelBuilder::openDoBlock(){
    // FunctionType * functionType = FunctionType::get(Type::getVoidTy(mMod->getContext()), 
    //     std::vector<Type *>({PointerType::get(mKernelStructType, 0), mInputStreamType, mInputScalarType}), false); 
       
    FunctionType * functionType = FunctionType::get(Type::getVoidTy(mMod->getContext()), 
        std::vector<Type *>({PointerType::get(mKernelStructType, 0), mInputStreamType}), false); 

    mDoBlockFunction = Function::Create(functionType, GlobalValue::ExternalLinkage, mKernelName + "_DoBlock", mMod);
    mDoBlockFunction->setCallingConv(CallingConv::C);
   
    Function::arg_iterator args = mDoBlockFunction->arg_begin();
    mKernelStructParam = args++;
    mKernelStructParam->setName("this");
    Value* input_stream_param = args++;
    input_stream_param->setName("input_stream");
    // Value* input_scalar_param = args++;
    // input_scalar_param->setName("input_scalar");

    iBuilder->SetInsertPoint(BasicBlock::Create(mMod->getContext(), "entry", mDoBlockFunction,0));

    struct Inputs inputs;
    for(int j = 0; j<mSegmentBlocks; j++){
        for(int i = 0; i<mInputStreams.size(); i++){
            Value* indices[] = {iBuilder->getInt64(0), iBuilder->getInt32(j), iBuilder->getInt32(i)};
            Value * gep = iBuilder->CreateGEP(input_stream_param, indices);
            Type * t = gep->getType()->getPointerElementType();
            if (t != mBitBlockType) {
                int arraySize = t->getArrayNumElements();
                inputs.streams.resize(mSegmentBlocks, std::vector<valptr>(arraySize));
                for (int k=0; k<arraySize; k++){
                    Value * gep_array_elem = iBuilder->CreateGEP(gep, {iBuilder->getInt32(0), iBuilder->getInt32(k)});
                    inputs.streams[j][k] = iBuilder->CreateAlignedLoad(gep_array_elem, mBlockSize/8, false, mInputStreamNames.at(i));
                }
            }
            else{
                inputs.streams.resize(mSegmentBlocks, std::vector<valptr>(mInputStreams.size()));
                inputs.streams[j][i] = iBuilder->CreateAlignedLoad(gep, mBlockSize/8, false, mInputStreamNames.at(i));
            }
            
        }
    }

    // inputs.scalars.resize(mInputScalars.size());
    // for(int i = 0; i<mInputScalars.size(); i++){
    //     Value* indices[] = {iBuilder->getInt64(0), iBuilder->getInt32(i)};
    //     Value * gep = iBuilder->CreateGEP(input_scalar_param, indices);
    //     inputs.scalars[i] = iBuilder->CreateAlignedLoad(gep, mBlockSize/8, false, mInputScalarNames.at(i));
    // }

    return inputs;
}

void KernelBuilder::closeDoBlock(struct Outputs result){
    

    for(int j=0; j<mSegmentBlocks; j++){
        for(int i = 0; i<mOutputStreams.size(); i++){   
            Value* indices[] = {iBuilder->getInt64(0), iBuilder->getInt32(1), iBuilder->getInt32(j), iBuilder->getInt32(i)};
            Value* gep = iBuilder->CreateGEP(mKernelStructParam, indices);
            iBuilder->CreateAlignedStore(result.streams[j][i], gep, mBlockSize/8, false);
        }
    }

    for(int i = 0; i<mOutputAccums.size(); i++){    
        Value* indices[] = {iBuilder->getInt64(0), iBuilder->getInt32(2), iBuilder->getInt32(i)};
        Value* gep = iBuilder->CreateGEP(mKernelStructParam, indices);
        iBuilder->CreateAlignedStore(result.accums[i], gep, mBlockSize/8, false);
    }

    iBuilder->CreateRetVoid();
}

void KernelBuilder::changeKernelInternalState(Value * kernelStruct, int idx, Value * stateValue){
    Value* indices[] = {iBuilder->getInt64(0), iBuilder->getInt32(0), iBuilder->getInt32(idx)};
    Value* gep = iBuilder->CreateGEP(kernelStruct, indices);
    iBuilder->CreateAlignedStore(stateValue, gep, mBlockSize/8, false);
}

Value * KernelBuilder::getKernelInternalState(Value * kernelStruct, int idx){
    Value* indices[] = {iBuilder->getInt64(0), iBuilder->getInt32(0), iBuilder->getInt32(idx)};
    Value* gep = iBuilder->CreateGEP(kernelStruct, indices);
    return iBuilder->CreateAlignedLoad(gep, mBlockSize/8, false, "state"+std::to_string(idx));
}

Value * KernelBuilder::getKernelInternalStatePtr(Value * kernelStruct, int idx){
    Value* indices[] = {iBuilder->getInt64(0), iBuilder->getInt32(0), iBuilder->getInt32(idx)};
    Value* gep = iBuilder->CreateGEP(kernelStruct, indices);
    return gep;
}

void KernelBuilder::finalizeMethods(){
    Type * T = iBuilder->getIntNTy(64);

    Constant* c = mMod->getOrInsertFunction(mKernelName+"_Init", Type::getVoidTy(mMod->getContext()), PointerType::get(mKernelStructType, 0), NULL);
    Function* mInitFunction = cast<Function>(c);
    mInitFunction->setCallingConv(CallingConv::C);
    Function::arg_iterator args = mInitFunction->arg_begin();

    Value* this_param = args++;
    this_param->setName("this");

    int i = mPredifinedStates;
    iBuilder->SetInsertPoint(BasicBlock::Create(mMod->getContext(), "entry", mInitFunction, 0));
    Value * gep = iBuilder->CreateGEP(this_param, std::vector<Value *>({ iBuilder->getInt32(0), iBuilder->getInt32(0), iBuilder->getInt32(i++) }));
    iBuilder->CreateStore(iBuilder->getInt64(0), gep);  //FileBasePos
    gep = iBuilder->CreateGEP(this_param, std::vector<Value *>({ iBuilder->getInt32(0), iBuilder->getInt32(0), iBuilder->getInt32(i++) }));
    iBuilder->CreateStore(iBuilder->getInt64(0), gep);  //AvailableBlocks

    while(i < mStates.size()){
        gep = iBuilder->CreateGEP(this_param, std::vector<Value *>({ iBuilder->getInt32(0), iBuilder->getInt32(0), iBuilder->getInt32(i++) }));
        Value * gep_next = iBuilder->CreateGEP(gep, std::vector<Value *>({iBuilder->getInt32(1)}));
        Value * get_int = iBuilder->CreatePtrToInt(gep, T);
        Value * get_next_int = iBuilder->CreatePtrToInt(gep_next, T);
        Value * state_size = iBuilder->CreateSub(get_next_int, get_int);
        iBuilder->CreateMemSet(gep, iBuilder->getInt8(0), state_size, 4);
    }

    iBuilder->CreateRetVoid();

    c = mMod->getOrInsertFunction(mKernelName+"_Create_Default", Type::getVoidTy(mMod->getContext()), PointerType::get(mKernelStructType, 0), T, T, NULL);
    mConstructor = cast<Function>(c);
    mConstructor->setCallingConv(CallingConv::C);
    args = mConstructor->arg_begin();

    this_param = args++;
    this_param->setName("this");
    Value* block_size_param = args++;
    block_size_param->setName("block_size");
    Value* seg_size_param = args++;
    seg_size_param->setName("seg_size");
 
    iBuilder->SetInsertPoint(BasicBlock::Create(mMod->getContext(), "entry", mConstructor, 0));
    gep = iBuilder->CreateGEP(this_param, std::vector<Value *>({ iBuilder->getInt32(0), iBuilder->getInt32(0), iBuilder->getInt32(0) }));
    iBuilder->CreateStore(block_size_param, gep);   
    gep = iBuilder->CreateGEP(this_param, std::vector<Value *>({ iBuilder->getInt32(0), iBuilder->getInt32(0), iBuilder->getInt32(1) }));
    iBuilder->CreateStore(seg_size_param, gep); 

    iBuilder->CreateCall(mInitFunction, this_param);
    iBuilder->CreateRetVoid();

}

Value * KernelBuilder::generateKernelInstance(){

    mKernelStruct = iBuilder->CreateAlloca(mKernelStructType); 
    iBuilder->CreateCall3(mConstructor, mKernelStruct, 
        ConstantInt::get(iBuilder->getIntNTy(64), mBlockSize), 
        ConstantInt::get(iBuilder->getIntNTy(64), mBufferSize));
    return mKernelStruct;

}
void KernelBuilder::generateInitCall(){
    iBuilder->CreateCall(mInitFunction, mKernelStruct);
}

// void KernelBuilder::generateDoBlockCall(Value * inputStreams, Value * inputScalars){
//     iBuilder->CreateCall3(mDoBlockFunction, mKernelStruct, inputStreams, inputScalars);
// }
void KernelBuilder::generateDoBlockCall(Value * inputStreams){
    iBuilder->CreateCall2(mDoBlockFunction, mKernelStruct, inputStreams);
}

int KernelBuilder::getSegmentBlocks(){
    return mSegmentBlocks;
}

Function * KernelBuilder::getDoBlockFunction(){
    return mDoBlockFunction;
}

Type * KernelBuilder::getKernelStructType(){
    return mKernelStructType;
}

Value * KernelBuilder::getKernelStructParam(){
    return mKernelStructParam;
}

