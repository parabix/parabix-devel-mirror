/*
 *  Copyright (c) 2016 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 */

#include "pipeline.h"
#include <toolchain.h>
#include <IR_Gen/idisa_builder.h>
#include <kernels/interface.h>
#include <kernels/kernel.h>
#include <kernels/s2p_kernel.h>
#include <iostream>
#include <unordered_map>

using namespace kernel;
using namespace parabix;
using namespace llvm;

#if 0

using BufferMap = std::unordered_map<StreamSetBuffer *, std::pair<KernelBuilder *, unsigned>>;

static void createStreamBufferMap(BufferMap & bufferMap, const std::vector<KernelBuilder *> & kernels) {
    for (auto k: kernels) {
        auto outputSets = k->getStreamSetOutputBuffers();
        for (unsigned i = 0; i < outputSets.size(); i++) {
            bufferMap.insert(std::make_pair(outputSets[i], std::make_pair(k, i)));
        }
    }
    for (auto k: kernels) {
        auto inputSets = k->getStreamSetInputBuffers();
        for (unsigned i = 0; i < inputSets.size(); i++) {
            if (bufferMap.find(inputSets[i]) == bufferMap.end()) {
                llvm::report_fatal_error("Pipeline error: input buffer #" + std::to_string(i) + " of " + k->getName() + ": no corresponding output buffer. ");
            }
        }
    }
}

static Value * getSegmentBlocks(BufferMap & bufferMap, KernelBuilder * kernel) {
    IDISA::IDISA_Builder * iBuilder = kernel->getBuilder();
    std::cerr << "getSegmentBlocks\n"; 

    KernelBuilder * sourceKernel;

    unsigned outputIndex;
    auto inputs = kernel->getStreamSetInputBuffers();
    if (inputs.empty()) return iBuilder->getSize(codegen::SegmentSize * iBuilder->getStride() / iBuilder->getBitBlockWidth());
    std::string inputSetName = kernel->getStreamInputs()[0].name;
    std::cerr << "inputSetName = " << inputSetName << "\n"; 
    auto f = bufferMap.find(inputs[0]);
    assert(f != bufferMap.end()  && "bufferMap failure");
    std::tie(sourceKernel, outputIndex) = f->second;
    std::cerr << "outputIndex = " << outputIndex << "\n"; 
    Value * produced = sourceKernel->getProducedItemCount(sourceKernel->getInstance(), sourceKernel->getStreamOutputs()[outputIndex].name);
    iBuilder->CallPrintInt("produced", produced);
    Value * processed = kernel->getProcessedItemCount(kernel->getInstance(), inputSetName);
    iBuilder->CallPrintInt("processed", processed);
    Value * itemsToDo = iBuilder->CreateSub(produced, processed);
    return iBuilder->CreateUDiv(itemsToDo, iBuilder->getSize(iBuilder->getStride()));
}

#endif

Function * generateSegmentParallelPipelineThreadFunction(std::string name, IDISA::IDISA_Builder * iBuilder, const std::vector<KernelBuilder *> & kernels, Type * sharedStructType, int id) {

    Module * m = iBuilder->getModule();
    Type * const size_ty = iBuilder->getSizeTy();
    Type * const voidTy = iBuilder->getVoidTy();
    Type * const voidPtrTy = iBuilder->getVoidPtrTy();
    Type * const int8PtrTy = iBuilder->getInt8PtrTy();

    Function * const threadFunc = cast<Function>(m->getOrInsertFunction(name, voidTy, int8PtrTy, nullptr));
    threadFunc->setCallingConv(CallingConv::C);
    Function::arg_iterator args = threadFunc->arg_begin();

    Value * const input = &*(args++);
    input->setName("input");

    unsigned threadNum = codegen::ThreadNum;

     // Create the basic blocks for the thread function.
    BasicBlock * entryBlock = BasicBlock::Create(iBuilder->getContext(), "entry", threadFunc, 0);
    BasicBlock * segmentLoop = BasicBlock::Create(iBuilder->getContext(), "segmentLoop", threadFunc, 0);
    BasicBlock * exitThreadBlock = BasicBlock::Create(iBuilder->getContext(), "exitThread", threadFunc, 0);
    
    std::vector<BasicBlock *> segmentWait;
    std::vector<BasicBlock *> segmentLoopBody;
    std::vector<BasicBlock *> partialSegmentWait;
    std::vector<BasicBlock *> partialSegmentLoopBody;
    bool terminationSignalEncountered = false;
    for (unsigned i = 0; i < kernels.size(); i++) {
        std::string kname = kernels[i]->getName();
        segmentWait.push_back(BasicBlock::Create(iBuilder->getContext(), kname + "Wait", threadFunc, 0));
        segmentLoopBody.push_back(BasicBlock::Create(iBuilder->getContext(), "do_" + kname, threadFunc, 0));
        if (terminationSignalEncountered) {
            partialSegmentWait.push_back(BasicBlock::Create(iBuilder->getContext(), kname + "WaitFinal", threadFunc, 0));
            partialSegmentLoopBody.push_back(BasicBlock::Create(iBuilder->getContext(), "finish_" + kname, threadFunc, 0));
        }
        else {
            partialSegmentWait.push_back(nullptr);
            partialSegmentLoopBody.push_back(nullptr);
            terminationSignalEncountered = kernels[i]->hasNoTerminateAttribute() == false;
        }
    }
    segmentWait.push_back(segmentLoop); // If the last kernel does not terminate, loop back.
    partialSegmentWait.push_back(exitThreadBlock); // After the last kernel terminates, we're done.

    iBuilder->SetInsertPoint(entryBlock);
    Value * sharedStruct = iBuilder->CreateBitCast(input, PointerType::get(sharedStructType, 0));
    Constant * myThreadId = ConstantInt::get(size_ty, id);
    std::vector<Value *> instancePtrs;
    for (unsigned i = 0; i < kernels.size(); i++) {
        Value * ptr = iBuilder->CreateGEP(sharedStruct, {iBuilder->getInt32(0), iBuilder->getInt32(i)});
        instancePtrs.push_back(iBuilder->CreateLoad(ptr));
    }
    
    // Some important constant values.
    int segmentSize = codegen::SegmentSize;
    Constant * segmentBlocks = ConstantInt::get(size_ty, segmentSize);
    iBuilder->CreateBr(segmentLoop);

    iBuilder->SetInsertPoint(segmentLoop);
    PHINode * segNo = iBuilder->CreatePHI(size_ty, 2, "segNo");
    segNo->addIncoming(myThreadId, entryBlock);
    Value * nextSegNo = iBuilder->CreateAdd(segNo, iBuilder->getSize(1));
    unsigned last_kernel = kernels.size() - 1;
    Value * alreadyDone = kernels[last_kernel]->getTerminationSignal(instancePtrs[last_kernel]);
    iBuilder->CreateCondBr(alreadyDone, exitThreadBlock, segmentWait[0]);

    
    
    for (unsigned i = 0; i < kernels.size(); i++) {
        iBuilder->SetInsertPoint(segmentWait[i]);
        Value * processedSegmentCount = kernels[i]->acquireLogicalSegmentNo(instancePtrs[i]);
        Value * cond = iBuilder->CreateICmpEQ(segNo, processedSegmentCount);
        iBuilder->CreateCondBr(cond, segmentLoopBody[i], segmentWait[i]);

        iBuilder->SetInsertPoint(segmentLoopBody[i]);
        if (i == last_kernel) {
            segNo->addIncoming(iBuilder->CreateAdd(segNo, ConstantInt::get(size_ty, threadNum)), segmentLoopBody[last_kernel]);
        }
        kernels[i]->createDoSegmentCall(instancePtrs[i], segmentBlocks);
        if (kernels[i]->hasNoTerminateAttribute()) {
            kernels[i]->releaseLogicalSegmentNo(instancePtrs[i], nextSegNo);
            iBuilder->CreateBr(segmentWait[i+1]);
        }
        else {
            Value * terminated = kernels[i]->getTerminationSignal(instancePtrs[i]);
            kernels[i]->releaseLogicalSegmentNo(instancePtrs[i], nextSegNo);
            iBuilder->CreateCondBr(terminated, partialSegmentWait[i+1], segmentWait[i+1]);
        }
        if (partialSegmentWait[i] != nullptr) {
            iBuilder->SetInsertPoint(partialSegmentWait[i]);
            Value * processedSegmentCount = kernels[i]->acquireLogicalSegmentNo(instancePtrs[i]);
            Value * cond = iBuilder->CreateICmpEQ(segNo, processedSegmentCount);
            iBuilder->CreateCondBr(cond, partialSegmentLoopBody[i], partialSegmentWait[i]);
            
            iBuilder->SetInsertPoint(partialSegmentLoopBody[i]);
            kernels[i]->createFinalSegmentCall(instancePtrs[i], segmentBlocks);
            kernels[i]->releaseLogicalSegmentNo(instancePtrs[i], nextSegNo);
            iBuilder->CreateBr(partialSegmentWait[i+1]);
        }
    }
   
    iBuilder->SetInsertPoint(exitThreadBlock);
    Value * nullVal = Constant::getNullValue(voidPtrTy);
    iBuilder->CreatePThreadExitCall(nullVal);
    iBuilder->CreateRetVoid();

    return threadFunc;
}

// Given a computation expressed as a logical pipeline of K kernels k0, k_1, ...k_(K-1)
// operating over an input stream set S, a segment-parallel implementation divides the input 
// into segments and coordinates a set of T <= K threads to each process one segment at a time.   
// Let S_0, S_1, ... S_N be the segments of S.   Segments are assigned to threads in a round-robin
// fashion such that processing of segment S_i by the full pipeline is carried out by thread i mod T.


void generateSegmentParallelPipeline(IDISA::IDISA_Builder * iBuilder, const std::vector<KernelBuilder *> & kernels) {
    
    unsigned threadNum = codegen::ThreadNum;

    Module * m = iBuilder->getModule();

    Type * const size_ty = iBuilder->getSizeTy();
    Type * const voidPtrTy = iBuilder->getVoidPtrTy();
    Type * const int8PtrTy = iBuilder->getInt8PtrTy();

    for (auto k : kernels) k->createInstance();

    Type * const pthreadsTy = ArrayType::get(size_ty, threadNum);
    AllocaInst * const pthreads = iBuilder->CreateAlloca(pthreadsTy);
    std::vector<Value *> pthreadsPtrs;
    for (unsigned i = 0; i < threadNum; i++) {
        pthreadsPtrs.push_back(iBuilder->CreateGEP(pthreads, {iBuilder->getInt32(0), iBuilder->getInt32(i)}));
    }
    Value * nullVal = Constant::getNullValue(voidPtrTy);
    AllocaInst * const status = iBuilder->CreateAlloca(int8PtrTy);

    std::vector<Type *> structTypes;
    for (unsigned i = 0; i < kernels.size(); i++) {
        structTypes.push_back(kernels[i]->getInstance()->getType());
    }
    Type * sharedStructType = StructType::get(m->getContext(), structTypes);

    AllocaInst * sharedStruct = iBuilder->CreateAlloca(sharedStructType);
    for (unsigned i = 0; i < kernels.size(); i++) {
        Value * ptr = iBuilder->CreateGEP(sharedStruct, {iBuilder->getInt32(0), iBuilder->getInt32(i)});
        iBuilder->CreateStore(kernels[i]->getInstance(), ptr);
    }

    std::vector<Function *> thread_functions;
    const auto ip = iBuilder->saveIP();
    for (unsigned i = 0; i < threadNum; i++) {
        thread_functions.push_back(generateSegmentParallelPipelineThreadFunction("thread"+std::to_string(i), iBuilder, kernels, sharedStructType, i));
    }
    iBuilder->restoreIP(ip);

    for (unsigned i = 0; i < threadNum; i++) {
        iBuilder->CreatePThreadCreateCall(pthreadsPtrs[i], nullVal, thread_functions[i], iBuilder->CreateBitCast(sharedStruct, int8PtrTy));
    }

    std::vector<Value *> threadIDs;
    for (unsigned i = 0; i < threadNum; i++) { 
        threadIDs.push_back(iBuilder->CreateLoad(pthreadsPtrs[i]));
    }
    
    for (unsigned i = 0; i < threadNum; i++) { 
        iBuilder->CreatePThreadJoinCall(threadIDs[i], status);
    }

}

void generatePipelineParallel(IDISA::IDISA_Builder * iBuilder, const std::vector<KernelBuilder *> & kernels) {
 
    Type * pthreadTy = iBuilder->getSizeTy();
    Type * const voidPtrTy = iBuilder->getVoidPtrTy();
    Type * const int8PtrTy = iBuilder->getInt8PtrTy();

    Type * const pthreadsTy = ArrayType::get(pthreadTy, kernels.size());

    for (auto k : kernels) k->createInstance();

    AllocaInst * const pthreads = iBuilder->CreateAlloca(pthreadsTy);
    std::vector<Value *> pthreadsPtrs;
    for (unsigned i = 0; i < kernels.size(); i++) {
        pthreadsPtrs.push_back(iBuilder->CreateGEP(pthreads, {iBuilder->getInt32(0), iBuilder->getInt32(i)}));
    }
    Value * nullVal = Constant::getNullValue(voidPtrTy);
    AllocaInst * const status = iBuilder->CreateAlloca(int8PtrTy);

    std::vector<Function *> kernel_functions;
    const auto ip = iBuilder->saveIP();
    for (unsigned i = 0; i < kernels.size(); i++) {
        kernel_functions.push_back(kernels[i]->generateThreadFunction("k_"+std::to_string(i)));
    }
    iBuilder->restoreIP(ip);

    for (unsigned i = 0; i < kernels.size(); i++) {
        iBuilder->CreatePThreadCreateCall(pthreadsPtrs[i], nullVal, kernel_functions[i], iBuilder->CreateBitCast(kernels[i]->getInstance(), int8PtrTy));
    }

    std::vector<Value *> threadIDs;
    for (unsigned i = 0; i < kernels.size(); i++) { 
        threadIDs.push_back(iBuilder->CreateLoad(pthreadsPtrs[i]));
    }
    
    for (unsigned i = 0; i < kernels.size(); i++) { 
        iBuilder->CreatePThreadJoinCall(threadIDs[i], status);
    }
}


void generatePipelineLoop(IDISA::IDISA_Builder * iBuilder, const std::vector<KernelBuilder *> & kernels) {
    for (auto k : kernels) k->createInstance();
    //BufferMap bufferMap;
    //createStreamBufferMap(bufferMap, kernels);
    
    BasicBlock * entryBlock = iBuilder->GetInsertBlock();
    Function * main = entryBlock->getParent();

    // Create the basic blocks.  
    BasicBlock * segmentLoop = BasicBlock::Create(iBuilder->getContext(), "segmentLoop", main, 0);
    BasicBlock * exitBlock = BasicBlock::Create(iBuilder->getContext(), "exitBlock", main, 0);
    // We create vectors of loop body and final segment blocks indexed by kernel.
    std::vector<BasicBlock *> loopBodyBlocks;
    std::vector<BasicBlock *> finalSegmentBlocks;

    loopBodyBlocks.push_back(segmentLoop); 
    finalSegmentBlocks.push_back(nullptr);  
    
    for (unsigned i = 1; i < kernels.size(); i++) {
        if (kernels[i-1]->hasNoTerminateAttribute()) {
            // Previous kernel cannot terminate.   Continue with the previous blocks;
            loopBodyBlocks.push_back(loopBodyBlocks.back());
            finalSegmentBlocks.push_back(finalSegmentBlocks.back());
        }
        else {
            loopBodyBlocks.push_back(BasicBlock::Create(iBuilder->getContext(), "do_" + kernels[i]->getName(), main, 0));
            finalSegmentBlocks.push_back(BasicBlock::Create(iBuilder->getContext(), "finish_" + kernels[i]->getName(), main, 0));
        }
    }
    loopBodyBlocks.push_back(segmentLoop); // If the last kernel does not terminate, loop back.
    finalSegmentBlocks.push_back(exitBlock); // If the last kernel does terminate, we're done.
    
    iBuilder->CreateBr(segmentLoop);
    Constant * segBlocks = iBuilder->getSize(codegen::SegmentSize * iBuilder->getStride() / iBuilder->getBitBlockWidth());
    for (unsigned i = 0; i < kernels.size(); i++) {
        iBuilder->SetInsertPoint(loopBodyBlocks[i]);
        //Value * segBlocks = getSegmentBlocks(bufferMap, kernels[i]);
        Value * segNo = kernels[i]->acquireLogicalSegmentNo(kernels[i]->getInstance());
        kernels[i]->createDoSegmentCall(kernels[i]->getInstance(), segBlocks);
        if (kernels[i]->hasNoTerminateAttribute()) {
            kernels[i]->releaseLogicalSegmentNo(kernels[i]->getInstance(), iBuilder->CreateAdd(segNo, iBuilder->getSize(1)));
            if (i == kernels.size() - 1) {
                iBuilder->CreateBr(segmentLoop);
            }
        }
        else {
            Value * terminated = kernels[i]->getTerminationSignal(kernels[i]->getInstance());
            kernels[i]->releaseLogicalSegmentNo(kernels[i]->getInstance(), iBuilder->CreateAdd(segNo, iBuilder->getSize(1)));
            iBuilder->CreateCondBr(terminated, finalSegmentBlocks[i+1], loopBodyBlocks[i+1]);
        }
        if (finalSegmentBlocks[i] != nullptr) {
            iBuilder->SetInsertPoint(finalSegmentBlocks[i]);
            Value * segNo = kernels[i]->acquireLogicalSegmentNo(kernels[i]->getInstance());
            kernels[i]->createFinalSegmentCall(kernels[i]->getInstance(), segBlocks);
            kernels[i]->releaseLogicalSegmentNo(kernels[i]->getInstance(), iBuilder->CreateAdd(segNo, iBuilder->getSize(1)));
            if (finalSegmentBlocks[i] != finalSegmentBlocks[i+1]) {
                iBuilder->CreateBr(finalSegmentBlocks[i+1]);
            }
        }
    }
    iBuilder->SetInsertPoint(exitBlock);
}
