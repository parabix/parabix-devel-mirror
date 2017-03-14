/*
 *  Copyright (c) 2016 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 */

#include "pipeline.h"
#include <toolchain.h>
#include <kernels/kernel.h>
#include <kernels/streamset.h>
#include <llvm/IR/Module.h>
#include <unordered_map>

using namespace kernel;
using namespace parabix;
using namespace llvm;

#include <iostream>

using ProducerTable = std::vector<std::vector<std::pair<unsigned, unsigned>>>;

ProducerTable createProducerTable(const std::vector<KernelBuilder *> & kernels) {
    ProducerTable producerTable(kernels.size());
    
    std::vector<std::vector<bool>> userTable(kernels.size());
    
    // First prepare a map from streamSet output buffers to their producing kernel and output index.
    std::unordered_map<const StreamSetBuffer *, std::pair<unsigned, unsigned>> bufferMap;
    
    for (unsigned k = 0; k < kernels.size(); k++) {
        auto outputSets = kernels[k]->getStreamSetOutputBuffers();
        for (unsigned j = 0; j < outputSets.size(); j++) {
            userTable[k].push_back(false);
            bufferMap.insert(std::make_pair(outputSets[j], std::make_pair(k, j)));
        }
    }
    for (unsigned k = 0; k < kernels.size(); k++) {
        auto inputSets = kernels[k]->getStreamSetInputBuffers();
        for (unsigned i = 0; i < inputSets.size(); i++) {
            auto f = bufferMap.find(inputSets[i]);
            if (f == bufferMap.end()) {
                llvm::report_fatal_error("Pipeline error: input buffer #" + std::to_string(i) + " of " + kernels[k]->getName() + ": no corresponding output buffer. ");
            }
            producerTable[k].push_back(f->second);
            unsigned sourceKernel, outputIndex;
            std::tie(sourceKernel, outputIndex) = f->second;
            if (sourceKernel >= k) {
                llvm::report_fatal_error("Pipeline error: input buffer #" + std::to_string(i) + " of " + kernels[k]->getName() + ": not defined before use. ");
            }
            //errs() << "sourceKernel: " + std::to_string(sourceKernel) + ", outputIndex: " + std::to_string(outputIndex) + ", user: " + std::to_string(k) + "\n";
            userTable[sourceKernel][outputIndex]= true;
            
        }
    }
    /*  TODO:  define sinks for  all outputs so that the following check succeeds on
     *  well-structured pipelines. 
    for (unsigned k = 0; k < kernels.size(); k++) {
        auto outputSets = kernels[k]->getStreamSetOutputBuffers();
        //errs() << "kernel: " + kernels[k]->getName() + "\n";
        for (unsigned j = 0; j < outputSets.size(); j++) {
            if (userTable[k][j] == false) {
                llvm::report_fatal_error("Pipeline error: output buffer #" + std::to_string(j) + " of " + kernels[k]->getName() + ": no users. ");
            }
        }
    }
    */
    return producerTable;
}

using ConsumerTable = std::vector<std::vector<std::vector<unsigned>>>;

ConsumerTable createConsumerTable(const std::vector<KernelBuilder *> & kernels) {
    ConsumerTable consumerTable(kernels.size());
    
    // First prepare a map from streamSet input buffers to their consuming kernel and input index.
    std::unordered_map<const StreamSetBuffer *, std::vector<unsigned>> bufferMap;
    
    for (unsigned k = 0; k < kernels.size(); k++) {
        auto inputSets = kernels[k]->getStreamSetInputBuffers();
        for (unsigned j = 0; j < inputSets.size(); j++) {
            auto f = bufferMap.find(inputSets[j]);
            std::vector<unsigned> kernelNo;
            kernelNo.push_back(k);
            if (f == bufferMap.end()) {
                bufferMap.insert(std::make_pair(inputSets[j], kernelNo));
            }
            else{
                f->second.push_back(k);
            }
        }
    }
    for (unsigned k = 0; k < kernels.size(); k++) {
        auto outputSets = kernels[k]->getStreamSetOutputBuffers();
        for (unsigned i = 0; i < outputSets.size(); i++) {
            auto f = bufferMap.find(outputSets[i]);
            if (f == bufferMap.end()) {
                llvm::report_fatal_error("Pipeline error: output buffer #" + std::to_string(i) + " of " + kernels[k]->getName() + ": not used by any kernel. ");
            }
            else {
                consumerTable[k].push_back(f->second);  
            }          
        }
    }
    return consumerTable;
}

Function * generateSegmentParallelPipelineThreadFunction(std::string name, IDISA::IDISA_Builder * iBuilder, const std::vector<KernelBuilder *> & kernels, Type * sharedStructType, ProducerTable & producerTable, int id) {
    
    // ProducerPos[k][i] will hold the producedItemCount of the i^th output stream
    // set of the k^th kernel.  These values will be loaded immediately after the
    // doSegment and finalSegment calls for kernel k and later used as the
    // producer position arguments for later doSegment/finalSegment calls.
    
    std::vector<std::vector<Value *>> ProducerPos;
    
    
    const auto ip = iBuilder->saveIP();
    
    Module * m = iBuilder->getModule();
    Type * const voidTy = iBuilder->getVoidTy();
    PointerType * const voidPtrTy = iBuilder->getVoidPtrTy();
    PointerType * const int8PtrTy = iBuilder->getInt8PtrTy();

    Function * const threadFunc = cast<Function>(m->getOrInsertFunction(name, voidTy, int8PtrTy, nullptr));
    threadFunc->setCallingConv(CallingConv::C);
    Function::arg_iterator args = threadFunc->arg_begin();

    Value * const input = &*(args++);
    input->setName("input");


     // Create the basic blocks for the thread function.
    BasicBlock * entryBlock = BasicBlock::Create(iBuilder->getContext(), "entry", threadFunc, 0);
    BasicBlock * segmentLoop = BasicBlock::Create(iBuilder->getContext(), "segmentLoop", threadFunc, 0);
    BasicBlock * exitThreadBlock = BasicBlock::Create(iBuilder->getContext(), "exitThread", threadFunc, 0);
    
    std::vector<BasicBlock *> segmentWait;
    std::vector<BasicBlock *> segmentLoopBody;
    for (unsigned i = 0; i < kernels.size(); i++) {
        auto kname = kernels[i]->getName();
        segmentWait.push_back(BasicBlock::Create(iBuilder->getContext(), kname + "Wait", threadFunc, 0));
        segmentLoopBody.push_back(BasicBlock::Create(iBuilder->getContext(), kname + "Do", threadFunc, 0));
    }

    iBuilder->SetInsertPoint(entryBlock);
    
    Value * sharedStruct = iBuilder->CreateBitCast(input, PointerType::get(sharedStructType, 0));
    std::vector<Value *> instancePtrs;
    for (unsigned k = 0; k < kernels.size(); k++) {
        Value * ptr = iBuilder->CreateGEP(sharedStruct, {iBuilder->getInt32(0), iBuilder->getInt32(k)});
        instancePtrs.push_back(iBuilder->CreateLoad(ptr));
    }
    
    iBuilder->CreateBr(segmentLoop);

    iBuilder->SetInsertPoint(segmentLoop);
    PHINode * segNo = iBuilder->CreatePHI(iBuilder->getSizeTy(), 2, "segNo");
    segNo->addIncoming(iBuilder->getSize(id), entryBlock);
    const unsigned last_kernel = kernels.size() - 1;
    Value * doFinal = ConstantInt::getNullValue(iBuilder->getInt1Ty());
    Value * nextSegNo = iBuilder->CreateAdd(segNo, iBuilder->getSize(1));
    iBuilder->CreateBr(segmentWait[0]);
    for (unsigned k = 0; k < kernels.size(); k++) {
        iBuilder->SetInsertPoint(segmentWait[k]);
        unsigned waitForKernel = k;
        if (codegen::DebugOptionIsSet(codegen::SerializeThreads)) {
            waitForKernel = last_kernel;
        }
        Value * processedSegmentCount = kernels[waitForKernel]->acquireLogicalSegmentNo(instancePtrs[waitForKernel]);
        Value * ready = iBuilder->CreateICmpEQ(segNo, processedSegmentCount);

        if (kernels[k]->hasNoTerminateAttribute()) {
            iBuilder->CreateCondBr(ready, segmentLoopBody[k], segmentWait[k]);
        } else { // If the kernel was terminated in a previous segment then the pipeline is done.
            BasicBlock * completionTest = BasicBlock::Create(iBuilder->getContext(), kernels[k]->getName() + "Completed", threadFunc, 0);
            BasicBlock * exitBlock = BasicBlock::Create(iBuilder->getContext(), kernels[k]->getName() + "Exit", threadFunc, 0);
            iBuilder->CreateCondBr(ready, completionTest, segmentWait[k]);
            iBuilder->SetInsertPoint(completionTest);
            Value * alreadyDone = kernels[k]->getTerminationSignal(instancePtrs[k]);
            iBuilder->CreateCondBr(alreadyDone, exitBlock, segmentLoopBody[k]);
            iBuilder->SetInsertPoint(exitBlock);
            // Ensure that the next thread will also exit.
            kernels[k]->releaseLogicalSegmentNo(instancePtrs[k], nextSegNo);
            iBuilder->CreateBr(exitThreadBlock);
        }
        iBuilder->SetInsertPoint(segmentLoopBody[k]);
        std::vector<Value *> doSegmentArgs = {instancePtrs[k], doFinal};
        for (unsigned j = 0; j < kernels[k]->getStreamInputs().size(); j++) {
            unsigned producerKernel, outputIndex;
            std::tie(producerKernel, outputIndex) = producerTable[k][j];
            doSegmentArgs.push_back(ProducerPos[producerKernel][outputIndex]);
        }
        kernels[k]->createDoSegmentCall(doSegmentArgs);
        std::vector<Value *> produced;
        for (unsigned i = 0; i < kernels[k]->getStreamOutputs().size(); i++) {
            produced.push_back(kernels[k]->getProducedItemCount(instancePtrs[k], kernels[k]->getStreamOutputs()[i].name));
        }
        ProducerPos.push_back(produced);
        if (! (kernels[k]->hasNoTerminateAttribute())) {
            Value * terminated = kernels[k]->getTerminationSignal(instancePtrs[k]);
            doFinal = iBuilder->CreateOr(doFinal, terminated);
        }

        kernels[k]->releaseLogicalSegmentNo(instancePtrs[k], nextSegNo);
        if (k == last_kernel) {
            segNo->addIncoming(iBuilder->CreateAdd(segNo, iBuilder->getSize(codegen::ThreadNum)), segmentLoopBody[last_kernel]);
            iBuilder->CreateCondBr(doFinal, exitThreadBlock, segmentLoop);
        } else {
            iBuilder->CreateBr(segmentWait[k+1]);
        }
    }

    iBuilder->SetInsertPoint(exitThreadBlock);
    Value * nullVal = Constant::getNullValue(voidPtrTy);
    iBuilder->CreatePThreadExitCall(nullVal);
    iBuilder->CreateRetVoid();
    iBuilder->restoreIP(ip);

    return threadFunc;
}

// Given a computation expressed as a logical pipeline of K kernels k0, k_1, ...k_(K-1)
// operating over an input stream set S, a segment-parallel implementation divides the input 
// into segments and coordinates a set of T <= K threads to each process one segment at a time.   
// Let S_0, S_1, ... S_N be the segments of S.   Segments are assigned to threads in a round-robin
// fashion such that processing of segment S_i by the full pipeline is carried out by thread i mod T.


void generateSegmentParallelPipeline(IDISA::IDISA_Builder * iBuilder, const std::vector<KernelBuilder *> & kernels) {
    
    const unsigned threadNum = codegen::ThreadNum;
    
    Module * m = iBuilder->getModule();
    
    IntegerType * const size_ty = iBuilder->getSizeTy();
    PointerType * const voidPtrTy = iBuilder->getVoidPtrTy();
    PointerType * const int8PtrTy = iBuilder->getInt8PtrTy();
    
    for (auto k : kernels) k->createInstance();
    
    ProducerTable producerTable = createProducerTable(kernels);
    
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
    for (unsigned i = 0; i < kernels.size(); i++) {
        kernels[i]->releaseLogicalSegmentNo(kernels[i]->getInstance(), iBuilder->getSize(0));
    }

    std::vector<Function *> thread_functions;
    const auto ip = iBuilder->saveIP();
    for (unsigned i = 0; i < threadNum; i++) {
        thread_functions.push_back(generateSegmentParallelPipelineThreadFunction("thread"+std::to_string(i), iBuilder, kernels, sharedStructType, producerTable, i));
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

Function * generateParallelPipelineThreadFunction(std::string name, IDISA::IDISA_Builder * iBuilder, const std::vector<KernelBuilder *> & kernels, Type * sharedStructType, ProducerTable & producerTable, ConsumerTable & consumerTable, int id) {
        
    const auto ip = iBuilder->saveIP();
    
    Module * m = iBuilder->getModule();
    Type * const voidTy = iBuilder->getVoidTy();
    IntegerType * const size_ty = iBuilder->getSizeTy();
    PointerType * const voidPtrTy = iBuilder->getVoidPtrTy();
    PointerType * const int8PtrTy = iBuilder->getInt8PtrTy();
    IntegerType * const int1ty = iBuilder->getInt1Ty();

    Function * const threadFunc = cast<Function>(m->getOrInsertFunction(name, voidTy, int8PtrTy, nullptr));
    threadFunc->setCallingConv(CallingConv::C);
    Function::arg_iterator args = threadFunc->arg_begin();

    Value * const input = &*(args++);
    input->setName("input");

    KernelBuilder * targetK = kernels[id];
    Value * bufferSegments = ConstantInt::get(size_ty, codegen::BufferSegments - 1);
    ConstantInt * segmentItems = iBuilder->getSize(codegen::SegmentSize * iBuilder->getBitBlockWidth());
    Value * waitCondTest = nullptr;

     // Create the basic blocks for the thread function.
    BasicBlock * entryBlock = BasicBlock::Create(iBuilder->getContext(), "entry", threadFunc, 0);
    BasicBlock * outputCheckBlock = BasicBlock::Create(iBuilder->getContext(), "outputCheck", threadFunc, 0);
    BasicBlock * doSegmentBlock = BasicBlock::Create(iBuilder->getContext(), "doSegment", threadFunc, 0);
    BasicBlock * exitThreadBlock = BasicBlock::Create(iBuilder->getContext(), "exitThread", threadFunc, 0);

    iBuilder->SetInsertPoint(entryBlock);
    
    Value * sharedStruct = iBuilder->CreateBitCast(input, PointerType::get(sharedStructType, 0));
    std::vector<Value *> instancePtrs;
    std::vector<std::vector<Value *>> ProducerPos;
    for (unsigned k = 0; k < kernels.size(); k++) {
        Value * ptr = iBuilder->CreateGEP(sharedStruct, {iBuilder->getInt32(0), iBuilder->getInt32(k)});
        instancePtrs.push_back(iBuilder->CreateLoad(ptr));

        std::vector<Value *> produced;
        for (unsigned i = 0; i < kernels[k]->getStreamOutputs().size(); i++) {
            produced.push_back(kernels[k]->getProducedItemCount(instancePtrs[k], kernels[k]->getStreamOutputs()[i].name));
        }
        ProducerPos.push_back(produced);
    }

    iBuilder->CreateBr(outputCheckBlock);

    iBuilder->SetInsertPoint(outputCheckBlock);
    PHINode * segNo = iBuilder->CreatePHI(iBuilder->getSizeTy(), 2, "segNo");
    segNo->addIncoming(iBuilder->getSize(0), entryBlock);
    segNo->addIncoming(segNo, outputCheckBlock);

    waitCondTest = ConstantInt::get(int1ty, 1);
    for (unsigned j = 0; j < targetK->getStreamOutputs().size(); j++) {
        std::vector<unsigned> consumerKernels = consumerTable[id][j];
        for (unsigned k = 0; k < consumerKernels.size(); k++) {
            Value * consumerSegNo = kernels[consumerKernels[k]]->acquireLogicalSegmentNo(instancePtrs[consumerKernels[k]]);
            waitCondTest = iBuilder->CreateAnd(waitCondTest, iBuilder->CreateICmpULE(segNo, iBuilder->CreateAdd(consumerSegNo, bufferSegments)));
        } 
    }

    if(targetK->getStreamInputs().size() == 0) {

        iBuilder->CreateCondBr(waitCondTest, doSegmentBlock, outputCheckBlock); 

        iBuilder->SetInsertPoint(doSegmentBlock);

        Value * terminated = targetK->getTerminationSignal(instancePtrs[id]);
        std::vector<Value *> doSegmentArgs = {instancePtrs[id], terminated};       
        targetK->createDoSegmentCall(doSegmentArgs);
        Value * nextSegNo = iBuilder->CreateAdd(segNo, iBuilder->getSize(1));
        segNo->addIncoming(nextSegNo, doSegmentBlock);
        targetK->releaseLogicalSegmentNo(instancePtrs[id], nextSegNo);

        iBuilder->CreateCondBr(terminated, exitThreadBlock, outputCheckBlock);

    }
    else{

        BasicBlock * inputCheckBlock = BasicBlock::Create(iBuilder->getContext(), "inputCheck", threadFunc, 0);

        iBuilder->CreateCondBr(waitCondTest, inputCheckBlock, outputCheckBlock); 

        iBuilder->SetInsertPoint(inputCheckBlock); 
        
        waitCondTest = ConstantInt::get(int1ty, 1);
        for (unsigned j = 0; j < targetK->getStreamInputs().size(); j++) {
            unsigned producerKernel, outputIndex;
            std::tie(producerKernel, outputIndex) = producerTable[id][j];
            Value * producerSegNo = kernels[producerKernel]->acquireLogicalSegmentNo(instancePtrs[producerKernel]);
            waitCondTest = iBuilder->CreateAnd(waitCondTest, iBuilder->CreateICmpULT(segNo, producerSegNo));  
        }

        iBuilder->CreateCondBr(waitCondTest, doSegmentBlock, inputCheckBlock);

        iBuilder->SetInsertPoint(doSegmentBlock);

        Value * nextSegNo = iBuilder->CreateAdd(segNo, iBuilder->getSize(1));
        Value * terminated = ConstantInt::get(int1ty, 1);
        for (unsigned j = 0; j < targetK->getStreamInputs().size(); j++) {
            unsigned producerKernel, outputIndex;
            std::tie(producerKernel, outputIndex) = producerTable[id][j];
            terminated = iBuilder->CreateAnd(terminated, kernels[producerKernel]->getTerminationSignal(instancePtrs[producerKernel]));
            Value * producerSegNo = kernels[producerKernel]->acquireLogicalSegmentNo(instancePtrs[producerKernel]);
            terminated = iBuilder->CreateAnd(terminated, iBuilder->CreateICmpEQ(nextSegNo, producerSegNo));
        }
        
        std::vector<Value *> doSegmentArgs = {instancePtrs[id], terminated};
        for (unsigned j = 0; j < targetK->getStreamInputs().size(); j++) {
            unsigned producerKernel, outputIndex;
            std::tie(producerKernel, outputIndex) = producerTable[id][j];
            // doSegmentArgs.push_back(ProducerPos[producerKernel][outputIndex]);
            doSegmentArgs.push_back(iBuilder->CreateMul(segmentItems, segNo));
        }
        targetK->createDoSegmentCall(doSegmentArgs);
        segNo->addIncoming(nextSegNo, doSegmentBlock);
        targetK->releaseLogicalSegmentNo(instancePtrs[id], nextSegNo);

        iBuilder->CreateCondBr(terminated, exitThreadBlock, outputCheckBlock);
    }

    iBuilder->SetInsertPoint(exitThreadBlock);

    Value * nullVal = Constant::getNullValue(voidPtrTy);
    iBuilder->CreatePThreadExitCall(nullVal);
    iBuilder->CreateRetVoid();
    iBuilder->restoreIP(ip);

    return threadFunc;

}

void generateParallelPipeline(IDISA::IDISA_Builder * iBuilder, const std::vector<KernelBuilder *> & kernels) {
    const unsigned threadNum = kernels.size();
    
    Module * m = iBuilder->getModule();
    
    IntegerType * const size_ty = iBuilder->getSizeTy();
    PointerType * const voidPtrTy = iBuilder->getVoidPtrTy();
    PointerType * const int8PtrTy = iBuilder->getInt8PtrTy();
    
    for (auto k : kernels) k->createInstance();
    
    ProducerTable producerTable = createProducerTable(kernels);
    ConsumerTable consumerTable = createConsumerTable(kernels);
    
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
    for (unsigned i = 0; i < kernels.size(); i++) {
        kernels[i]->releaseLogicalSegmentNo(kernels[i]->getInstance(), iBuilder->getSize(0));
    }

    std::vector<Function *> thread_functions;
    const auto ip = iBuilder->saveIP();
    for (unsigned i = 0; i < threadNum; i++) {
        thread_functions.push_back(generateParallelPipelineThreadFunction("thread"+std::to_string(i), iBuilder, kernels, sharedStructType, producerTable, consumerTable, i));
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

void generatePipelineLoop(IDISA::IDISA_Builder * iBuilder, const std::vector<KernelBuilder *> & kernels) {
    for (auto k : kernels) {
        k->createInstance();
    }
    BasicBlock * entryBlock = iBuilder->GetInsertBlock();
    Function * main = entryBlock->getParent();

    // Create the basic blocks for the loop.
    BasicBlock * segmentLoop = BasicBlock::Create(iBuilder->getContext(), "segmentLoop", main, 0);
    BasicBlock * exitBlock = BasicBlock::Create(iBuilder->getContext(), "exitBlock", main, 0);
    
    ProducerTable producerTable = createProducerTable(kernels);
    
    // ProducerPos[k][i] will hold the producedItemCount of the i^th output stream
    // set of the k^th kernel.  These values will be loaded immediately after the
    // doSegment and finalSegment calls for kernel k and later used as the
    // producer position arguments for later doSegment/finalSegment calls.
    
    std::vector<std::vector<Value *>> ProducerPos;
    
    iBuilder->CreateBr(segmentLoop);
    iBuilder->SetInsertPoint(segmentLoop);

    Value * terminationFound = ConstantInt::getNullValue(iBuilder->getInt1Ty());
    for (unsigned k = 0; k < kernels.size(); k++) {
        Value * instance = kernels[k]->getInstance();
        std::vector<Value *> doSegmentArgs = {instance, terminationFound};
        for (unsigned j = 0; j < kernels[k]->getStreamInputs().size(); j++) {
            unsigned producerKernel, outputIndex;
            std::tie(producerKernel, outputIndex) = producerTable[k][j];
            doSegmentArgs.push_back(ProducerPos[producerKernel][outputIndex]);
        }
        kernels[k]->createDoSegmentCall(doSegmentArgs);
        if (! (kernels[k]->hasNoTerminateAttribute())) {
            Value * terminated = kernels[k]->getTerminationSignal(instance);
            terminationFound = iBuilder->CreateOr(terminationFound, terminated);
        }
        std::vector<Value *> produced;
        for (unsigned i = 0; i < kernels[k]->getStreamOutputs().size(); i++) {
            produced.push_back(kernels[k]->getProducedItemCount(instance, kernels[k]->getStreamOutputs()[i].name));
        }
        ProducerPos.push_back(produced);
        Value * segNo = kernels[k]->acquireLogicalSegmentNo(instance);
        kernels[k]->releaseLogicalSegmentNo(instance, iBuilder->CreateAdd(segNo, iBuilder->getSize(1)));
    }
    iBuilder->CreateCondBr(terminationFound, exitBlock, segmentLoop);
    iBuilder->SetInsertPoint(exitBlock);
}

    
