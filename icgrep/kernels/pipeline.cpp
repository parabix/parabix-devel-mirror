/*
 *  Copyright (c) 2016 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 */

#include "pipeline.h"
#include <toolchain.h>
#include <kernels/kernel.h>
#include <kernels/streamset.h>
#include <llvm/IR/Module.h>
#include <boost/container/flat_map.hpp>

using namespace kernel;
using namespace parabix;
using namespace llvm;

using ProducerTable = std::vector<std::vector<std::pair<unsigned, unsigned>>>;

using ConsumerTable = std::vector<std::vector<std::vector<unsigned>>>;

template <typename Value>
using StreamSetBufferMap = boost::container::flat_map<const StreamSetBuffer *, Value>;


ProducerTable createProducerTable(const std::vector<KernelBuilder *> & kernels) {
    // map each output streamSet to its producing kernel and output index.
    StreamSetBufferMap<std::pair<unsigned, unsigned>> map;
    for (unsigned k = 0; k < kernels.size(); k++) {
        const auto & outputSets = kernels[k]->getStreamSetOutputBuffers();
        for (unsigned j = 0; j < outputSets.size(); j++) {
            map.emplace(outputSets[j], std::make_pair(k, j));
        }
    }
    // TODO: replace this with a sparse matrix? it would be easier to understand that the i,j-th element indicated kernel i's input was from the j-th kernel
    ProducerTable producerTable(kernels.size());
    for (unsigned k = 0; k < kernels.size(); k++) {
        const KernelBuilder * const kernel = kernels[k];
        const auto & inputSets = kernel->getStreamSetInputBuffers();
        for (unsigned i = 0; i < inputSets.size(); i++) {
            const auto f = map.find(inputSets[i]);
            if (LLVM_UNLIKELY(f == map.end())) {
                report_fatal_error("Pipeline error: input buffer #" + std::to_string(i) + " of " + kernel->getName() + ": no corresponding output buffer. ");
            }
            unsigned sourceKernel, outputIndex;
            std::tie(sourceKernel, outputIndex) = f->second;
            producerTable[k].emplace_back(sourceKernel, outputIndex);
            if (LLVM_UNLIKELY(sourceKernel >= k)) {
                report_fatal_error("Pipeline error: input buffer #" + std::to_string(i) + " of " + kernel->getName() + ": not defined before use. ");
            }
        }
    }
    return producerTable;
}

ConsumerTable createConsumerTable(const std::vector<KernelBuilder *> & kernels) {
    // map each input streamSet to its set of consuming kernels
    StreamSetBufferMap<std::vector<unsigned>> map;
    for (unsigned k = 0; k < kernels.size(); k++) {
        const auto & inputSets = kernels[k]->getStreamSetInputBuffers();
        for (const StreamSetBuffer * inputSet : inputSets) {
            auto f = map.find(inputSet);
            if (f == map.end()) {
                map.emplace(inputSet, std::vector<unsigned>({k}));
            } else {
                f->second.push_back(k);
            }
        }
    }
    ConsumerTable consumerTable(kernels.size());
    for (unsigned k = 0; k < kernels.size(); k++) {
        const auto & outputSets = kernels[k]->getStreamSetOutputBuffers();
        for (const StreamSetBuffer * outputSet : outputSets) {
            auto f = map.find(outputSet);
            if (LLVM_LIKELY(f != map.end())) {
                consumerTable[k].emplace_back(std::move(f->second));
            }          
        }
    }
    return consumerTable;
}

Function * generateSegmentParallelPipelineThreadFunction(std::string name, IDISA::IDISA_Builder * iBuilder, const std::vector<KernelBuilder *> & kernels, Type * sharedStructType, const ProducerTable & producerTable, int id) {
    
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

        KernelBuilder * const K = kernels[k];

        if (K->hasNoTerminateAttribute()) {
            iBuilder->CreateCondBr(ready, segmentLoopBody[k], segmentWait[k]);
        } else { // If the kernel was terminated in a previous segment then the pipeline is done.
            BasicBlock * completionTest = BasicBlock::Create(iBuilder->getContext(), K->getName() + "Completed", threadFunc, 0);
            BasicBlock * exitBlock = BasicBlock::Create(iBuilder->getContext(), K->getName() + "Exit", threadFunc, 0);
            iBuilder->CreateCondBr(ready, completionTest, segmentWait[k]);
            iBuilder->SetInsertPoint(completionTest);
            Value * alreadyDone = K->getTerminationSignal(instancePtrs[k]);
            iBuilder->CreateCondBr(alreadyDone, exitBlock, segmentLoopBody[k]);
            iBuilder->SetInsertPoint(exitBlock);
            // Ensure that the next thread will also exit.
            K->releaseLogicalSegmentNo(instancePtrs[k], nextSegNo);
            iBuilder->CreateBr(exitThreadBlock);
        }
        iBuilder->SetInsertPoint(segmentLoopBody[k]);
        std::vector<Value *> args = {instancePtrs[k], doFinal};
        for (unsigned j = 0; j < K->getStreamInputs().size(); j++) {
            unsigned producerKernel, outputIndex;
            std::tie(producerKernel, outputIndex) = producerTable[k][j];
            args.push_back(ProducerPos[producerKernel][outputIndex]);
        }
        K->createDoSegmentCall(args);
         if (! (K->hasNoTerminateAttribute())) {
            Value * terminated = K->getTerminationSignal(instancePtrs[k]);
            doFinal = iBuilder->CreateOr(doFinal, terminated);
        }
        std::vector<Value *> produced;
        for (unsigned i = 0; i < K->getStreamOutputs().size(); i++) {
            produced.push_back(K->getProducedItemCount(instancePtrs[k], K->getStreamOutputs()[i].name, doFinal));
        }
        ProducerPos.push_back(produced);

        K->releaseLogicalSegmentNo(instancePtrs[k], nextSegNo);
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
        
    Module * m = iBuilder->getModule();
    
    IntegerType * const size_ty = iBuilder->getSizeTy();
    PointerType * const voidPtrTy = iBuilder->getVoidPtrTy();
    PointerType * const int8PtrTy = iBuilder->getInt8PtrTy();
    
    for (auto k : kernels) k->createInstance();
    
    const ProducerTable producerTable = createProducerTable(kernels);
    
    Type * const pthreadsTy = ArrayType::get(size_ty, codegen::ThreadNum);
    AllocaInst * const pthreads = iBuilder->CreateAlloca(pthreadsTy);
    std::vector<Value *> pthreadsPtrs;
    for (int i = 0; i < codegen::ThreadNum; i++) {
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
    for (int i = 0; i < codegen::ThreadNum; i++) {
        thread_functions.push_back(generateSegmentParallelPipelineThreadFunction("thread"+std::to_string(i), iBuilder, kernels, sharedStructType, producerTable, i));
    }
    iBuilder->restoreIP(ip);
    
    for (int i = 0; i < codegen::ThreadNum; i++) {
        iBuilder->CreatePThreadCreateCall(pthreadsPtrs[i], nullVal, thread_functions[i], iBuilder->CreateBitCast(sharedStruct, int8PtrTy));
    }
    
    std::vector<Value *> threadIDs;
    for (int i = 0; i < codegen::ThreadNum; i++) {
        threadIDs.push_back(iBuilder->CreateLoad(pthreadsPtrs[i]));
    }
    
    for (int i = 0; i < codegen::ThreadNum; i++) {
        iBuilder->CreatePThreadJoinCall(threadIDs[i], status);
    }
    
}

Function * generateParallelPipelineThreadFunction(std::string name, IDISA::IDISA_Builder * iBuilder, const std::vector<KernelBuilder *> & kernels, Type * sharedStructType, const ProducerTable & producerTable, const ConsumerTable & consumerTable, const unsigned id) {
        
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

    KernelBuilder * const targetK = kernels[id];
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
        KernelBuilder * K = kernels[k];

        Value * ptr = iBuilder->CreateGEP(sharedStruct, {iBuilder->getInt32(0), iBuilder->getInt32(k)});
        instancePtrs.push_back(iBuilder->CreateLoad(ptr));

        std::vector<Value *> produced;
        for (unsigned i = 0; i < K->getStreamOutputs().size(); i++) {
            produced.push_back(K->getProducedItemCount(instancePtrs[k], K->getStreamOutputs()[i].name));
        }
        ProducerPos.push_back(produced);
    }

    iBuilder->CreateBr(outputCheckBlock);

    iBuilder->SetInsertPoint(outputCheckBlock);
    PHINode * segNo = iBuilder->CreatePHI(iBuilder->getSizeTy(), 2, "segNo");
    segNo->addIncoming(iBuilder->getSize(0), entryBlock);
    segNo->addIncoming(segNo, outputCheckBlock);

    waitCondTest = ConstantInt::getTrue(int1ty);
    for (unsigned j = 0; j < targetK->getStreamOutputs().size(); j++) {
        const auto & consumerKernels = consumerTable[id][j];
        for (unsigned k = 0; k < consumerKernels.size(); k++) {
            Value * consumerSegNo = kernels[consumerKernels[k]]->acquireLogicalSegmentNo(instancePtrs[consumerKernels[k]]);
            waitCondTest = iBuilder->CreateAnd(waitCondTest, iBuilder->CreateICmpULE(segNo, iBuilder->CreateAdd(consumerSegNo, bufferSegments)));
        } 
    }

    if (targetK->getStreamInputs().empty()) {

        iBuilder->CreateCondBr(waitCondTest, doSegmentBlock, outputCheckBlock); 

        iBuilder->SetInsertPoint(doSegmentBlock);

        Value * terminated = targetK->getTerminationSignal(instancePtrs[id]);
        std::vector<Value *> doSegmentArgs = {instancePtrs[id], terminated};       
        targetK->createDoSegmentCall(doSegmentArgs);
        Value * nextSegNo = iBuilder->CreateAdd(segNo, iBuilder->getSize(1));
        segNo->addIncoming(nextSegNo, doSegmentBlock);
        targetK->releaseLogicalSegmentNo(instancePtrs[id], nextSegNo);

        iBuilder->CreateCondBr(terminated, exitThreadBlock, outputCheckBlock);

    } else {

        BasicBlock * inputCheckBlock = BasicBlock::Create(iBuilder->getContext(), "inputCheck", threadFunc, 0);

        iBuilder->CreateCondBr(waitCondTest, inputCheckBlock, outputCheckBlock); 

        iBuilder->SetInsertPoint(inputCheckBlock); 
        
        waitCondTest = ConstantInt::getTrue(int1ty);
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
    
    for (auto k : kernels) {
        k->createInstance();
    }
    
    const ProducerTable producerTable = createProducerTable(kernels);
    const ConsumerTable consumerTable = createConsumerTable(kernels);
    
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
        KernelBuilder * const K = kernels[i];
        K->releaseLogicalSegmentNo(K->getInstance(), iBuilder->getSize(0));
    }

    std::vector<Function *> thread_functions;
    const auto ip = iBuilder->saveIP();
    for (unsigned i = 0; i < threadNum; i++) {
        thread_functions.push_back(generateParallelPipelineThreadFunction("thread" + std::to_string(i), iBuilder, kernels, sharedStructType, producerTable, consumerTable, i));
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
    
    const ProducerTable producer = createProducerTable(kernels);

 //   const ConsumerTable consumer = createConsumerTable(kernels);
    
    // ProducerPos[k][i] will hold the producedItemCount of the i^th output stream
    // set of the k^th kernel.  These values will be loaded immediately after the
    // doSegment and finalSegment calls for kernel k and later used as the
    // producer position arguments for later doSegment/finalSegment calls.
    
    std::vector<std::vector<Value *>> ProducerPos;
    
    iBuilder->CreateBr(segmentLoop);
    iBuilder->SetInsertPoint(segmentLoop);

    Value * terminated = ConstantInt::getFalse(iBuilder->getContext());
    for (unsigned k = 0; k < kernels.size(); k++) {
        KernelBuilder * const kernel = kernels[k];
        Value * const instance = kernel->getInstance();
        std::vector<Value *> args = {instance, terminated};
        for (unsigned i = 0; i < kernel->getStreamInputs().size(); ++i) {
            unsigned producerKernel, outputIndex;
            std::tie(producerKernel, outputIndex) = producer[k][i];
            args.push_back(ProducerPos[producerKernel][outputIndex]);
        }
        for (unsigned i = 0; i < kernel->getStreamOutputs().size(); ++i) {
            args.push_back(iBuilder->getSize(0));
        }
        kernel->createDoSegmentCall(args);
        if (!kernel->hasNoTerminateAttribute()) {
            terminated = iBuilder->CreateOr(terminated, kernel->getTerminationSignal(instance));
        }
        std::vector<Value *> produced;
        const auto & streamOutputs = kernel->getStreamOutputs();
        for (unsigned i = 0; i < streamOutputs.size(); i++) {
            produced.push_back(kernel->getProducedItemCount(instance, streamOutputs[i].name, terminated));
        }
        ProducerPos.push_back(produced);
        Value * const segNo = kernel->acquireLogicalSegmentNo(instance);
        kernel->releaseLogicalSegmentNo(instance, iBuilder->CreateAdd(segNo, iBuilder->getSize(1)));
    }

    iBuilder->CreateCondBr(terminated, exitBlock, segmentLoop);
    iBuilder->SetInsertPoint(exitBlock);
}
