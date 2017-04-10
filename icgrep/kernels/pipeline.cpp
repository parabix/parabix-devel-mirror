/*
 *  Copyright (c) 2016 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 */

#include "pipeline.h"
#include <kernels/toolchain.h>
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

Function * makeThreadFunction(const std::string & name, Module * const m) {
    LLVMContext & C = m->getContext();
    Type * const voidTy = Type::getVoidTy(C);
    PointerType * const int8PtrTy = Type::getInt8PtrTy(C);
    Function * const f = Function::Create(FunctionType::get(voidTy, {int8PtrTy}, false), Function::InternalLinkage, name, m);
    f->setCallingConv(CallingConv::C);
    f->arg_begin()->setName("input");
    return f;
}

Function * generateSegmentParallelPipelineThreadFunction(IDISA::IDISA_Builder * iBuilder, const std::vector<KernelBuilder *> & kernels, Type * sharedStructType, const ProducerTable & producerTable, const unsigned id) {
    
    // ProducerPos[k][i] will hold the producedItemCount of the i^th output stream
    // set of the k^th kernel.  These values will be loaded immediately after the
    // doSegment and finalSegment calls for kernel k and later used as the
    // producer position arguments for later doSegment/finalSegment calls.
    
    std::vector<std::vector<Value *>> ProducerPos;
    
    
    const auto ip = iBuilder->saveIP();
    
    Module * const m = iBuilder->getModule();
    PointerType * const voidPtrTy = iBuilder->getVoidPtrTy();

    Function * const threadFunc = makeThreadFunction("thread" + std::to_string(id), m);

     // Create the basic blocks for the thread function.
    BasicBlock * entryBlock = BasicBlock::Create(iBuilder->getContext(), "entry", threadFunc);
    BasicBlock * segmentLoop = BasicBlock::Create(iBuilder->getContext(), "segmentLoop", threadFunc);
    BasicBlock * exitThreadBlock = BasicBlock::Create(iBuilder->getContext(), "exitThread", threadFunc);
    
    std::vector<BasicBlock *> segmentWait;
    std::vector<BasicBlock *> segmentLoopBody;
    for (unsigned i = 0; i < kernels.size(); i++) {
        auto kname = kernels[i]->getName();
        segmentWait.push_back(BasicBlock::Create(iBuilder->getContext(), kname + "Wait", threadFunc));
        segmentLoopBody.push_back(BasicBlock::Create(iBuilder->getContext(), kname + "Do", threadFunc));
    }

    iBuilder->SetInsertPoint(entryBlock);
    
    Value * input = &(*threadFunc->arg_begin());
    Value * sharedStruct = iBuilder->CreateBitCast(input, sharedStructType->getPointerTo());
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

        KernelBuilder * const kernel = kernels[k];

        if (kernel->hasNoTerminateAttribute()) {
            iBuilder->CreateCondBr(ready, segmentLoopBody[k], segmentWait[k]);
        } else { // If the kernel was terminated in a previous segment then the pipeline is done.
            BasicBlock * completionTest = BasicBlock::Create(iBuilder->getContext(), kernel->getName() + "Completed", threadFunc, 0);
            BasicBlock * exitBlock = BasicBlock::Create(iBuilder->getContext(), kernel->getName() + "Exit", threadFunc, 0);
            iBuilder->CreateCondBr(ready, completionTest, segmentWait[k]);
            iBuilder->SetInsertPoint(completionTest);
            Value * alreadyDone = kernel->getTerminationSignal(instancePtrs[k]);
            iBuilder->CreateCondBr(alreadyDone, exitBlock, segmentLoopBody[k]);
            iBuilder->SetInsertPoint(exitBlock);
            // Ensure that the next thread will also exit.
            kernel->releaseLogicalSegmentNo(instancePtrs[k], nextSegNo);
            iBuilder->CreateBr(exitThreadBlock);
        }
        iBuilder->SetInsertPoint(segmentLoopBody[k]);
        std::vector<Value *> args = {instancePtrs[k], doFinal};
        for (unsigned j = 0; j < kernel->getStreamInputs().size(); j++) {
            unsigned producerKernel, outputIndex;
            std::tie(producerKernel, outputIndex) = producerTable[k][j];
            args.push_back(ProducerPos[producerKernel][outputIndex]);
        }
        for (unsigned i = 0; i < kernel->getStreamOutputs().size(); ++i) {
            args.push_back(iBuilder->getSize(0));
        }
        kernel->createDoSegmentCall(args);
         if (!(kernel->hasNoTerminateAttribute())) {
            Value * terminated = kernel->getTerminationSignal(instancePtrs[k]);
            doFinal = iBuilder->CreateOr(doFinal, terminated);
        }
        std::vector<Value *> produced;
        for (unsigned i = 0; i < kernel->getStreamOutputs().size(); i++) {
            produced.push_back(kernel->getProducedItemCount(instancePtrs[k], kernel->getStreamOutputs()[i].name, doFinal));
        }
        ProducerPos.push_back(produced);

        kernel->releaseLogicalSegmentNo(instancePtrs[k], nextSegNo);
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
    
    for (auto k : kernels) {
        k->createInstance();
    }

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
        thread_functions.push_back(generateSegmentParallelPipelineThreadFunction(iBuilder, kernels, sharedStructType, producerTable, i));
    }
    iBuilder->restoreIP(ip);
    
    for (int i = 0; i < codegen::ThreadNum; i++) {
        iBuilder->CreatePThreadCreateCall(pthreadsPtrs[i], nullVal, thread_functions[i], sharedStruct);
    }
    
    std::vector<Value *> threadIDs;
    for (int i = 0; i < codegen::ThreadNum; i++) {
        threadIDs.push_back(iBuilder->CreateLoad(pthreadsPtrs[i]));
    }
    
    for (int i = 0; i < codegen::ThreadNum; i++) {
        iBuilder->CreatePThreadJoinCall(threadIDs[i], status);
    }
    
}

Function * generateParallelPipelineThreadFunction(IDISA::IDISA_Builder * iBuilder, const std::vector<KernelBuilder *> & kernels, Type * sharedStructType, const ProducerTable & producerTable, const ConsumerTable & consumerTable, const unsigned id) {
        
    const auto ip = iBuilder->saveIP();
    
    Module * m = iBuilder->getModule();
    Function * const threadFunc = makeThreadFunction("thread" + std::to_string(id), m);

    KernelBuilder * const kernel = kernels[id];
    Value * bufferSegments = ConstantInt::get(iBuilder->getSizeTy(), codegen::BufferSegments - 1);
    ConstantInt * segmentItems = iBuilder->getSize(codegen::SegmentSize * iBuilder->getBitBlockWidth());

     // Create the basic blocks for the thread function.
    BasicBlock * entryBlock = BasicBlock::Create(iBuilder->getContext(), "entry", threadFunc);
    BasicBlock * outputCheckBlock = BasicBlock::Create(iBuilder->getContext(), "outputCheck", threadFunc);
    BasicBlock * doSegmentBlock = BasicBlock::Create(iBuilder->getContext(), "doSegment", threadFunc);
    BasicBlock * exitThreadBlock = BasicBlock::Create(iBuilder->getContext(), "exitThread", threadFunc);

    iBuilder->SetInsertPoint(entryBlock);
    
    Value * input = &(*threadFunc->arg_begin());
    Value * sharedStruct = iBuilder->CreateBitCast(input, sharedStructType->getPointerTo());

    const unsigned n = kernels.size();

    Value * instancePtrs[n];
    for (unsigned k = 0; k < n; k++) {
        Value * const ptr = iBuilder->CreateGEP(sharedStruct, {iBuilder->getInt32(0), iBuilder->getInt32(k)});
        instancePtrs[k] = iBuilder->CreateLoad(ptr);
    }

    iBuilder->CreateBr(outputCheckBlock);

    iBuilder->SetInsertPoint(outputCheckBlock);
    PHINode * segNo = iBuilder->CreatePHI(iBuilder->getSizeTy(), 3, "segNo");
    segNo->addIncoming(iBuilder->getSize(0), entryBlock);
    segNo->addIncoming(segNo, outputCheckBlock);

    Value * outputWaitCond = iBuilder->getTrue();
    for (unsigned j = 0; j < kernel->getStreamOutputs().size(); j++) {
        const auto & consumerKernels = consumerTable[id][j];
        for (unsigned k = 0; k < consumerKernels.size(); k++) {
            Value * consumerSegNo = kernels[consumerKernels[k]]->acquireLogicalSegmentNo(instancePtrs[consumerKernels[k]]);
            outputWaitCond = iBuilder->CreateAnd(outputWaitCond, iBuilder->CreateICmpULE(segNo, iBuilder->CreateAdd(consumerSegNo, bufferSegments)));
        }
    }

    if (kernel->getStreamInputs().empty()) {

        iBuilder->CreateCondBr(outputWaitCond, doSegmentBlock, outputCheckBlock);

        iBuilder->SetInsertPoint(doSegmentBlock);

        std::vector<Value *> args = {instancePtrs[id], iBuilder->getFalse()};
        for (unsigned i = 0; i < kernel->getStreamOutputs().size(); ++i) {
            args.push_back(iBuilder->getSize(0));
        }
        kernel->createDoSegmentCall(args);
        Value * nextSegNo = iBuilder->CreateAdd(segNo, iBuilder->getSize(1));
        segNo->addIncoming(nextSegNo, doSegmentBlock);
        Value * const terminated = kernel->getTerminationSignal(instancePtrs[id]);
        kernel->releaseLogicalSegmentNo(instancePtrs[id], nextSegNo);
        iBuilder->CreateCondBr(terminated, exitThreadBlock, outputCheckBlock);

    } else {

        BasicBlock * inputCheckBlock = BasicBlock::Create(iBuilder->getContext(), "inputCheck", threadFunc, doSegmentBlock);

        iBuilder->CreateCondBr(outputWaitCond, inputCheckBlock, outputCheckBlock);

        iBuilder->SetInsertPoint(inputCheckBlock); 
        
        Value * inputWaitCond = iBuilder->getTrue();
        for (unsigned j = 0; j < kernel->getStreamInputs().size(); j++) {
            unsigned producerKernel, outputIndex;
            std::tie(producerKernel, outputIndex) = producerTable[id][j];
            Value * producerSegNo = kernels[producerKernel]->acquireLogicalSegmentNo(instancePtrs[producerKernel]);
            inputWaitCond = iBuilder->CreateAnd(inputWaitCond, iBuilder->CreateICmpULT(segNo, producerSegNo));
        }

        iBuilder->CreateCondBr(inputWaitCond, doSegmentBlock, inputCheckBlock);

        iBuilder->SetInsertPoint(doSegmentBlock);

        Value * nextSegNo = iBuilder->CreateAdd(segNo, iBuilder->getSize(1));

        Value * terminated = iBuilder->getTrue();
        for (unsigned j = 0; j < kernel->getStreamInputs().size(); j++) {
            unsigned producerKernel, outputIndex;
            std::tie(producerKernel, outputIndex) = producerTable[id][j];
            terminated = iBuilder->CreateAnd(terminated, kernels[producerKernel]->getTerminationSignal(instancePtrs[producerKernel]));
            Value * producerSegNo = kernels[producerKernel]->acquireLogicalSegmentNo(instancePtrs[producerKernel]);
            terminated = iBuilder->CreateAnd(terminated, iBuilder->CreateICmpEQ(nextSegNo, producerSegNo));
        }
        
        std::vector<Value *> args = {instancePtrs[id], terminated};
        for (unsigned j = 0; j < kernel->getStreamInputs().size(); j++) {
            unsigned producerKernel, outputIndex;
            std::tie(producerKernel, outputIndex) = producerTable[id][j];
            args.push_back(iBuilder->CreateMul(segmentItems, segNo));
        }
        for (unsigned i = 0; i < kernel->getStreamOutputs().size(); ++i) {
            args.push_back(iBuilder->getSize(0));
        }
        kernel->createDoSegmentCall(args);
        segNo->addIncoming(nextSegNo, doSegmentBlock);
        kernel->releaseLogicalSegmentNo(instancePtrs[id], nextSegNo);

        iBuilder->CreateCondBr(terminated, exitThreadBlock, outputCheckBlock);
    }

    iBuilder->SetInsertPoint(exitThreadBlock);
    iBuilder->CreatePThreadExitCall(ConstantPointerNull::getNullValue(iBuilder->getVoidPtrTy()));
    iBuilder->CreateRetVoid();
    iBuilder->restoreIP(ip);
    return threadFunc;
}

void generateParallelPipeline(IDISA::IDISA_Builder * iBuilder, const std::vector<KernelBuilder *> & kernels) {
    
    Module * const m = iBuilder->getModule();
    IntegerType * const size_ty = iBuilder->getSizeTy();
    PointerType * const voidPtrTy = iBuilder->getVoidPtrTy();
    PointerType * const int8PtrTy = iBuilder->getInt8PtrTy();
    
    for (auto k : kernels) {
        k->createInstance();
    }

    const ProducerTable producerTable = createProducerTable(kernels);
    const ConsumerTable consumerTable = createConsumerTable(kernels);
    const unsigned n = kernels.size();

    Type * const pthreadsTy = ArrayType::get(size_ty, n);
    AllocaInst * const pthreads = iBuilder->CreateAlloca(pthreadsTy);    
    Value * pthreadsPtrs[n];
    for (unsigned i = 0; i < n; i++) {
        pthreadsPtrs[i] = iBuilder->CreateGEP(pthreads, {iBuilder->getInt32(0), iBuilder->getInt32(i)});
    }
    
    std::vector<Type *> structTypes;
    for (unsigned i = 0; i < n; i++) {
        structTypes.push_back(kernels[i]->getInstance()->getType());
    }
    Type * const sharedStructType = StructType::get(m->getContext(), structTypes);
    AllocaInst * sharedStruct = iBuilder->CreateAlloca(sharedStructType);
    for (unsigned i = 0; i < n; i++) {
        Value * ptr = iBuilder->CreateGEP(sharedStruct, {iBuilder->getInt32(0), iBuilder->getInt32(i)});
        iBuilder->CreateStore(kernels[i]->getInstance(), ptr);
    }
    for (unsigned i = 0; i < n; i++) {
        KernelBuilder * const K = kernels[i];
        K->releaseLogicalSegmentNo(K->getInstance(), iBuilder->getSize(0));
    }

    Function * thread_functions[n];
    for (unsigned i = 0; i < n; i++) {
        thread_functions[i] = generateParallelPipelineThreadFunction(iBuilder, kernels, sharedStructType, producerTable, consumerTable, i);
    }
    
    Value * nullVal = Constant::getNullValue(voidPtrTy);
    for (unsigned i = 0; i < n; i++) {
        iBuilder->CreatePThreadCreateCall(pthreadsPtrs[i], nullVal, thread_functions[i], sharedStruct);
    }

    AllocaInst * const status = iBuilder->CreateAlloca(int8PtrTy);
    for (unsigned i = 0; i < n; i++) {
        Value * threadId = iBuilder->CreateLoad(pthreadsPtrs[i]);
        iBuilder->CreatePThreadJoinCall(threadId, status);
    }

}

void generatePipelineLoop(IDISA::IDISA_Builder * iBuilder, const std::vector<KernelBuilder *> & kernels) {

    for (auto k : kernels) {
        k->createInstance();
    }
    BasicBlock * entryBlock = iBuilder->GetInsertBlock();
    Function * main = entryBlock->getParent();

    // Create the basic blocks for the loop.
    BasicBlock * pipelineLoop = BasicBlock::Create(iBuilder->getContext(), "pipelineLoop", main);
    BasicBlock * pipelineExit = BasicBlock::Create(iBuilder->getContext(), "pipelineExit", main);

    StreamSetBufferMap<Value *> producedPos;
    StreamSetBufferMap<std::pair<PHINode *, Value *>> consumedPos;

    iBuilder->CreateBr(pipelineLoop);
    iBuilder->SetInsertPoint(pipelineLoop);

    // Gather all of our
    for (unsigned k = 0; k < kernels.size(); k++) {
        KernelBuilder * const kernel = kernels[k];
        const auto & outputs = kernel->getStreamOutputs();
        for (unsigned i = 0; i < outputs.size(); ++i) {
            const StreamSetBuffer * const buf = kernel->getStreamSetOutputBuffer(i);
            if (LLVM_UNLIKELY(consumedPos.count(buf) != 0)) {
                report_fatal_error(kernel->getName() + " redefines stream set " + outputs[i].name);
            }
            PHINode * phi = iBuilder->CreatePHI(iBuilder->getSizeTy(), 2);
            phi->addIncoming(iBuilder->getSize(0), entryBlock);
            consumedPos.emplace(buf, std::make_pair(phi, nullptr));
        }
    }

    Value * terminated = iBuilder->getFalse();
    for (unsigned k = 0; k < kernels.size(); k++) {
        KernelBuilder * const kernel = kernels[k];
        Value * const instance = kernel->getInstance();
        const auto & inputs = kernel->getStreamInputs();
        const auto & outputs = kernel->getStreamOutputs();
        std::vector<Value *> args = {instance, terminated};
        for (unsigned i = 0; i < inputs.size(); ++i) {
            const auto f = producedPos.find(kernel->getStreamSetInputBuffer(i));
            if (LLVM_UNLIKELY(f == producedPos.end())) {
                report_fatal_error(kernel->getName() + " uses stream set " + inputs[i].name + " prior to its definition");
            }
            args.push_back(f->second);
        }
        for (unsigned i = 0; i < outputs.size(); ++i) {
            const auto f = consumedPos.find(kernel->getStreamSetOutputBuffer(i));
            assert (f != consumedPos.end());
            args.push_back(std::get<0>(f->second));
        }
        kernel->createDoSegmentCall(args);
        if (!kernel->hasNoTerminateAttribute()) {
            terminated = iBuilder->CreateOr(terminated, kernel->getTerminationSignal(instance));
        }
        for (unsigned i = 0; i < outputs.size(); i++) {
            Value * const produced = kernel->getProducedItemCount(instance, outputs[i].name, terminated);
            const StreamSetBuffer * const buf = kernel->getStreamSetOutputBuffer(i);
            assert (producedPos.count(buf) == 0);
            producedPos.emplace(buf, produced);
        }
        for (unsigned i = 0; i < inputs.size(); i++) {
            Value * const processed = kernel->getProcessedItemCount(instance, inputs[i].name);
            const StreamSetBuffer * const buf = kernel->getStreamSetInputBuffer(i);
            const auto f = consumedPos.find(buf);
            assert (f != consumedPos.end());
            Value *& consumed = std::get<1>(f->second);
            if (consumed) {
                consumed = iBuilder->CreateSelect(iBuilder->CreateICmpULT(processed, consumed), processed, consumed);
            } else {
                consumed = processed;
            }
        }
        Value * const segNo = kernel->acquireLogicalSegmentNo(instance);
        kernel->releaseLogicalSegmentNo(instance, iBuilder->CreateAdd(segNo, iBuilder->getSize(1)));
    }
    // update the consumed position phi nodes with the last min processed count of each input stream
    for (const auto entry : consumedPos) {
        PHINode * const phi = std::get<0>(entry.second);
        Value * const value = std::get<1>(entry.second);
        phi->addIncoming(value ? value : phi, pipelineLoop);
    }
    iBuilder->CreateCondBr(terminated, pipelineExit, pipelineLoop);
    iBuilder->SetInsertPoint(pipelineExit);
}
