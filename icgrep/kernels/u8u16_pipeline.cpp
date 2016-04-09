/*
 *  Copyright (c) 2016 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 */

#include <kernels/u8u16_pipeline.h>
#include <utf_encoding.h>

#include <kernels/s2p_kernel.h>
#include <kernels/deletion.h>
#include <kernels/p2s_kernel.h>
#include <kernels/stdout_kernel.h>
#include <kernels/instance.h>

#include <pablo/function.h>
#include <pablo/pablo_compiler.h>
#include <pablo/pablo_toolchain.h>

static cl::opt<unsigned> SegmentSize("segment-size", cl::desc("Segment Size"), cl::value_desc("positive integer"), cl::init(1));


using namespace pablo;
using namespace kernel;

PipelineBuilder::PipelineBuilder(Module * m, IDISA::IDISA_Builder * b)
: mMod(m)
, iBuilder(b)
, mBitBlockType(b->getBitBlockType())
, mBlockSize(b->getBitBlockWidth()){

}

PipelineBuilder::~PipelineBuilder(){
    delete mS2PKernel;
    delete mU8U16Kernel;
    delete mDelKernel;
    delete mP2SKernel;
    //delete mStdOutKernel;
}

void PipelineBuilder::CreateKernels(PabloFunction * function){
    mS2PKernel = new KernelBuilder(iBuilder, "s2p", SegmentSize);
    mU8U16Kernel = new KernelBuilder(iBuilder, "u8u16", SegmentSize);
    mDelKernel = new KernelBuilder(iBuilder, "del", SegmentSize);
    mP2SKernel = new KernelBuilder(iBuilder, "p2s", SegmentSize);    
    //mStdOutKernel = new KernelBuilder(iBuilder, "stdout", SegmentSize);

    generateS2PKernel(mMod, iBuilder, mS2PKernel);
    generateP2S_16_withCompressedOutputKernel(mMod, iBuilder, mP2SKernel);
    generateDeletionKernel(mMod, iBuilder, iBuilder->getBitBlockWidth()/16, /*stream_count=*/16, mDelKernel);
    //generateStdOutKernel(mMod, iBuilder, mStdOutKernel, 16);

    pablo_function_passes(function);

    PabloCompiler pablo_compiler(mMod, iBuilder);
    try {
        pablo_compiler.setKernel(mU8U16Kernel);
        pablo_compiler.compile(function);
        delete function;
        releaseSlabAllocatorMemory();
    } catch (std::runtime_error e) {
        delete function;
        releaseSlabAllocatorMemory();
        std::cerr << "Runtime error: " << e.what() << std::endl;
        exit(1);
    }
    
}

Function *  PipelineBuilder::ExecuteKernels() {
    Type * const int64ty = iBuilder->getInt64Ty();
    Type * const inputType = PointerType::get(ArrayType::get(StructType::get(mMod->getContext(), std::vector<Type *>({ArrayType::get(mBitBlockType, 8)})), 1), 0);
    
    Function * const main = cast<Function>(mMod->getOrInsertFunction("Main", Type::getVoidTy(mMod->getContext()), inputType, int64ty, nullptr));
    main->setCallingConv(CallingConv::C);
    Function::arg_iterator args = main->arg_begin();
    
    Value * const inputStream = args++;
    inputStream->setName("input");
    Value * const bufferSize = args++;
    bufferSize->setName("bufferSize");
    
    iBuilder->SetInsertPoint(BasicBlock::Create(mMod->getContext(), "entry", main,0));
    
    BasicBlock * entryBlock = iBuilder->GetInsertBlock();

    BasicBlock * segmentCondBlock = nullptr;
    BasicBlock * segmentBodyBlock = nullptr;
    const unsigned segmentSize = SegmentSize;
    if (segmentSize > 1) {
        segmentCondBlock = BasicBlock::Create(mMod->getContext(), "segmentCond", main, 0);
        segmentBodyBlock = BasicBlock::Create(mMod->getContext(), "segmentBody", main, 0);
    }
    BasicBlock * fullCondBlock = BasicBlock::Create(mMod->getContext(), "fullCond", main, 0);
    BasicBlock * fullBodyBlock = BasicBlock::Create(mMod->getContext(), "fullBody", main, 0);
    BasicBlock * finalBlock = BasicBlock::Create(mMod->getContext(), "final", main, 0);
    BasicBlock * finalPartialBlock = BasicBlock::Create(mMod->getContext(), "partial", main, 0);
    BasicBlock * finalEmptyBlock = BasicBlock::Create(mMod->getContext(), "empty", main, 0);
    BasicBlock * endBlock = BasicBlock::Create(mMod->getContext(), "end", main, 0);

    Instance * s2pInstance = mS2PKernel->instantiate(inputStream);
    Instance * u8u16Instance = mU8U16Kernel->instantiate(s2pInstance->getOutputStreamBuffer());
    Instance * delInstance = mDelKernel->instantiate(u8u16Instance->getOutputStreamBuffer());
    Instance * p2sInstance = mP2SKernel->instantiate(delInstance->getOutputStreamBuffer());

    
    Value * initialBufferSize = nullptr;
    BasicBlock * initialBlock = nullptr;
    
    if (segmentSize > 1) {
        iBuilder->CreateBr(segmentCondBlock);
        iBuilder->SetInsertPoint(segmentCondBlock);
        PHINode * remainingBytes = iBuilder->CreatePHI(int64ty, 2, "remainingBytes");
        remainingBytes->addIncoming(bufferSize, entryBlock);
        Constant * const step = ConstantInt::get(int64ty, mBlockSize * segmentSize);
        Value * segmentCondTest = iBuilder->CreateICmpULT(remainingBytes, step);
        iBuilder->CreateCondBr(segmentCondTest, fullCondBlock, segmentBodyBlock);
        iBuilder->SetInsertPoint(segmentBodyBlock);
        for (unsigned i = 0; i < segmentSize; ++i) {
            s2pInstance->CreateDoBlockCall();
        }
        for (unsigned i = 0; i < segmentSize; ++i) {
            u8u16Instance->CreateDoBlockCall();
        }
        for (unsigned i = 0; i < segmentSize; ++i) {
            delInstance->CreateDoBlockCall();
        }
        for (unsigned i = 0; i < segmentSize; ++i) {
            p2sInstance->CreateDoBlockCall();
        }
        remainingBytes->addIncoming(iBuilder->CreateSub(remainingBytes, step), segmentBodyBlock);
        iBuilder->CreateBr(segmentCondBlock);
        initialBufferSize = remainingBytes;
        initialBlock = segmentCondBlock;
    } else {
        initialBufferSize = bufferSize;
        initialBlock = entryBlock;
        iBuilder->CreateBr(fullCondBlock);
    }

    iBuilder->SetInsertPoint(fullCondBlock);
    PHINode * remainingBytes = iBuilder->CreatePHI(int64ty, 2, "remainingBytes");
    remainingBytes->addIncoming(initialBufferSize, initialBlock);

    Constant * const step = ConstantInt::get(int64ty, mBlockSize);
    Value * fullCondTest = iBuilder->CreateICmpULT(remainingBytes, step);
    iBuilder->CreateCondBr(fullCondTest, finalBlock, fullBodyBlock);
    
    iBuilder->SetInsertPoint(fullBodyBlock);

    s2pInstance->CreateDoBlockCall();
    u8u16Instance->CreateDoBlockCall();
    delInstance->CreateDoBlockCall();
    p2sInstance->CreateDoBlockCall();

    Value * diff = iBuilder->CreateSub(remainingBytes, step);

    remainingBytes->addIncoming(diff, fullBodyBlock);
    iBuilder->CreateBr(fullCondBlock);
    
    iBuilder->SetInsertPoint(finalBlock);
    Value * emptyBlockCond = iBuilder->CreateICmpEQ(remainingBytes, ConstantInt::get(int64ty, 0));
    iBuilder->CreateCondBr(emptyBlockCond, finalEmptyBlock, finalPartialBlock);
    
    
    iBuilder->SetInsertPoint(finalPartialBlock);
    s2pInstance->CreateDoBlockCall();
    iBuilder->CreateBr(endBlock);
    
    iBuilder->SetInsertPoint(finalEmptyBlock);
    s2pInstance->clearOutputStreamSet();
    iBuilder->CreateBr(endBlock);
    
    iBuilder->SetInsertPoint(endBlock);

    Value * const delmaskPtr = u8u16Instance->getOutputStream(16);
    u8u16Instance->CreateDoBlockCall();
    Value * remaining = iBuilder->CreateZExt(remainingBytes, iBuilder->getIntNTy(mBlockSize));
    Value * EOF_del = iBuilder->bitCast(iBuilder->CreateShl(Constant::getAllOnesValue(iBuilder->getIntNTy(mBlockSize)), remaining));
    Value * const delmaskVal = iBuilder->CreateBlockAlignedLoad(delmaskPtr);
    iBuilder->CreateBlockAlignedStore(iBuilder->CreateOr(EOF_del, delmaskVal), delmaskPtr);
    delInstance->CreateDoBlockCall();
    p2sInstance->CreateDoBlockCall();
    iBuilder->CreateRetVoid();
    return main;
}
