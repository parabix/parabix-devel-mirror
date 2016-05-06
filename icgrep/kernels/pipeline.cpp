/*
 *  Copyright (c) 2016 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 */


#include <toolchain.h>
#include "pipeline.h"
#include "utf_encoding.h"

#include <kernels/scanmatchgen.h>
#include <kernels/s2p_kernel.h>
#include <kernels/instance.h>

#include <pablo/function.h>
#include <pablo/pablo_compiler.h>
#include <pablo/pablo_toolchain.h>

#include <llvm/IR/Intrinsics.h>

using namespace pablo;
using namespace kernel;

PipelineBuilder::PipelineBuilder(Module * m, IDISA::IDISA_Builder * b)
: mMod(m)
, iBuilder(b)
, mBitBlockType(b->getBitBlockType())
, mBlockSize(b->getBitBlockWidth()) {

}

PipelineBuilder::~PipelineBuilder() {
    delete mS2PKernel;
    delete mICgrepKernel;
    delete mScanMatchKernel;
}

void PipelineBuilder::CreateKernels(PabloFunction * function, bool isNameExpression){
    mS2PKernel = new KernelBuilder(iBuilder, "s2p", codegen::SegmentSize);
    mICgrepKernel = new KernelBuilder(iBuilder, "icgrep", codegen::SegmentSize);
    mScanMatchKernel = new KernelBuilder(iBuilder, "scanMatch", codegen::SegmentSize);
    generateS2PKernel(mMod, iBuilder, mS2PKernel);
    generateScanMatch(mMod, iBuilder, 64, mScanMatchKernel, isNameExpression);
    pablo_function_passes(function);
    PabloCompiler pablo_compiler(mMod, iBuilder);
    try {
        pablo_compiler.setKernel(mICgrepKernel);
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

Value * generatePopcount(IDISA::IDISA_Builder * iBuilder, Value * bits) {
    Value * ctpopFunc = Intrinsic::getDeclaration(iBuilder->getModule(), Intrinsic::ctpop, bits->getType());
    return iBuilder->CreateCall(ctpopFunc, std::vector<Value *>({bits}));
}

Value * Cal_Count(Instance * icGrepInstance, IDISA::IDISA_Builder * iBuilder, int mBlockSize) {
    const unsigned index = 0;
    const unsigned streamOffset = 0;
    Value * match = (icGrepInstance->getOutputStream(index, streamOffset));
    Value * temp = iBuilder->CreateLoad(match);
    Value * matches = iBuilder->CreateBitCast(temp, iBuilder->getIntNTy(mBlockSize));
    Value * popcount = generatePopcount(iBuilder, matches);
    return popcount;
}

Function * PipelineBuilder::ExecuteKernels(bool CountOnly) {
    Type * const int64ty = iBuilder->getInt64Ty();
    Type * const int8PtrTy = iBuilder->getInt8PtrTy();
    Type * const inputType = PointerType::get(ArrayType::get(StructType::get(mMod->getContext(), std::vector<Type *>({ArrayType::get(mBitBlockType, 8)})), 1), 0);

    Function * const main = cast<Function>(mMod->getOrInsertFunction("Main", Type::getVoidTy(mMod->getContext()), inputType, int64ty, int64ty, iBuilder->getInt1Ty(), nullptr));
    main->setCallingConv(CallingConv::C);
    Function::arg_iterator args = main->arg_begin();

    Value * const inputStream = &*(args++);
    inputStream->setName("input");
    Value * const bufferSize = &*(args++);
    bufferSize->setName("bufferSize");
    Value * const fileIdx = &*(args++);
    fileIdx->setName("fileIdx");
    Value * const finalLineUnterminated = &*(args++);
    finalLineUnterminated->setName("finalLineUnterminated");

    iBuilder->SetInsertPoint(BasicBlock::Create(mMod->getContext(), "entry", main,0));


    BasicBlock * entryBlock = iBuilder->GetInsertBlock();
    BasicBlock * segmentCondBlock = nullptr;
    BasicBlock * segmentBodyBlock = nullptr;
    const unsigned segmentSize = codegen::SegmentSize;
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
    BasicBlock * unterminatedBlock = BasicBlock::Create(mMod->getContext(), "unterminated", main, 0);
    BasicBlock * exitBlock = BasicBlock::Create(mMod->getContext(), "exit", main, 0);

    Value * count = iBuilder->CreateAlloca (Type::getIntNTy(mMod->getContext(), mBlockSize), nullptr, "count");
    Value * num = ConstantInt::get(iBuilder->getIntNTy(mBlockSize), 0);
    iBuilder->CreateStore(num, count, false);

    Instance * s2pInstance = mS2PKernel->instantiate(inputStream);
    Instance * icGrepInstance = mICgrepKernel->instantiate(s2pInstance->getOutputStreamBuffer());
    Instance * scanMatchInstance = mScanMatchKernel->instantiate(icGrepInstance->getOutputStreamBuffer());
    
    if(!CountOnly) {
	scanMatchInstance->setInternalState("FileBuf", iBuilder->CreateBitCast(inputStream, int8PtrTy));
	scanMatchInstance->setInternalState("FileSize", bufferSize);
	scanMatchInstance->setInternalState("FileIdx", fileIdx);
    }
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
            icGrepInstance->CreateDoBlockCall();
	    if(CountOnly){
		Value * popcount_for = Cal_Count(icGrepInstance, iBuilder, mBlockSize);
		Value * temp_countfor = iBuilder->CreateLoad(count);
		Value * add_for = iBuilder->CreateAdd(temp_countfor, popcount_for);
		iBuilder->CreateStore(add_for, count);
	    }
        }
	if(!CountOnly) {
	    for (unsigned i = 0; i < segmentSize; ++i) {
	        scanMatchInstance->CreateDoBlockCall();
	    }
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
    icGrepInstance->CreateDoBlockCall();
    if(CountOnly){
	Value * popcount = Cal_Count(icGrepInstance, iBuilder, mBlockSize);    
	Value * temp_count = iBuilder->CreateLoad(count);
	Value * add = iBuilder->CreateAdd(temp_count, popcount);
	iBuilder->CreateStore(add, count);
    }

    if(!CountOnly) {
	scanMatchInstance->CreateDoBlockCall();
    }

    remainingBytes->addIncoming(iBuilder->CreateSub(remainingBytes, step), fullBodyBlock);
    iBuilder->CreateBr(fullCondBlock);

    iBuilder->SetInsertPoint(finalBlock);
    Value * const b4 = s2pInstance->getOutputStream(4);
    Value * const b6 = s2pInstance->getOutputStream(6);
    Value * emptyBlockCond = iBuilder->CreateICmpEQ(remainingBytes, ConstantInt::get(int64ty, 0));
    iBuilder->CreateCondBr(emptyBlockCond, finalEmptyBlock, finalPartialBlock);


    iBuilder->SetInsertPoint(finalPartialBlock);
    s2pInstance->CreateDoBlockCall();
    iBuilder->CreateBr(endBlock);

    iBuilder->SetInsertPoint(finalEmptyBlock);
    s2pInstance->clearOutputStreamSet();
    iBuilder->CreateBr(endBlock);

    iBuilder->SetInsertPoint(endBlock);
    Value * isFinalLineUnterminated = iBuilder->CreateICmpEQ(finalLineUnterminated, ConstantInt::getNullValue(finalLineUnterminated->getType()));
    iBuilder->CreateCondBr(isFinalLineUnterminated, exitBlock, unterminatedBlock);
    
    iBuilder->SetInsertPoint(unterminatedBlock);

    Value * remaining = iBuilder->CreateZExt(remainingBytes, iBuilder->getIntNTy(mBlockSize));
    Value * EOF_pos = iBuilder->CreateShl(ConstantInt::get(iBuilder->getIntNTy(mBlockSize), 1), remaining);
    EOF_pos = iBuilder->CreateBitCast(EOF_pos, mBitBlockType);


    Value * b4val = iBuilder->CreateBlockAlignedLoad(b4);
    b4val = iBuilder->CreateOr(b4val, EOF_pos);
    iBuilder->CreateBlockAlignedStore(b4val, b4);

    Value * b6val = iBuilder->CreateBlockAlignedLoad(b6);
    b6val = iBuilder->CreateOr(b6val, EOF_pos);
    iBuilder->CreateBlockAlignedStore(b6val, b6);

    iBuilder->CreateBr(exitBlock);

    iBuilder->SetInsertPoint(exitBlock);

    icGrepInstance->CreateDoBlockCall();
    if(CountOnly){
	Value * popcount1 = Cal_Count(icGrepInstance, iBuilder, mBlockSize);    
	Value * temp_count1 = iBuilder->CreateLoad(count);
	Value * add1 = iBuilder->CreateAdd(temp_count1, popcount1);
	iBuilder->CreateStore(add1, count);
    }
    if(!CountOnly) {
	scanMatchInstance->CreateDoBlockCall();
    }
    if(CountOnly){
	Value * Ret = iBuilder->CreateLoad(count);
	iBuilder->CreateRet(Ret);
    }
    else{
	iBuilder->CreateRetVoid();
    }


    return main;
}
