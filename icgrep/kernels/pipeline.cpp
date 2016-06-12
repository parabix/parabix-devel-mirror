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

void PipelineBuilder::CreateKernels(PabloFunction * function, bool UTF_16, bool isNameExpression){
    mS2PKernel = new KernelBuilder(iBuilder, "s2p", codegen::SegmentSize);
    mICgrepKernel = new KernelBuilder(iBuilder, "icgrep", codegen::SegmentSize);
    mScanMatchKernel = new KernelBuilder(iBuilder, "scanMatch", codegen::SegmentSize);
    if (UTF_16) {
	generateS2P_16Kernel(mMod, iBuilder, mS2PKernel);
    }
    else {
	generateS2PKernel(mMod, iBuilder, mS2PKernel);
    }
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

inline Value * generatePopcount(IDISA::IDISA_Builder * iBuilder, Value * bits) {
    Value * ctpopFunc = Intrinsic::getDeclaration(iBuilder->getModule(), Intrinsic::ctpop, bits->getType());
    return iBuilder->CreateCall(ctpopFunc, {bits});
}

inline Value * Cal_Count(Instance * icGrepInstance, IDISA::IDISA_Builder * iBuilder) {
    Value * match = icGrepInstance->getOutputStream(0, 0);
    Value * matches = iBuilder->CreateLoad(match, false, "match");
    return generatePopcount(iBuilder, matches);
}

Function * PipelineBuilder::ExecuteKernels(bool CountOnly, bool UTF_16) {
    Type * const int64ty = iBuilder->getInt64Ty();
    Type * const int8PtrTy = iBuilder->getInt8PtrTy();
    Type * const inputType = PointerType::get(ArrayType::get(StructType::get(mMod->getContext(), std::vector<Type *>({ArrayType::get(mBitBlockType, (UTF_16 ? 16 : 8))})), 1), 0);
    Type * const resultTy = CountOnly ? int64ty : iBuilder->getVoidTy();
    Function * const main = cast<Function>(mMod->getOrInsertFunction("Main", resultTy, inputType, int64ty, int64ty, nullptr));
    main->setCallingConv(CallingConv::C);
    Function::arg_iterator args = main->arg_begin();

    Value * const inputStream = &*(args++);
    inputStream->setName("input");
    Value * const bufferSize = &*(args++);
    bufferSize->setName("bufferSize");
    Value * const fileIdx = &*(args++);
    fileIdx->setName("fileIdx");

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
    BasicBlock * exitBlock = BasicBlock::Create(mMod->getContext(), "exit", main, 0);

    Value * count = nullptr;
    if (CountOnly) {
        count = iBuilder->CreateAlloca(mBitBlockType, nullptr, "count");
        iBuilder->CreateStore(ConstantInt::getNullValue(mBitBlockType), count);
    }

    Instance * s2pInstance = mS2PKernel->instantiate(inputStream);
    Instance * icGrepInstance = mICgrepKernel->instantiate(s2pInstance->getOutputStreamBuffer());
    Instance * scanMatchInstance = nullptr;
    
    if (!CountOnly) {
        scanMatchInstance = mScanMatchKernel->instantiate(icGrepInstance->getOutputStreamBuffer());
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
        Constant * const step = ConstantInt::get(int64ty, mBlockSize * segmentSize * (UTF_16 ? 2 : 1));
        Value * segmentCondTest = iBuilder->CreateICmpULT(remainingBytes, step);
        iBuilder->CreateCondBr(segmentCondTest, fullCondBlock, segmentBodyBlock);
        iBuilder->SetInsertPoint(segmentBodyBlock);
        for (unsigned i = 0; i < segmentSize; ++i) {
            s2pInstance->CreateDoBlockCall();
        }
        for (unsigned i = 0; i < segmentSize; ++i) {
	    Value * match = (icGrepInstance->getOutputStream(0, 0)); 
	    icGrepInstance->CreateDoBlockCall();
	    Value * temp = iBuilder->CreateLoad(match);
	    Value * matches = iBuilder->CreateBitCast(temp, iBuilder->getIntNTy(mBlockSize));
	    Value * popcount_for = generatePopcount(iBuilder, matches);
	    if(CountOnly){
		Value * temp_count = iBuilder->CreateLoad(count);
		Value * prev_count = iBuilder->CreateBitCast(temp_count, iBuilder->getIntNTy(mBlockSize));
		Value * add_for = iBuilder->CreateAdd(prev_count, popcount_for);
		Value * add = iBuilder->CreateBitCast(add_for, mBitBlockType);
		iBuilder->CreateStore(add, count);
	    }
        }
        if (!CountOnly) {
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

    Constant * const step = ConstantInt::get(int64ty, mBlockSize * (UTF_16 ? 2 : 1));
    Value * fullCondTest = iBuilder->CreateICmpULT(remainingBytes, step);
    iBuilder->CreateCondBr(fullCondTest, finalBlock, fullBodyBlock);

    iBuilder->SetInsertPoint(fullBodyBlock);
    s2pInstance->CreateDoBlockCall();
    icGrepInstance->CreateDoBlockCall();
    if (CountOnly) {
        Value * popcount = Cal_Count(icGrepInstance, iBuilder);
        Value * temp_count = iBuilder->CreateLoad(count);
        Value * add = iBuilder->CreateAdd(temp_count, popcount);
        iBuilder->CreateStore(add, count);
    } else {
        scanMatchInstance->CreateDoBlockCall();
    }

    remainingBytes->addIncoming(iBuilder->CreateSub(remainingBytes, step), fullBodyBlock);
    iBuilder->CreateBr(fullCondBlock);

    iBuilder->SetInsertPoint(finalBlock);
    Value * emptyBlockCond = iBuilder->CreateICmpEQ(remainingBytes, ConstantInt::get(int64ty, 0));
    iBuilder->CreateCondBr(emptyBlockCond, finalEmptyBlock, finalPartialBlock);


    iBuilder->SetInsertPoint(finalPartialBlock);
    s2pInstance->CreateDoBlockCall();
    iBuilder->CreateBr(exitBlock);

    iBuilder->SetInsertPoint(finalEmptyBlock);
    s2pInstance->clearOutputStreamSet();
    iBuilder->CreateBr(exitBlock);

    iBuilder->SetInsertPoint(exitBlock);

    Value * remainingByte = iBuilder->CreateZExt(remainingBytes, iBuilder->getIntNTy(mBlockSize));
    Value * remainingUnit = iBuilder->CreateLShr(remainingByte, ConstantInt::get(iBuilder->getIntNTy(mBlockSize), 1));
    Value * EOFmark = iBuilder->CreateShl(ConstantInt::get(iBuilder->getIntNTy(mBlockSize), 1), UTF_16 ? remainingUnit : remainingByte);
    icGrepInstance->setInternalState("EOFmark", iBuilder->CreateBitCast(EOFmark, mBitBlockType));

    icGrepInstance->CreateDoBlockCall();
    if (CountOnly) {
        Value * popcount1 = Cal_Count(icGrepInstance, iBuilder);
        Value * temp_count1 = iBuilder->CreateLoad(count);
        Value * result = iBuilder->CreateAdd(temp_count1, popcount1);
        for (unsigned width = (mBlockSize / 64); width > 1; width /= 2) {
            std::vector<Constant *> mask(width / 2);
            for (unsigned i = 0; i < (width / 2); ++i) {
                mask[i] = iBuilder->getInt32(i);
            }
            Value * const undef = UndefValue::get(VectorType::get(int64ty, width));
            Value * const lh = iBuilder->CreateShuffleVector(result, undef, ConstantVector::get(mask));
            for (unsigned i = 0; i < (width / 2); ++i) {
                mask[i] = iBuilder->getInt32(i + (width / 2));
            }
            Value * const rh = iBuilder->CreateShuffleVector(result, undef, ConstantVector::get(mask));
            result = iBuilder->CreateAdd(lh, rh);
        }
        iBuilder->CreateRet(iBuilder->CreateExtractElement(result, iBuilder->getInt32(0)));
    } else {
        scanMatchInstance->CreateDoBlockCall();
        iBuilder->CreateRetVoid();
    }
    return main;
}
