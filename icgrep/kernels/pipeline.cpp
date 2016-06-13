/*
 *  Copyright (c) 2016 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 */


#include <toolchain.h>
#include "streamset.h"
#include "interface.h"

#include "pipeline.h"
#include "utf_encoding.h"

#include <kernels/scanmatchgen.h>
#include <kernels/s2p_kernel.h>
#include <kernels/instance.h>

#include <pablo/function.h>
#include <pablo/pablo_compiler.h>
#include <pablo/pablo_toolchain.h>

#include <llvm/IR/Intrinsics.h>
#include "llvm/Support/SourceMgr.h"
#include "llvm/IRReader/IRReader.h"
#include "llvm/Linker/Linker.h"

using namespace pablo;
using namespace kernel;

PipelineBuilder::PipelineBuilder(Module * m, IDISA::IDISA_Builder * b)
: mMod(m)
, iBuilder(b)
, mBitBlockType(b->getBitBlockType())
, mBlockSize(b->getBitBlockWidth()) {

}

PipelineBuilder::~PipelineBuilder() {
}

void PipelineBuilder::CreateKernels(PabloFunction * function, bool UTF_16, bool isNameExpression){
    //mS2PKernel = new KernelBuilder(iBuilder, "s2p", codegen::SegmentSize);
    mICgrepKernel = new KernelBuilder(iBuilder, "icgrep", codegen::SegmentSize);
    mScanMatchKernel = new KernelBuilder(iBuilder, "scanMatch", codegen::SegmentSize);
//#define LOAD_S2P_LL
#ifdef LOAD_S2P_LL
    SMDiagnostic Err;
    std::unique_ptr<Module> s2p_Module = parseIRFile("s2p.ll", Err, mMod->getContext());
    if (!s2p_Module) {
         Err.print("icgrep", errs());
         exit(1);
    }
    Linker L(*mMod);
    //s2p_Module->dump();
    generateScanMatch(mMod, iBuilder, 64, mScanMatchKernel, isNameExpression);
    L.linkInModule(std::move(s2p_Module));
    errs() << "s2p.ll loaded\n";
#else
    errs() << "A\n";
    //mS2PKernel = new KernelBuilder(iBuilder, "s2p", codegen::SegmentSize);
    s2pKernel  s2pk(iBuilder);
    //s2pk.addKernelDeclarations(mMod);
    //mMod->dump();
    std::unique_ptr<Module> s2pM = s2pk.createKernelModule();
    Linker L(*mMod);
    L.linkInModule(std::move(s2pM));
    errs() << "b\n";
    
    
    generateScanMatch(mMod, iBuilder, 64, mScanMatchKernel, isNameExpression);
#endif
    
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

inline Value * Cal_Count(Value * match_ptr, IDISA::IDISA_Builder * iBuilder) {
    Value * matches = iBuilder->CreateLoad(match_ptr, false, "match");
    return generatePopcount(iBuilder, matches);
}

Function * PipelineBuilder::ExecuteKernels(bool CountOnly, bool UTF_16) {

    Type * const int64ty = iBuilder->getInt64Ty();
    Type * const int8PtrTy = iBuilder->getInt8PtrTy();
    Type * const inputType = PointerType::get(ArrayType::get(ArrayType::get(mBitBlockType, (UTF_16 ? 16 : 8)), 1), 0);
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
    errs() << "B\n";

    iBuilder->SetInsertPoint(BasicBlock::Create(mMod->getContext(), "entry", main,0));
    iBuilder->CallPrintInt("bufferSize", bufferSize); 
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

    Value * count = nullptr;
    if (CountOnly) {
        count = iBuilder->CreateAlloca(mBitBlockType, nullptr, "count");
        iBuilder->CreateStore(ConstantInt::getNullValue(mBitBlockType), count);
    }
    errs() << "C\n";

    Value * s2pI = make_New(iBuilder,  "s2p", {});
    errs() << "D\n";

    StreamSetBuffer ByteStream(iBuilder, StreamSetType(1, (UTF_16 ? 16 : 8)), 0);
    ByteStream.setStreamSetBuffer(inputStream);
    
    StreamSetBuffer BasisBits(iBuilder, StreamSetType((UTF_16 ? 16 : 8), 1), codegen::SegmentSize);
    Value * basis_bits_ptr = BasisBits.allocateBuffer();
    
    Value * grepI = make_New(iBuilder,  "icgrep", {});
    
    StreamSetBuffer MatchResults(iBuilder, StreamSetType(2, 1), codegen::SegmentSize);
    Value * match_results_ptr = MatchResults.allocateBuffer();

    Instance * scanMatchInstance = nullptr;
    
    if (!CountOnly) {
        scanMatchInstance = mScanMatchKernel->instantiate(match_results_ptr);
        scanMatchInstance->setInternalState("FileBuf", iBuilder->CreateBitCast(inputStream, int8PtrTy));
        scanMatchInstance->setInternalState("FileSize", bufferSize);
        scanMatchInstance->setInternalState("FileIdx", fileIdx);
    }
    Value * initialBufferSize = nullptr;
    Value * initialBlockNo = nullptr;
    BasicBlock * initialBlock = nullptr;

    if (segmentSize > 1) {
        iBuilder->CreateBr(segmentCondBlock);
        iBuilder->SetInsertPoint(segmentCondBlock);
        PHINode * remainingBytes = iBuilder->CreatePHI(int64ty, 2, "remainingBytes");
        remainingBytes->addIncoming(bufferSize, entryBlock);
        PHINode * blockNo = iBuilder->CreatePHI(int64ty, 2, "blockNo");
        blockNo->addIncoming(iBuilder->getInt64(0), entryBlock);
        Constant * const step = ConstantInt::get(int64ty, mBlockSize * segmentSize * (UTF_16 ? 2 : 1));
        Value * segmentCondTest = iBuilder->CreateICmpULT(remainingBytes, step);
        iBuilder->CreateCondBr(segmentCondTest, fullCondBlock, segmentBodyBlock);
        iBuilder->SetInsertPoint(segmentBodyBlock);
        for (unsigned i = 0; i < segmentSize; ++i) {
            Value * blkNo = iBuilder->CreateAdd(blockNo, iBuilder->getInt64(i));
            make_DoBlock_Call(iBuilder, "s2p", {s2pI, ByteStream.getBlockPointer(blkNo), BasisBits.getBlockPointer(blkNo)});
        }
        for (unsigned i = 0; i < segmentSize; ++i) {
            Value * blkNo = iBuilder->CreateAdd(blockNo, iBuilder->getInt64(i));
            match_results_ptr = MatchResults.getBlockPointer(blkNo);
            make_DoBlock_Call(iBuilder, "icgrep", {grepI, BasisBits.getBlockPointer(blkNo), match_results_ptr});
            if (CountOnly) {
                Value * matchptr = iBuilder->CreateGEP(mBitBlockType, match_results_ptr, {iBuilder->getInt64(0), iBuilder->getInt32(0), iBuilder->getInt32(0)});
                Value * temp = iBuilder->CreateLoad(matchptr);
                Value * matches = iBuilder->CreateBitCast(temp, iBuilder->getIntNTy(mBlockSize));
                Value * popcount_for = generatePopcount(iBuilder, matches);
                Value * temp_count = iBuilder->CreateLoad(count);
                Value * prev_count = iBuilder->CreateBitCast(temp_count, iBuilder->getIntNTy(mBlockSize));
                Value * add_for = iBuilder->CreateAdd(prev_count, popcount_for);
                Value * add = iBuilder->CreateBitCast(add_for, mBitBlockType);
                iBuilder->CreateStore(add, count);
            }
        }
        if (!CountOnly) {
            for (unsigned i = 0; i < segmentSize; ++i) {
                Value * blkNo = iBuilder->CreateAdd(blockNo, iBuilder->getInt64(i));
                match_results_ptr = MatchResults.getBlockPointer(blkNo);
                make_DoBlock_Call(iBuilder, "scanMatch", {scanMatchInstance->getKernelState(), match_results_ptr});
            }
        }
        remainingBytes->addIncoming(iBuilder->CreateSub(remainingBytes, step), segmentBodyBlock);
        blockNo->addIncoming(iBuilder->CreateAdd(blockNo, iBuilder->getInt64(segmentSize)), segmentBodyBlock);
        iBuilder->CreateBr(segmentCondBlock);
        initialBufferSize = remainingBytes;
        initialBlock = segmentCondBlock;
        initialBlockNo = blockNo;
    } else {
        initialBufferSize = bufferSize;
        initialBlock = entryBlock;
        initialBlockNo = iBuilder->getInt64(0);
        iBuilder->CreateBr(fullCondBlock);
    }

    iBuilder->SetInsertPoint(fullCondBlock);
    PHINode * remainingBytes = iBuilder->CreatePHI(int64ty, 2, "remainingBytes");
    remainingBytes->addIncoming(initialBufferSize, initialBlock);
    PHINode * blockNo = iBuilder->CreatePHI(int64ty, 2, "blockNo");
    blockNo->addIncoming(initialBlockNo, initialBlock);
    
    Constant * const step = ConstantInt::get(int64ty, mBlockSize * (UTF_16 ? 2 : 1));
    Value * fullCondTest = iBuilder->CreateICmpULT(remainingBytes, step);
    iBuilder->CreateCondBr(fullCondTest, finalBlock, fullBodyBlock);

    iBuilder->SetInsertPoint(fullBodyBlock);
    errs() << "E\n";

    make_DoBlock_Call(iBuilder, "s2p", {s2pI, ByteStream.getBlockPointer(blockNo), BasisBits.getBlockPointer(blockNo)});
    errs() << "F\n";
    make_DoBlock_Call(iBuilder, "icgrep", {grepI, BasisBits.getBlockPointer(blockNo), MatchResults.getBlockPointer(blockNo)});
    if (CountOnly) {
        Value * matchptr = iBuilder->CreateGEP(mBitBlockType, match_results_ptr, {iBuilder->getInt64(0), iBuilder->getInt32(0)});
        Value * popcount = Cal_Count(matchptr, iBuilder);
        Value * temp_count = iBuilder->CreateLoad(count);
        Value * add = iBuilder->CreateAdd(temp_count, popcount);
        iBuilder->CreateStore(add, count);
    } else {
        make_DoBlock_Call(iBuilder, "scanMatch", {scanMatchInstance->getKernelState(), MatchResults.getBlockPointer(blockNo)});
    }

    remainingBytes->addIncoming(iBuilder->CreateSub(remainingBytes, step), fullBodyBlock);
    blockNo->addIncoming(iBuilder->CreateAdd(blockNo, iBuilder->getInt64(1)), fullBodyBlock);
    iBuilder->CreateBr(fullCondBlock);

    iBuilder->SetInsertPoint(finalBlock);
    basis_bits_ptr = BasisBits.getBlockPointer(blockNo);

    make_FinalBlock_Call(iBuilder, "s2p", {s2pI, remainingBytes, ByteStream.getBlockPointer(blockNo), basis_bits_ptr});
    
    errs() << "G\n";

    Value * remainingByte = iBuilder->CreateZExt(remainingBytes, iBuilder->getIntNTy(mBlockSize));
    Value * remainingUnit = iBuilder->CreateLShr(remainingByte, ConstantInt::get(iBuilder->getIntNTy(mBlockSize), 1));
    Value * EOFmark = iBuilder->CreateShl(ConstantInt::get(iBuilder->getIntNTy(mBlockSize), 1), UTF_16 ? remainingUnit : remainingByte);
    //icGrepInstance->setInternalState("EOFmark", iBuilder->CreateBitCast(EOFmark, mBitBlockType));
    match_results_ptr = MatchResults.getBlockPointer(blockNo);
    make_DoBlock_Call(iBuilder, "icgrep", {grepI, basis_bits_ptr, match_results_ptr});
    errs() << "H\n";
    if (CountOnly) {
        Value * matchptr = iBuilder->CreateGEP(mBitBlockType, match_results_ptr, {iBuilder->getInt64(0), iBuilder->getInt32(0)});
        Value * popcount1 = Cal_Count(matchptr, iBuilder);
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
        make_DoBlock_Call(iBuilder, "scanMatch", {scanMatchInstance->getKernelState(), match_results_ptr});
        iBuilder->CreateRetVoid();
    }
    return main;
}
