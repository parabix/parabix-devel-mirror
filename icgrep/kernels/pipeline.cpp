/*
 *  Copyright (c) 2016 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 */

#include "pipeline.h"
#include "toolchain.h"
#include "utf_encoding.h"

#include <kernels/scanmatchgen.h>
#include <kernels/s2p_kernel.h>

#include <pablo/function.h>
#include <pablo/pablo_compiler.h>


PipelineBuilder::PipelineBuilder(Module * m, IDISA::IDISA_Builder * b)
: mMod(m)
, iBuilder(b)
, mFileBufIdx(7)
, mFileSizeIdx(8)
, mFileNameIdx(9)
, mBitBlockType(b->getBitBlockType())
, mBlockSize(b->getBitBlockWidth()){

}
PipelineBuilder::~PipelineBuilder(){
}

void PipelineBuilder::CreateKernels(pablo::PabloFunction * function, bool isNameExpression){
    mS2PKernel = new KernelBuilder("s2p", mMod, iBuilder);
    mICgrepKernel = new KernelBuilder("icgrep", mMod, iBuilder);
    mScanMatchKernel = new KernelBuilder("scanMatch", mMod, iBuilder);


    generateS2PKernel(mMod, iBuilder, mS2PKernel);
    generateScanMatch(mMod, iBuilder, 64, mScanMatchKernel, isNameExpression);

    pablo_function_passes(function);
          
    pablo::PabloCompiler pablo_compiler(mMod, iBuilder);
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

void PipelineBuilder::ExecuteKernels(){
    Type * T = iBuilder->getIntNTy(64);   
    Type * S = PointerType::get(iBuilder->getIntNTy(8), 0);
    Type * inputType = PointerType::get(ArrayType::get(StructType::get(mMod->getContext(), std::vector<Type *>({ArrayType::get(mBitBlockType, 8)})), 1), 0);
 
    Constant* c = mMod->getOrInsertFunction("Main", Type::getVoidTy(mMod->getContext()), inputType, T, S, T, NULL);
    Function* mMainFunction = cast<Function>(c);
    mMainFunction->setCallingConv(CallingConv::C);
    Function::arg_iterator args = mMainFunction->arg_begin();


    Value* input_param = args++;
    input_param->setName("input");
    Value* buffersize_param = args++;
    buffersize_param->setName("buffersize");   
    Value* filename_param = args++;
    filename_param->setName("filename");      
    Value* finalLineUnterminated_param = args++;
    finalLineUnterminated_param->setName("finalLineUnterminated");

    iBuilder->SetInsertPoint(BasicBlock::Create(mMod->getContext(), "entry", mMainFunction,0));


    BasicBlock * entry_block = iBuilder->GetInsertBlock();
    BasicBlock * pipeline_test_block = BasicBlock::Create(mMod->getContext(), "pipeline_test_block", mMainFunction, 0);
    BasicBlock * pipeline_do_block = BasicBlock::Create(mMod->getContext(), "pipeline_do_block", mMainFunction, 0);
    BasicBlock * pipeline_final_block = BasicBlock::Create(mMod->getContext(), "pipeline_final_block", mMainFunction, 0);
    BasicBlock * pipeline_partial_block = BasicBlock::Create(mMod->getContext(), "pipeline_partial_block", mMainFunction, 0);
    BasicBlock * pipeline_empty_block = BasicBlock::Create(mMod->getContext(), "pipeline_empty_block", mMainFunction, 0);
    BasicBlock * pipeline_end_block = BasicBlock::Create(mMod->getContext(), "pipeline_end_block", mMainFunction, 0);    
    BasicBlock * pipeline_Unterminated_block = BasicBlock::Create(mMod->getContext(), "pipeline_Unterminated_block", mMainFunction, 0);
    BasicBlock * pipeline_return_block = BasicBlock::Create(mMod->getContext(), "pipeline_return_block", mMainFunction, 0);

    Value * s2pKernelStruct = mS2PKernel->generateKernelInstance();
    Value * icGrepKernelStruct = mICgrepKernel->generateKernelInstance();
    Value * scanMatchKernelStruct = mScanMatchKernel->generateKernelInstance();

    Value* filebuf = iBuilder->CreateBitCast(input_param, S);
    mScanMatchKernel->changeKernelInternalState(scanMatchKernelStruct, mFileBufIdx, filebuf);
    mScanMatchKernel->changeKernelInternalState(scanMatchKernelStruct, mFileSizeIdx, buffersize_param);
    mScanMatchKernel->changeKernelInternalState(scanMatchKernelStruct, mFileNameIdx, filename_param);

    Value * basis_bits = iBuilder->CreateGEP(s2pKernelStruct, {iBuilder->getInt32(0), iBuilder->getInt32(1)});
    Value * results = iBuilder->CreateGEP(icGrepKernelStruct, {iBuilder->getInt32(0), iBuilder->getInt32(1)});
  
    iBuilder->CreateBr(pipeline_test_block);

    iBuilder->SetInsertPoint(pipeline_test_block);
    PHINode * remaining_phi = iBuilder->CreatePHI(T, 2, "remaining");    
    PHINode * blkNo_phi = iBuilder->CreatePHI(T, 2, "blkNo");
    remaining_phi->addIncoming(buffersize_param, entry_block);
    blkNo_phi->addIncoming(iBuilder->getInt64(0), entry_block);

    Value * final_block_cond = iBuilder->CreateICmpSLT(remaining_phi, ConstantInt::get(T, mBlockSize));
    iBuilder->CreateCondBr(final_block_cond, pipeline_final_block, pipeline_do_block);

    iBuilder->SetInsertPoint(pipeline_do_block);

    Value * gep = iBuilder->CreateGEP(input_param, {blkNo_phi});
    Value * update_blkNo = iBuilder->CreateAdd(blkNo_phi, iBuilder->getInt64(1));
    blkNo_phi->addIncoming(update_blkNo, pipeline_do_block);

    mS2PKernel->generateDoBlockCall(gep);
    mICgrepKernel->generateDoBlockCall(basis_bits);
    mScanMatchKernel->generateDoBlockCall(results);

    Value * update_remaining = iBuilder->CreateSub(remaining_phi, iBuilder->getInt64(mBlockSize));
    remaining_phi->addIncoming(update_remaining, pipeline_do_block);
    iBuilder->CreateBr(pipeline_test_block);

    iBuilder->SetInsertPoint(pipeline_final_block);

    Value * empty_block_cond = iBuilder->CreateICmpEQ(remaining_phi, ConstantInt::get(T, 0));
    iBuilder->CreateCondBr(empty_block_cond, pipeline_empty_block, pipeline_partial_block);

    iBuilder->SetInsertPoint(pipeline_partial_block);

    gep = iBuilder->CreateGEP(input_param, {blkNo_phi});
    mS2PKernel->generateDoBlockCall(gep);
    iBuilder->CreateBr(pipeline_end_block);

    iBuilder->SetInsertPoint(pipeline_empty_block);

    iBuilder->CreateMemSet(basis_bits, iBuilder->getInt8(0), mBlockSize, 4);
    iBuilder->CreateBr(pipeline_end_block);

    iBuilder->SetInsertPoint(pipeline_end_block);

    Value * return_block_cond = iBuilder->CreateICmpEQ(finalLineUnterminated_param, ConstantInt::get(T, 0));
    iBuilder->CreateCondBr(return_block_cond, pipeline_return_block, pipeline_Unterminated_block);
    
    iBuilder->SetInsertPoint(pipeline_Unterminated_block);

    Value * remaining = iBuilder->CreateZExt(remaining_phi, iBuilder->getIntNTy(mBlockSize));
    Value * EOF_pos = iBuilder->CreateShl(ConstantInt::get(iBuilder->getIntNTy(mBlockSize), 1), remaining);
    EOF_pos = iBuilder->CreateBitCast(EOF_pos, mBitBlockType);

    Value * gep_bits4 = iBuilder->CreateGEP(basis_bits, {iBuilder->getInt32(0), iBuilder->getInt32(0), iBuilder->getInt32(4)});
    Value * bits4 = iBuilder->CreateAlignedLoad(gep_bits4, mBlockSize/8, false, "bits4");
    bits4 = iBuilder->CreateOr(bits4, EOF_pos);
    iBuilder->CreateAlignedStore(bits4, gep_bits4, mBlockSize/8, false);

    Value * gep_bits6 = iBuilder->CreateGEP(basis_bits, {iBuilder->getInt32(0), iBuilder->getInt32(0), iBuilder->getInt32(6)});
    Value * bits6 = iBuilder->CreateAlignedLoad(gep_bits6, mBlockSize/8, false, "bits6");
    bits6 = iBuilder->CreateOr(bits6, EOF_pos);
    iBuilder->CreateAlignedStore(bits6, gep_bits6, mBlockSize/8, false);
    iBuilder->CreateBr(pipeline_return_block);

    iBuilder->SetInsertPoint(pipeline_return_block);

    mICgrepKernel->generateDoBlockCall(basis_bits);
    mScanMatchKernel->generateDoBlockCall(results);
    iBuilder->CreateRetVoid();

}



