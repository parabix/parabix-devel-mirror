/*
 *  Copyright (c) 2016 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 */

#include "pipeline.h"
#include "utf_encoding.h"

#include <kernels/scanmatchgen.h>
#include <kernels/s2p_kernel.h>
#include <kernels/instance.h>

#include <pablo/function.h>
#include <pablo/pablo_compiler.h>
#include <pablo/pablo_toolchain.h>


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
    delete mICgrepKernel;
    delete mScanMatchKernel;
}

void PipelineBuilder::CreateKernels(PabloFunction * function, bool isNameExpression){
    mS2PKernel = new KernelBuilder("s2p", mMod, iBuilder);
    mICgrepKernel = new KernelBuilder("icgrep", mMod, iBuilder);
    mScanMatchKernel = new KernelBuilder("scanMatch", mMod, iBuilder);

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

void PipelineBuilder::ExecuteKernels() {
    Type * T = iBuilder->getIntNTy(64);   
    Type * S = PointerType::get(iBuilder->getIntNTy(8), 0);
    Type * inputType = PointerType::get(ArrayType::get(StructType::get(mMod->getContext(), std::vector<Type *>({ArrayType::get(mBitBlockType, 8)})), 1), 0); 
    Function * const main = cast<Function>(mMod->getOrInsertFunction("Main", Type::getVoidTy(mMod->getContext()), inputType, T, S, T, nullptr));
    main->setCallingConv(CallingConv::C);
    Function::arg_iterator args = main->arg_begin();

    Value * const inputStream = args++;
    inputStream->setName("input");
    Value* buffersize_param = args++;
    buffersize_param->setName("buffersize");   
    Value* filename_param = args++;
    filename_param->setName("filename");      
    Value* finalLineUnterminated_param = args++;
    finalLineUnterminated_param->setName("finalLineUnterminated");

    iBuilder->SetInsertPoint(BasicBlock::Create(mMod->getContext(), "entry", main,0));

    BasicBlock * entry_block = iBuilder->GetInsertBlock();
    BasicBlock * pipeline_test_block = BasicBlock::Create(mMod->getContext(), "pipeline_test_block", main, 0);
    BasicBlock * pipeline_do_block = BasicBlock::Create(mMod->getContext(), "pipeline_do_block", main, 0);
    BasicBlock * pipeline_final_block = BasicBlock::Create(mMod->getContext(), "pipeline_final_block", main, 0);
    BasicBlock * pipeline_partial_block = BasicBlock::Create(mMod->getContext(), "pipeline_partial_block", main, 0);
    BasicBlock * pipeline_empty_block = BasicBlock::Create(mMod->getContext(), "pipeline_empty_block", main, 0);
    BasicBlock * pipeline_end_block = BasicBlock::Create(mMod->getContext(), "pipeline_end_block", main, 0);
    BasicBlock * pipeline_Unterminated_block = BasicBlock::Create(mMod->getContext(), "pipeline_Unterminated_block", main, 0);
    BasicBlock * pipeline_return_block = BasicBlock::Create(mMod->getContext(), "pipeline_return_block", main, 0);

    Instance * s2pInstance = mS2PKernel->instantiate();
    Instance * icGrepInstance = mICgrepKernel->instantiate();
    Instance * scanMatchInstance = mScanMatchKernel->instantiate();


    Value * gep = scanMatchInstance->getInternalState("FileBuf");
    Value * filebuf = iBuilder->CreateBitCast(inputStream, S);
    iBuilder->CreateStore(filebuf, gep);

    gep = scanMatchInstance->getInternalState("FileSize");
    iBuilder->CreateStore(buffersize_param, gep);

    gep = scanMatchInstance->getInternalState("FileName");
    iBuilder->CreateStore(filename_param, gep);

    Value * basis_bits = s2pInstance->getOutputStreamSet();

    Value * results = icGrepInstance->getOutputStreamSet();

    iBuilder->CreateBr(pipeline_test_block);

    iBuilder->SetInsertPoint(pipeline_test_block);
    PHINode * blockNo = iBuilder->CreatePHI(T, 2, "blockNo");
    blockNo->addIncoming(iBuilder->getInt64(0), entry_block);
    PHINode * remainingBytes = iBuilder->CreatePHI(T, 2, "remainingBytes");
    remainingBytes->addIncoming(buffersize_param, entry_block);

    Constant * step = ConstantInt::get(T, mBlockSize * mS2PKernel->getSegmentBlocks());

    Value * final_block_cond = iBuilder->CreateICmpSLT(remainingBytes, step);
    iBuilder->CreateCondBr(final_block_cond, pipeline_final_block, pipeline_do_block);

    iBuilder->SetInsertPoint(pipeline_do_block);

    s2pInstance->call(iBuilder->CreateGEP(inputStream, blockNo));

    icGrepInstance->call(basis_bits);
    scanMatchInstance->call(results);

    blockNo->addIncoming(iBuilder->CreateAdd(blockNo, iBuilder->getInt64(1)), pipeline_do_block);
    Value * update_remaining = iBuilder->CreateSub(remainingBytes, step);
    remainingBytes->addIncoming(update_remaining, pipeline_do_block);
    iBuilder->CreateBr(pipeline_test_block);

    iBuilder->SetInsertPoint(pipeline_final_block);

    Value * empty_block_cond = iBuilder->CreateICmpEQ(remainingBytes, ConstantInt::get(T, 0));
    iBuilder->CreateCondBr(empty_block_cond, pipeline_empty_block, pipeline_partial_block);

    iBuilder->SetInsertPoint(pipeline_partial_block);

    s2pInstance->call(iBuilder->CreateGEP(inputStream, blockNo));

    iBuilder->CreateBr(pipeline_end_block);

    iBuilder->SetInsertPoint(pipeline_empty_block);

    iBuilder->CreateMemSet(basis_bits, iBuilder->getInt8(0), mBlockSize, 4);
    iBuilder->CreateBr(pipeline_end_block);

    iBuilder->SetInsertPoint(pipeline_end_block);

    Value * return_block_cond = iBuilder->CreateICmpEQ(finalLineUnterminated_param, ConstantInt::get(T, 0));
    iBuilder->CreateCondBr(return_block_cond, pipeline_return_block, pipeline_Unterminated_block);
    
    iBuilder->SetInsertPoint(pipeline_Unterminated_block);

    Value * remaining = iBuilder->CreateZExt(remainingBytes, iBuilder->getIntNTy(mBlockSize));
    Value * EOF_pos = iBuilder->CreateShl(ConstantInt::get(iBuilder->getIntNTy(mBlockSize), 1), remaining);
    EOF_pos = iBuilder->CreateBitCast(EOF_pos, mBitBlockType);

    Value * gep_bits4 = s2pInstance->getOutputStream(4);
    Value * bits4 = iBuilder->CreateBlockAlignedLoad(gep_bits4);
    bits4 = iBuilder->CreateOr(bits4, EOF_pos);
    iBuilder->CreateBlockAlignedStore(bits4, gep_bits4);

    Value * gep_bits6 = s2pInstance->getOutputStream(6);
    Value * bits6 = iBuilder->CreateBlockAlignedLoad(gep_bits6);
    bits6 = iBuilder->CreateOr(bits6, EOF_pos);
    iBuilder->CreateBlockAlignedStore(bits6, gep_bits6);

    iBuilder->CreateBr(pipeline_return_block);

    iBuilder->SetInsertPoint(pipeline_return_block);

    icGrepInstance->call(basis_bits);
    scanMatchInstance->call(results);
    iBuilder->CreateRetVoid();

}
