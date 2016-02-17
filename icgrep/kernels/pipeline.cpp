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
, mFilePosIdx(2)
, mBitBlockType(b->getBitBlockType())
, mBlockSize(b->getBitBlockWidth()){

}
PipelineBuilder::~PipelineBuilder(){
}

void PipelineBuilder::CreateKernels(re::RE * re_ast){
    mS2PKernel = new KernelBuilder("s2p", mMod, iBuilder);
    mICgrepKernel = new KernelBuilder("icgrep", mMod, iBuilder);
    mScanMatchKernel = new KernelBuilder("scanMatch", mMod, iBuilder);


    generateS2PKernel(mMod, iBuilder, mS2PKernel);
    generateScanMatch(mMod, iBuilder, 64, mScanMatchKernel);

    Encoding encoding(Encoding::Type::UTF_8, 8);
    re_ast = regular_expression_passes(encoding, re_ast);
    
    pablo::PabloFunction * function = re2pablo_compiler(encoding, re_ast);

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
 
    Constant* c = mMod->getOrInsertFunction("Main", Type::getVoidTy(mMod->getContext()), inputType, T, S, NULL);
    Function* mMainFunction = cast<Function>(c);
    mMainFunction->setCallingConv(CallingConv::C);
    Function::arg_iterator args = mMainFunction->arg_begin();


    Value* input_param = args++;
    input_param->setName("input");
    Value* buffersize_param = args++;
    buffersize_param->setName("buffersize");   
    Value* filename_param = args++;
    filename_param->setName("filename");

    iBuilder->SetInsertPoint(BasicBlock::Create(mMod->getContext(), "entry", mMainFunction,0));


    BasicBlock * entry_block = iBuilder->GetInsertBlock();
    BasicBlock * pipeline_test_block = BasicBlock::Create(mMod->getContext(), "pipeline_test_block", mMainFunction, 0);
    BasicBlock * pipeline_do_block = BasicBlock::Create(mMod->getContext(), "pipeline_do_block", mMainFunction, 0);
    BasicBlock * pipeline_partial_block = BasicBlock::Create(mMod->getContext(), "pipeline_partial_block", mMainFunction, 0);

    Value * s2pKernelStruct = mS2PKernel->generateKernelInstance();
    Value * icGrepKernelStruct = mICgrepKernel->generateKernelInstance();
    Value * scanMatchKernelStruct = mScanMatchKernel->generateKernelInstance();
    iBuilder->CreateBr(pipeline_test_block);

    iBuilder->SetInsertPoint(pipeline_test_block);
    PHINode * remaining_phi = iBuilder->CreatePHI(T, 2, "remaining");    
    PHINode * blkNo_phi = iBuilder->CreatePHI(T, 2, "blkNo");
    remaining_phi->addIncoming(buffersize_param, entry_block);
    blkNo_phi->addIncoming(iBuilder->getInt64(0), entry_block);
    Value * cond = iBuilder->CreateICmpSLT(remaining_phi, ConstantInt::get(T, mBlockSize));
    iBuilder->CreateCondBr(cond, pipeline_partial_block, pipeline_do_block);


    iBuilder->SetInsertPoint(pipeline_do_block);

    Value * gep = iBuilder->CreateGEP(input_param, {blkNo_phi});
    mS2PKernel->generateDoBlockCall(gep);
    Value * update_blkNo = iBuilder->CreateAdd(blkNo_phi, iBuilder->getInt64(1));
    blkNo_phi->addIncoming(update_blkNo, pipeline_do_block);

    Value * basis_bits = iBuilder->CreateGEP(s2pKernelStruct, {iBuilder->getInt32(0), iBuilder->getInt32(1)});
    mICgrepKernel->generateDoBlockCall(basis_bits);

    gep = iBuilder->CreateGEP(scanMatchKernelStruct, {iBuilder->getInt64(0), iBuilder->getInt32(0), iBuilder->getInt32(7)});
    Value* filebuf = iBuilder->CreateBitCast(input_param, S);
    iBuilder->CreateStore(filebuf, gep);
    gep = iBuilder->CreateGEP(scanMatchKernelStruct, {iBuilder->getInt64(0), iBuilder->getInt32(0), iBuilder->getInt32(8)});
    iBuilder->CreateStore(buffersize_param, gep);
    gep = iBuilder->CreateGEP(scanMatchKernelStruct, {iBuilder->getInt64(0), iBuilder->getInt32(0), iBuilder->getInt32(9)});
    iBuilder->CreateStore(filename_param, gep);

    Value * results = iBuilder->CreateGEP(icGrepKernelStruct, {iBuilder->getInt32(0), iBuilder->getInt32(1)});
    mScanMatchKernel->generateDoBlockCall(results);

    Value * update_remaining = iBuilder->CreateSub(remaining_phi, iBuilder->getInt64(iBuilder->getBitBlockWidth()));
    remaining_phi->addIncoming(update_remaining, pipeline_do_block);
    iBuilder->CreateBr(pipeline_test_block);

    iBuilder->SetInsertPoint(pipeline_partial_block);
    iBuilder->CreateRetVoid();

}



