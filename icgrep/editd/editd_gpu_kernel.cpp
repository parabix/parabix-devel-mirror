/*
 *  Copyright (c) 2016 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 */
#include "editd_gpu_kernel.h"
#include <kernels/kernel.h>
#include <IR_Gen/idisa_builder.h>
#include <llvm/Support/raw_ostream.h>
#include <iostream>

namespace kernel {
using namespace llvm;


void bitblock_advance_ci_co(IDISA::IDISA_Builder * iBuilder, Value * val, unsigned shift, Value * stideCarryArr, unsigned carryIdx, std::vector<std::vector<Value *>> & adv, std::vector<std::vector<int>> & calculated, int i, int j){   
    if(!calculated[i][j]){
        Value * ptr = iBuilder->CreateGEP(stideCarryArr, {iBuilder->getInt32(0), iBuilder->getInt32(carryIdx)});
        Value * ci = iBuilder->CreateLoad(ptr);
        std::pair<Value *, Value *> rslt = iBuilder->bitblock_advance(val, ci, shift);
        iBuilder->CreateStore(std::get<0>(rslt), ptr);
        adv[i][j] = std::get<1>(rslt);
        calculated[i][j] = 1;
    }
    return;
}

void editdGPUKernel::generateFinalBlockMethod() {
    IDISA::IDISA_Builder::InsertPoint savePoint = iBuilder->saveIP();
    Module * m = iBuilder->getModule();
    Function * doBlockFunction = m->getFunction(mKernelName + doBlock_suffix);
    Function * finalBlockFunction = m->getFunction(mKernelName + finalBlock_suffix);
    iBuilder->SetInsertPoint(BasicBlock::Create(iBuilder->getContext(), "fb_entry", finalBlockFunction, 0));
    // Final Block arguments: self, remaining, then the standard DoBlock args.
    Function::arg_iterator args = finalBlockFunction->arg_begin();
    Value * self = &*(args++);
    Value * remaining = &*(args++);
    std::vector<Value *> doBlockArgs = {self};
    while (args != finalBlockFunction->arg_end()){
        doBlockArgs.push_back(&*args++);
    }
    setScalarField(self, "EOFmask", iBuilder->bitblock_mask_from(remaining));
    iBuilder->CreateCall(doBlockFunction, doBlockArgs);
    /* Adjust the produced item count */
    Value * produced = getProducedItemCount(self);
    produced = iBuilder->CreateSub(produced, ConstantInt::get(iBuilder->getSizeTy(), iBuilder->getStride()));
    setProducedItemCount(self, iBuilder->CreateAdd(produced, remaining));
    iBuilder->CreateRetVoid();
    iBuilder->restoreIP(savePoint);
}
    
void editdGPUKernel::generateDoBlockMethod() {
    IDISA::IDISA_Builder::InsertPoint savePoint = iBuilder->saveIP();
    Module * m = iBuilder->getModule();  

    Type * const int32ty = iBuilder->getInt32Ty();
    Type * const int8ty = iBuilder->getInt8Ty();

    Function * doBlockFunction = m->getFunction(mKernelName + doBlock_suffix);
       
    BasicBlock * entryBlock = BasicBlock::Create(iBuilder->getContext(), "entry", doBlockFunction, 0);
    
    iBuilder->SetInsertPoint(entryBlock);

    Value * kernelStuctParam = getParameter(doBlockFunction, "self");
    Value * pattBuf = getScalarField(kernelStuctParam, "pattStream");
    Value * stideCarryArr = getScalarField(kernelStuctParam, "srideCarry");
    Value * blockNo = getScalarField(kernelStuctParam, blockNoScalar);
    Value * ccStreamPtr = getStreamSetBlockPtr(kernelStuctParam, "CCStream", blockNo);
    Value * resultStreamPtr = getStreamSetBlockPtr(kernelStuctParam, "ResultStream", blockNo);
    Value * pattLen = ConstantInt::get(int32ty, mPatternLen+1);
    Value * pattPos = ConstantInt::get(int32ty, 0);
   
    unsigned carryIdx = 0;

    std::vector<std::vector<Value *>> e(mPatternLen, std::vector<Value *>(mEditDistance+1));
    std::vector<std::vector<Value *>> adv(mPatternLen, std::vector<Value *>(mEditDistance+1));
    std::vector<std::vector<int>> calculated(mPatternLen, std::vector<int>(mEditDistance+1));
    for(unsigned i=0; i<mPatternLen; i++)
        for(unsigned j=0; j<=mEditDistance; j++)
            calculated[i][j] = 0;
    Function * bidFunc = cast<Function>(m->getOrInsertFunction("llvm.nvvm.read.ptx.sreg.ctaid.x", int32ty, nullptr));
    Value * bid = iBuilder->CreateCall(bidFunc);
    Value * pattStartPtr = iBuilder->CreateGEP(pattBuf, iBuilder->CreateMul(pattLen, bid));
    Value * pattPtr = iBuilder->CreateGEP(pattStartPtr, pattPos);
    Value * pattCh = iBuilder->CreateLoad(pattPtr);
    Value * pattIdx = iBuilder->CreateAnd(iBuilder->CreateLShr(pattCh, 1), ConstantInt::get(int8ty, 3));
    Value * pattStreamPtr = iBuilder->CreateGEP(ccStreamPtr, {iBuilder->getInt32(0), iBuilder->CreateZExt(pattIdx, int32ty)});
    Value * pattStream = iBuilder->CreateLoad(pattStreamPtr);
    pattPos = iBuilder->CreateAdd(pattPos, ConstantInt::get(int32ty, 1));

    e[0][0] = pattStream;
    for(unsigned j = 1; j <= mEditDistance; j++){
      e[0][j] = iBuilder->allOnes();
    }

    for(unsigned i = 1; i<mPatternLen; i++){     
        pattPtr = iBuilder->CreateGEP(pattStartPtr, pattPos);
        pattCh = iBuilder->CreateLoad(pattPtr);
        pattIdx = iBuilder->CreateAnd(iBuilder->CreateLShr(pattCh, 1), ConstantInt::get(int8ty, 3));
        pattStreamPtr = iBuilder->CreateGEP(ccStreamPtr, {iBuilder->getInt32(0), iBuilder->CreateZExt(pattIdx, int32ty)});
        pattStream = iBuilder->CreateLoad(pattStreamPtr);

        bitblock_advance_ci_co(iBuilder, e[i-1][0], 1, stideCarryArr, carryIdx++, adv, calculated, i-1, 0);
        e[i][0] = iBuilder->CreateAnd(adv[i-1][0], pattStream); 
        for(unsigned j = 1; j<= mEditDistance; j++){
            bitblock_advance_ci_co(iBuilder, e[i-1][j], 1, stideCarryArr, carryIdx++, adv, calculated, i-1, j);
            bitblock_advance_ci_co(iBuilder, e[i-1][j-1], 1, stideCarryArr, carryIdx++, adv, calculated, i-1, j-1);
            bitblock_advance_ci_co(iBuilder, e[i][j-1], 1, stideCarryArr, carryIdx++, adv, calculated, i, j-1);
            Value * tmp1 = iBuilder->CreateAnd(adv[i-1][j], pattStream);
            Value * tmp2 = iBuilder->CreateAnd(adv[i-1][j-1], iBuilder->CreateNot(pattStream));
            Value * tmp3 = iBuilder->CreateOr(adv[i][j-1], e[i-1][j-1]);
            e[i][j] = iBuilder->CreateOr(iBuilder->CreateOr(tmp1, tmp2), tmp3);

        }
        pattPos = iBuilder->CreateAdd(pattPos, ConstantInt::get(int32ty, 1));
    }

    Value * ptr = iBuilder->CreateGEP(resultStreamPtr, {iBuilder->getInt32(0), iBuilder->getInt32(0)});
    iBuilder->CreateStore(e[mPatternLen-1][0], ptr);
    for(unsigned j = 1; j<= mEditDistance; j++){
        ptr = iBuilder->CreateGEP(resultStreamPtr, {iBuilder->getInt32(0), iBuilder->getInt32(j)});
        iBuilder->CreateStore(iBuilder->CreateAnd(e[mPatternLen-1][j], iBuilder->CreateNot(e[mPatternLen-1][j-1])), ptr);
    }

    Value * produced = getProducedItemCount(kernelStuctParam);
    produced = iBuilder->CreateAdd(produced, ConstantInt::get(iBuilder->getSizeTy(), iBuilder->getStride()));
    setProducedItemCount(kernelStuctParam, produced); 
       
    iBuilder->CreateRetVoid();
    iBuilder->restoreIP(savePoint);
}

}


