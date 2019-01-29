/*
 *  Copyright (c) 2016 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 */
#include "editd_gpu_kernel.h"
#include <kernels/kernel_builder.h>
#include <llvm/IR/Module.h>
#include <iostream>
using namespace llvm;

namespace kernel {

void bitblock_advance_ci_co(const std::unique_ptr<KernelBuilder> & iBuilder, Value * val, unsigned shift, Value * stideCarryArr, unsigned carryIdx, std::vector<std::vector<Value *>> & adv, std::vector<std::vector<int>> & calculated, int i, int j){
    if (!calculated[i][j]) {
        Value * ptr = iBuilder->CreateGEP(stideCarryArr, {iBuilder->getInt32(0), iBuilder->getInt32(carryIdx)});
        Value * ci = iBuilder->CreateLoad(ptr);
        std::pair<Value *, Value *> rslt = iBuilder->bitblock_advance(val, ci, shift);
        iBuilder->CreateStore(std::get<0>(rslt), ptr);
        adv[i][j] = std::get<1>(rslt);
        calculated[i][j] = 1;
    }
}

void reset_to_zero(std::vector<std::vector<int>> & calculated){
    for (auto & sub : calculated) {
        std::fill(sub.begin(), sub.end(), 0);
    }
}

void editdGPUKernel::generateDoBlockMethod(const std::unique_ptr<KernelBuilder> & idb) {

    IntegerType * const int32ty = idb->getInt32Ty();
    IntegerType * const int8ty = idb->getInt8Ty();
    Value * groupLen = idb->getInt32((mPatternLen + 1) * mGroupSize);
    Value * pattPos = idb->getInt32(0);
    Value * pattBuf = idb->getScalarField("pattStream");
    Value * strideCarryArr = idb->getScalarField("strideCarry");

    unsigned carryIdx = 0;

    std::vector<std::vector<Value *>> e(mPatternLen + 1, std::vector<Value *>(mEditDistance + 1));
    std::vector<std::vector<Value *>> adv(mPatternLen, std::vector<Value *>(mEditDistance + 1));
    std::vector<std::vector<int>> calculated(mPatternLen, std::vector<int>(mEditDistance + 1, 0));

    Module * m = idb->getModule();
    Function * bidFunc = cast<Function>(m->getOrInsertFunction("llvm.nvvm.read.ptx.sreg.ctaid.x", FunctionType::get(int32ty, {})));
    Value * bid = idb->CreateCall(bidFunc);
    Value * pattStartPtr = idb->CreateGEP(pattBuf, idb->CreateMul(groupLen, bid));

    for(unsigned j = 0; j <= mEditDistance; j++){
        e[mPatternLen][j] = idb->allZeroes();
    }

    for(unsigned j = 1; j <= mEditDistance; j++){
        e[0][j] = idb->allOnes();
    }

    for(unsigned g = 0; g < mGroupSize; g++){
        Value * pattCh = idb->CreateLoad(idb->CreateGEP(pattStartPtr, pattPos));
        Value * pattIdx = idb->CreateAnd(idb->CreateLShr(pattCh, 1), ConstantInt::get(int8ty, 3));
        Value * pattStream = idb->loadInputStreamBlock("CCStream", idb->CreateZExt(pattIdx, int32ty));
        pattPos = idb->CreateAdd(pattPos, ConstantInt::get(int32ty, 1));
        e[0][0] = pattStream;
        for(unsigned i = 1; i < mPatternLen; i++){
            Value * pattCh = idb->CreateLoad(idb->CreateGEP(pattStartPtr, pattPos));
            pattIdx = idb->CreateAnd(idb->CreateLShr(pattCh, 1), ConstantInt::get(int8ty, 3));
            pattStream = idb->loadInputStreamBlock("CCStream", idb->CreateZExt(pattIdx, int32ty));
            pattPos = idb->CreateAdd(pattPos, ConstantInt::get(int32ty, 1));
            bitblock_advance_ci_co(idb, e[i-1][0], 1, strideCarryArr, carryIdx++, adv, calculated, i-1, 0);
            e[i][0] = idb->CreateAnd(adv[i-1][0], pattStream);
            for(unsigned j = 1; j<= mEditDistance; j++){
                bitblock_advance_ci_co(idb, e[i-1][j], 1, strideCarryArr, carryIdx++, adv, calculated, i-1, j);
                bitblock_advance_ci_co(idb, e[i-1][j-1], 1, strideCarryArr, carryIdx++, adv, calculated, i-1, j-1);
                bitblock_advance_ci_co(idb, e[i][j-1], 1, strideCarryArr, carryIdx++, adv, calculated, i, j-1);
                Value * tmp1 = idb->CreateAnd(adv[i-1][j], pattStream);
                Value * tmp2 = idb->CreateAnd(adv[i-1][j-1], idb->CreateNot(pattStream));
                Value * tmp3 = idb->CreateOr(adv[i][j-1], e[i-1][j-1]);
                e[i][j] = idb->CreateOr(idb->CreateOr(tmp1, tmp2), tmp3);
            }
        }
        e[mPatternLen][0] = idb->CreateOr(e[mPatternLen][0], e[mPatternLen-1][0]);
        for(unsigned j = 1; j<= mEditDistance; j++){
            e[mPatternLen][j] = idb->CreateOr(e[mPatternLen][j], idb->CreateAnd(e[mPatternLen - 1][j], idb->CreateNot(e[mPatternLen - 1][j - 1])));
        }
        pattPos = idb->CreateAdd(pattPos, ConstantInt::get(int32ty, 1));
        reset_to_zero(calculated);
    }

    for(unsigned j = 0; j<= mEditDistance; j++){
        idb->storeOutputStreamBlock("ResultStream", idb->getInt32(j), e[mPatternLen][j]);
    }
}

void editdGPUKernel::generateFinalBlockMethod(const std::unique_ptr<KernelBuilder> & idb, Value * remainingBytes) {
    idb->setScalarField("EOFmask", idb->bitblock_mask_from(remainingBytes));
    CreateDoBlockMethodCall(idb);
}

editdGPUKernel::editdGPUKernel(const std::unique_ptr<kernel::KernelBuilder> & b, unsigned dist, unsigned pattLen, unsigned groupSize) :
BlockOrientedKernel(b, "editd_gpu",
              {Binding{b->getStreamSetTy(4), "CCStream"}},
              {Binding{b->getStreamSetTy(dist + 1), "ResultStream"}},
              {Binding{PointerType::get(b->getInt8Ty(), 1), "pattStream"},
              Binding{PointerType::get(ArrayType::get(b->getBitBlockType(), pattLen * (dist + 1) * 4 * groupSize), 0), "strideCarry"}},
              {},
              {Binding{b->getBitBlockType(), "EOFmask"}})
, mEditDistance(dist)
, mPatternLen(pattLen)
, mGroupSize(groupSize) {
}

}


