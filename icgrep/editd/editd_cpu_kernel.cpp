/*
 *  Copyright (c) 2016 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 */

#include "editd_cpu_kernel.h"
#include <IR_Gen/idisa_builder.h>
#include <llvm/IR/Module.h>

using namespace llvm;

namespace kernel {

void editdCPUKernel::bitblock_advance_ci_co(Value * val, unsigned shift, Value * stideCarryArr, unsigned carryIdx, std::vector<std::vector<Value *>> & adv, std::vector<std::vector<int>> & calculated, int i, int j) const {
    if (calculated[i][j] == 0) {
        Value * ptr = iBuilder->CreateGEP(stideCarryArr, {iBuilder->getInt32(0), iBuilder->getInt32(carryIdx)});
        Value * ci = iBuilder->CreateLoad(ptr);
        std::pair<Value *, Value *> rslt = iBuilder->bitblock_advance(val, ci, shift);
        iBuilder->CreateStore(std::get<0>(rslt), ptr);
        adv[i][j] = std::get<1>(rslt);
        calculated[i][j] = 1;
    }
}

void editdCPUKernel::generateDoBlockMethod(Value * blockNo) {
    auto savePoint = iBuilder->saveIP();

    Type * const int32ty = iBuilder->getInt32Ty();
    Type * const int8ty = iBuilder->getInt8Ty();

    Value * pattStartPtr = getScalarField("pattStream");
    Value * stideCarryArr = getScalarField("srideCarry");

    unsigned carryIdx = 0;

    std::vector<std::vector<Value *>> e(mPatternLen + 1, std::vector<Value *>(mEditDistance + 1));
    std::vector<std::vector<Value *>> adv(mPatternLen, std::vector<Value *>(mEditDistance + 1));
    std::vector<std::vector<int>> calculated(mPatternLen, std::vector<int>(mEditDistance + 1, 0));
    Value * pattPos = iBuilder->getInt32(0);
    Value * pattPtr = iBuilder->CreateGEP(pattStartPtr, pattPos);
    Value * pattCh = iBuilder->CreateLoad(pattPtr);
    Value * pattIdx = iBuilder->CreateAnd(iBuilder->CreateLShr(pattCh, 1), ConstantInt::get(int8ty, 3));
    Value * pattStreamPtr = getStream("CCStream", blockNo, iBuilder->CreateZExt(pattIdx, int32ty));
    Value * pattStream = iBuilder->CreateLoad(pattStreamPtr);
    pattPos = iBuilder->CreateAdd(pattPos, ConstantInt::get(int32ty, 1));

    e[0][0] = pattStream;
    for(unsigned j = 1; j <= mEditDistance; j++){
      e[0][j] = iBuilder->allOnes();
    }

    for(unsigned i = 1; i < mPatternLen; i++){
        pattPtr = iBuilder->CreateGEP(pattStartPtr, pattPos);
        pattCh = iBuilder->CreateLoad(pattPtr);
        pattIdx = iBuilder->CreateAnd(iBuilder->CreateLShr(pattCh, 1), ConstantInt::get(int8ty, 3));
        pattStreamPtr = getStream("CCStream", blockNo, iBuilder->CreateZExt(pattIdx, int32ty));
        pattStream = iBuilder->CreateLoad(pattStreamPtr);

        bitblock_advance_ci_co(e[i-1][0], 1, stideCarryArr, carryIdx++, adv, calculated, i-1, 0);
        e[i][0] = iBuilder->CreateAnd(adv[i-1][0], pattStream); 
        for(unsigned j = 1; j<= mEditDistance; j++){
            bitblock_advance_ci_co(e[i-1][j], 1, stideCarryArr, carryIdx++, adv, calculated, i-1, j);
            bitblock_advance_ci_co(e[i-1][j-1], 1, stideCarryArr, carryIdx++, adv, calculated, i-1, j-1);
            bitblock_advance_ci_co(e[i][j-1], 1, stideCarryArr, carryIdx++, adv, calculated, i, j-1);
            Value * tmp1 = iBuilder->CreateAnd(adv[i-1][j], pattStream);
            Value * tmp2 = iBuilder->CreateAnd(adv[i-1][j-1], iBuilder->CreateNot(pattStream));
            Value * tmp3 = iBuilder->CreateOr(adv[i][j-1], e[i-1][j-1]);
            e[i][j] = iBuilder->CreateOr(iBuilder->CreateOr(tmp1, tmp2), tmp3);

        }
        pattPos = iBuilder->CreateAdd(pattPos, ConstantInt::get(int32ty, 1));
    }
    
    Value * ptr = getStream("ResultStream", blockNo, iBuilder->getInt32(0));
    iBuilder->CreateStore(e[mPatternLen - 1][0], ptr);
    for(unsigned j = 1; j<= mEditDistance; j++){
        ptr = getStream("ResultStream", blockNo, iBuilder->getInt32(j));
        iBuilder->CreateStore(iBuilder->CreateAnd(e[mPatternLen-1][j], iBuilder->CreateNot(e[mPatternLen-1][j-1])), ptr);
    }
       
    iBuilder->CreateRetVoid();
    iBuilder->restoreIP(savePoint);
}

void editdCPUKernel::generateFinalBlockMethod(Value * remainingBytes, Value * blockNo) {
    setScalarField("EOFmask", iBuilder->bitblock_mask_from(remainingBytes));
    CreateDoBlockMethodCall();
}

editdCPUKernel::editdCPUKernel(IDISA::IDISA_Builder * b, unsigned dist, unsigned pattLen) :
BlockOrientedKernel(b, "editd_cpu",
             {Binding{b->getStreamSetTy(4), "CCStream"}},
             {Binding{b->getStreamSetTy(dist + 1), "ResultStream"}},
             {Binding{PointerType::get(b->getInt8Ty(), 1), "pattStream"},
             Binding{PointerType::get(ArrayType::get(b->getBitBlockType(), pattLen * (dist + 1) * 4), 0), "srideCarry"}},
             {},
             {Binding{b->getBitBlockType(), "EOFmask"}}),
mEditDistance(dist),
mPatternLen(pattLen){
setDoBlockUpdatesProducedItemCountsAttribute(false);
}

}


