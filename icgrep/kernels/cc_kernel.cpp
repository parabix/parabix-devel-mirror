/*
 *  Copyright (c) 2016 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 */


#include "cc_kernel.h"


using namespace kernel;

void DirectCharacterClassKernelBuilder::generateDoBlockMethod() {
    IDISA::IDISA_Builder::InsertPoint savePoint = iBuilder->saveIP();
    Module * m = iBuilder->getModule();

    Function * doBlockFunction = m->getFunction(mKernelName + doBlock_suffix);
    
    iBuilder->SetInsertPoint(BasicBlock::Create(iBuilder->getContext(), "entry", doBlockFunction, 0));
    
    Value * self = getParameter(doBlockFunction, "self");
    Value * blockNo = getScalarField(self, blockNoScalar);
    
    Value * codeUnitStreamBlock_ptr = getStreamSetBlockPtr(self, "codeUnitStream", blockNo);
    Value * ccStreamBlock_ptr = getStreamSetBlockPtr(self, "ccStream", blockNo);

    unsigned packCount = 8 * mCodeUnitSize;  
    unsigned codeUnitWidth = 8 * mCodeUnitSize;
    Value * codeUnitPack[packCount];
    for (unsigned i = 0; i < packCount; i++) {
        codeUnitPack[i] = iBuilder->CreateBlockAlignedLoad(codeUnitStreamBlock_ptr, {iBuilder->getInt32(0), iBuilder->getInt32(0), iBuilder->getInt32(i)});
    }

    std::vector<Value *> ccStreams;
    for (unsigned j = 0; j < mCharClasses.size();  j++) {
        Value * theCCstream = iBuilder->allZeroes();
        for (const auto & interval : *mCharClasses[j]) {
            Value * strmPack[packCount];
            unsigned lo = re::lo_codepoint(interval);
            unsigned hi = re::hi_codepoint(interval);
            if (lo == hi) {
                Value * cp = ConstantInt::get(iBuilder->getIntNTy(codeUnitWidth), lo);
                Value * cp_splat = iBuilder->simd_fill(codeUnitWidth, cp);
                for (unsigned k = 0; k < packCount; k++) {
                    strmPack[k] = iBuilder->simd_eq(codeUnitWidth, codeUnitPack[k], cp_splat);
                }
            }
            else {
                Value * v1 = ConstantInt::get(iBuilder->getIntNTy(codeUnitWidth), lo-1);
                Value * lo_splat = iBuilder->simd_fill(codeUnitWidth, v1);
                Value * v2 = ConstantInt::get(iBuilder->getIntNTy(codeUnitWidth), hi+1);
                Value * hi_splat = iBuilder->simd_fill(codeUnitWidth, v2);
                for (unsigned k = 0; k < packCount; k++) {
                    Value * lo_test = iBuilder->simd_ugt(codeUnitWidth, codeUnitPack[k], lo_splat);
                    Value * hi_test = iBuilder->simd_ult(codeUnitWidth, codeUnitPack[k], hi_splat);
                    strmPack[k] = iBuilder->simd_and(lo_test, hi_test);
                }
            }
            unsigned packFields = iBuilder->getBitBlockWidth()/packCount;
            Value * pack = iBuilder->allZeroes();
            for (unsigned k = 0; k < packCount; k++) {
                pack = iBuilder->mvmd_insert(packFields, pack, iBuilder->CreateTrunc(iBuilder->hsimd_signmask(codeUnitWidth, strmPack[k]), iBuilder->getIntNTy(packFields)), k);
            }

            theCCstream = iBuilder->simd_or(theCCstream, pack);
        }
        iBuilder->CreateBlockAlignedStore(theCCstream, ccStreamBlock_ptr, {iBuilder->getInt32(0), iBuilder->getInt32(j)});
    }
 
    iBuilder->CreateRetVoid();
    iBuilder->restoreIP(savePoint);
}

