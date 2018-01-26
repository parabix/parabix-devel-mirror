/*
 *  Copyright (c) 2016 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 */

#include "cc_kernel.h"
#include <re/re_cc.h>
#include <cc/cc_compiler.h>
#include <kernels/kernel_builder.h>

using namespace cc;
using namespace kernel;
using namespace pablo;
using namespace re;
using namespace llvm;

DirectCharacterClassKernelBuilder::DirectCharacterClassKernelBuilder(
        const std::unique_ptr<kernel::KernelBuilder> & b, std::string ccSetName, std::vector<re::CC *> charClasses, unsigned codeUnitSize)
: BlockOrientedKernel(std::move(ccSetName),
              {Binding{b->getStreamSetTy(1, 8 * codeUnitSize), "codeUnitStream", FixedRate(), Principal()}},
              {Binding{b->getStreamSetTy(charClasses.size(), 1), "ccStream"}},
              {}, {}, {})
, mCharClasses(charClasses)
, mCodeUnitSize(codeUnitSize) {

}

void DirectCharacterClassKernelBuilder::generateDoBlockMethod(const std::unique_ptr<KernelBuilder> & iBuilder) {
    unsigned packCount = 8 * mCodeUnitSize;  
    unsigned codeUnitWidth = 8 * mCodeUnitSize;
    unsigned topBit = 1 << codeUnitWidth;
    unsigned maxCodeVal = (topBit - 1) | topBit;
    Value * codeUnitPack[packCount];
    for (unsigned i = 0; i < packCount; i++) {
        codeUnitPack[i] = iBuilder->loadInputStreamPack("codeUnitStream", iBuilder->getInt32(0), iBuilder->getInt32(i));
    }
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
            } else if (lo == 0) {
                if (hi == maxCodeVal) {
                    for (unsigned k = 0; k < packCount; k++) {
                        strmPack[k] = iBuilder->allOnes();
                    }
                } else {
                    Value * cp = ConstantInt::get(iBuilder->getIntNTy(codeUnitWidth), hi + 1);
                    Value * cp_splat = iBuilder->simd_fill(codeUnitWidth, cp);
                    for (unsigned k = 0; k < packCount; k++) {
                        strmPack[k] = iBuilder->simd_ult(codeUnitWidth, codeUnitPack[k], cp_splat);
                    }
                }
            } else if (hi == maxCodeVal) {
                Value * cp = ConstantInt::get(iBuilder->getIntNTy(codeUnitWidth), lo - 1);
                Value * cp_splat = iBuilder->simd_fill(codeUnitWidth, cp);
                for (unsigned k = 0; k < packCount; k++) {
                    strmPack[k] = iBuilder->simd_ugt(codeUnitWidth, codeUnitPack[k], cp_splat);
                }
            } else {
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
        iBuilder->storeOutputStreamBlock("ccStream", iBuilder->getInt32(j), theCCstream);
    }
}


ParabixCharacterClassKernelBuilder::ParabixCharacterClassKernelBuilder (
        const std::unique_ptr<kernel::KernelBuilder> & b, std::string ccSetName, const std::vector<CC *> & charClasses, unsigned codeUnitSize)
: PabloKernel(b, ccSetName +"_kernel",
// stream inputs
{Binding{b->getStreamSetTy(codeUnitSize), "basis"}}
// stream outputs
, {Binding(b->getStreamSetTy((unsigned int)charClasses.size()), "outputStream")}
)
, mCharClasses(charClasses) {

}

void ParabixCharacterClassKernelBuilder::generatePabloMethod() {
    PabloBuilder pb(getEntryScope());
    cc::CC_Compiler ccc(this, getInputStreamSet("basis"));
    Var * outputVar = getOutputStreamVar("outputStream");
    for (unsigned i = 0; i < mCharClasses.size(); ++i) {
        pb.createAssign(pb.createExtract(outputVar, i), ccc.compileCC("cc", mCharClasses[i], pb));
    }
}
