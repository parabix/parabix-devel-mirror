/*
 *  Copyright (c) 2016 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 */

#include "cc_kernel.h"
#include <re/re_cc.h>
#include <cc/cc_compiler.h>
#include <pablo/builder.hpp>
#include <llvm/IR/Module.h>

using namespace cc;
using namespace kernel;
using namespace pablo;
using namespace re;
using namespace llvm;

void DirectCharacterClassKernelBuilder::generateDoBlockMethod() {
    unsigned packCount = 8 * mCodeUnitSize;  
    unsigned codeUnitWidth = 8 * mCodeUnitSize;
    Value * codeUnitPack[packCount];
    for (unsigned i = 0; i < packCount; i++) {
        Value * ptr = getInputStream("codeUnitStream", iBuilder->getInt32(0), iBuilder->getInt32(i));
        codeUnitPack[i] = iBuilder->CreateBlockAlignedLoad(ptr);
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
        Value * ptr = getOutputStream("ccStream", iBuilder->getInt32(j));
        iBuilder->CreateBlockAlignedStore(theCCstream, ptr);
    }
}

ParabixCharacterClassKernelBuilder::ParabixCharacterClassKernelBuilder (
IDISA::IDISA_Builder * iBuilder
, std::string ccSetName
, const std::vector<CC *> & charClasses
, unsigned basisBitsCount)
: PabloKernel(iBuilder, ccSetName +"_kernel") {

    CC_Compiler ccc(this, basisBitsCount);
    auto & builder = ccc.getBuilder();
    for (CC * cc : charClasses) {
        Var * const r = addOutput(cc->canonicalName(re::ByteClass), getStreamSetTy());
        builder.createAssign(r, ccc.compileCC("cc", cc, builder));
    }

}
