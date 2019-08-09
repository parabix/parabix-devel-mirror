/*
 *  Copyright (c) 2019 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 */

#include <kernel/util/debug_display.h>

#include <kernel/core/kernel_builder.h>

using namespace llvm;

namespace kernel {

static std::string GenerateName(StringRef name, StreamSet * s) {
    return std::string("DebugDisplay::") + 
           "<i" + std::to_string(s->getFieldWidth()) + ">[" + std::to_string(s->getNumElements()) + "]" +
           "@" + name.str();
}

DebugDisplayKernel::DebugDisplayKernel(BuilderRef b, StringRef name, StreamSet * s)
: MultiBlockKernel(b, GenerateName(name, s), {{"s", s}}, {}, {}, {}, {InternalScalar{b->getSizeTy(), "initialStride"}})
, mName(name)
, mFW(s->getFieldWidth())
, mSCount(s->getNumElements())
{
    if (mFW != 1) {
        setStride(1);
    } else {
        setStride(b->getBitBlockWidth());
    }
    addAttribute(SideEffecting());
}

void DebugDisplayKernel::generateMultiBlockLogic(BuilderRef b, Value * const numOfStrides) {
    auto getRegName = [&](uint32_t i) -> std::string {
        if (mSCount != 1) {
            return std::string(mName) + "[" + std::to_string(i) + "]";
        } else {
            return mName;
        }
    };

    bool useBitblocks = mFW == 1;

    Type * const sizeTy = b->getSizeTy();
    Value * const sz_ZERO = b->getSize(0);
    Value * const sz_ONE = b->getSize(1);

    BasicBlock * const entry = b->GetInsertBlock();
    BasicBlock * const loop = b->CreateBasicBlock("loop");
    BasicBlock * const exit = b->CreateBasicBlock("exit");

    Value * initialStride = nullptr;
    if (!useBitblocks) {
        initialStride = b->getScalarField("initialStride");
    }
    
    if (!useBitblocks) {
        // Since stride width == 1, an extra final call to this kernel is made.
        // We don't want to print anything out on this final call.
        b->CreateCondBr(isFinal(), exit, loop);
    } else {
        b->CreateBr(loop);
    }

    b->SetInsertPoint(loop);
    PHINode * const strideNum = b->CreatePHI(sizeTy, 2);
    strideNum->addIncoming(sz_ZERO, entry);
    if (useBitblocks) {
        // strideNum is equivalent to the block number since stride width == bitblock width
        for (uint32_t i = 0; i < mSCount; ++i) {
            Value * const block = b->loadInputStreamBlock("s", b->getInt32(i), strideNum);
            b->CallPrintRegister(getRegName(i), block);
        }
    } else {
        for (uint32_t i = 0; i < mSCount; ++i) {
            Value * const ptr = b->getRawInputPointer("s", b->getInt32(i), b->CreateAdd(strideNum, initialStride));
            Value * const val = b->CreateLoad(ptr);
            b->CallPrintInt(getRegName(i), val);
        }
    }
    Value * const nextStrideNum = b->CreateAdd(strideNum, sz_ONE);
    strideNum->addIncoming(nextStrideNum, loop);
    if (!useBitblocks) {
        b->setScalarField("initialStride", b->CreateAdd(nextStrideNum, initialStride));
    }
    b->CreateCondBr(b->CreateICmpNE(nextStrideNum, numOfStrides), loop, exit);

    b->SetInsertPoint(exit);
}

}
