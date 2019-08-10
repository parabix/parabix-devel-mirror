/*
 * Copyright (c) 2019 International Characters.
 * This software is licensed to the public under the Open Software License 3.0.
 */

#include <testing/assert.h>

#include <kernel/core/kernel_builder.h>
#include <llvm/Support/raw_ostream.h>

using namespace llvm;

namespace kernel {

std::string KernelName(StreamSet * x) {
    std::string backing;
    raw_string_ostream str(backing);
    str << "StreamEquivalenceKernel::"
        << "<i" << x->getFieldWidth() << ">"
        << "[" << x->getNumElements() << "]";
    return str.str();
}

StreamEquivalenceKernel::StreamEquivalenceKernel(
    BuilderRef b,
    StreamSet * lhs,
    StreamSet * rhs,
    Scalar * outPtr)
: MultiBlockKernel(b, KernelName(lhs),
    {{"lhs", lhs, FixedRate(), Principal()}, {"rhs", rhs, FixedRate(), ZeroExtended()}},
    {},
    {{"result_ptr", outPtr}},
    {},
    {})
{
    assert(lhs->getFieldWidth() == rhs->getFieldWidth());
    assert(lhs->getNumElements() == rhs->getNumElements());
    setStride(b->getBitBlockWidth() / lhs->getFieldWidth());
    addAttribute(SideEffecting());
}

void StreamEquivalenceKernel::generateMultiBlockLogic(BuilderRef b, Value * const numOfStrides) {
    auto istreamset = getInputStreamSet("lhs");
    const uint32_t FW = istreamset->getFieldWidth();
    const uint32_t COUNT = istreamset->getNumElements();
    const uint32_t ITEMS_PER_BLOCK = b->getBitBlockWidth() / FW;

    Value * const ZERO = b->getInt32(0);
    Value * const ONE = b->getInt32(1);

    BasicBlock * const entryBlock = b->GetInsertBlock();
    BasicBlock * const loopBlock = b->CreateBasicBlock("loop");
    BasicBlock * const exitBlock = b->CreateBasicBlock("exit");
    Value * const initialOffset = b->CreateUDiv(b->getProcessedItemCount("lhs"), b->getSize(ITEMS_PER_BLOCK));
    b->CreateBr(loopBlock);

    b->SetInsertPoint(loopBlock);
    PHINode * strideNo = b->CreatePHI(b->getSizeTy(), 2);
    strideNo->addIncoming(b->getSize(0), entryBlock);
    Value * const blockOffset = b->CreateAdd(strideNo, initialOffset);
    for (uint32_t i = 0; i < COUNT; ++i) {
        Value * lhs;
        Value * rhs;
        if (FW == 1) {
            lhs = b->loadInputStreamBlock("lhs", b->getInt32(i), blockOffset);
            rhs = b->loadInputStreamBlock("rhs", b->getInt32(i), blockOffset);
        } else {
            lhs = b->loadInputStreamPack("lhs", b->getInt32(i), blockOffset);
            rhs = b->loadInputStreamPack("rhs", b->getInt32(i), blockOffset);
        }
        // b->CallPrintRegister("lhs", lhs);
        // b->CallPrintRegister("rhs", rhs);
        Value * const vecComp = b->CreateNot(b->CreateICmpEQ(lhs, rhs));
        Value * const vecAsInt = b->CreateBitCast(vecComp, b->getIntNTy(vecComp->getType()->getVectorNumElements()));
        Value * const neq = b->CreateZExt(vecAsInt, b->getInt64Ty(), "here");
        Value * const ptrVal = b->CreateLoad(b->getScalarField("result_ptr"));
        Value * const newPtrVal = b->CreateSelect(
            b->CreateICmpEQ(ptrVal, ZERO),
            b->CreateZExt(b->CreateICmpNE(neq, b->getInt64(0)), b->getInt32Ty()),
            ONE
        );
        b->CreateStore(newPtrVal, b->getScalarField("result_ptr"));
    }
    Value * const nextStrideNo = b->CreateAdd(strideNo, b->getSize(1));
    strideNo->addIncoming(nextStrideNo, loopBlock);
    b->CreateCondBr(b->CreateICmpNE(nextStrideNo, numOfStrides), loopBlock, exitBlock);

    b->SetInsertPoint(exitBlock);
}

} // namespace kernel

namespace testing {

void AssertEQ(TestEngine & T, StreamSet * lhs, StreamSet * rhs) {
    auto ptr = T->getInputScalar("__ptr_out");
    T->CreateKernelCall<kernel::StreamEquivalenceKernel>(lhs, rhs, ptr);
    T->CreateKernelCall<kernel::StreamEquivalenceKernel>(rhs, lhs, ptr);
}

} // namespace testing

