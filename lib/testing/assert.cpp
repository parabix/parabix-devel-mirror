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
    Scalar * carry)
: MultiBlockKernel(b, KernelName(lhs),
    {{"lhs", lhs, FixedRate(), Principal()}, {"rhs", rhs, FixedRate(), ZeroExtended()}},
    {},
    {{"carry", carry}},
    {{b->getInt32Ty(), "result"}},
    {})
{
    assert(lhs->getFieldWidth() == rhs->getFieldWidth());
    assert(lhs->getNumElements() == rhs->getNumElements());
    setStride(b->getBitBlockWidth() / lhs->getFieldWidth());
}

void StreamEquivalenceKernel::generateInitializeMethod(BuilderRef b) {
    b->setScalarField("result", b->getScalarField("carry"));
    // b->CallPrintInt("carry in", b->getScalarField("result"));
}

void StreamEquivalenceKernel::generateMultiBlockLogic(BuilderRef b, Value * const numOfStrides) {
    auto istreamset = getInputStreamSet("lhs");
    const uint32_t FW = istreamset->getFieldWidth();
    const uint32_t COUNT = istreamset->getNumElements();
   // const uint32_t ITEMS_PER_BLOCK = b->getBitBlockWidth() / FW;

    BasicBlock * const entryBlock = b->GetInsertBlock();
    BasicBlock * const loopBlock = b->CreateBasicBlock("loop");
    BasicBlock * const exitBlock = b->CreateBasicBlock("exit");
   // Value * const initialOffset = b->CreateUDiv(b->getProcessedItemCount("lhs"), b->getSize(ITEMS_PER_BLOCK));

    Value * const initialScalarVal = b->getScalarField("result");

    b->CreateBr(loopBlock);

    b->SetInsertPoint(loopBlock);
    PHINode * strideNo = b->CreatePHI(b->getSizeTy(), 2);
    strideNo->addIncoming(b->getSize(0), entryBlock);
    PHINode * const scalarValPhi = b->CreatePHI(initialScalarVal->getType(), 2);
    scalarValPhi->addIncoming(initialScalarVal, entryBlock);
    Value * scalarVal = scalarValPhi;

    //Value * const blockOffset = b->CreateAdd(strideNo, initialOffset);
    for (uint32_t i = 0; i < COUNT; ++i) {

        Constant * const I = b->getInt32(i);

        for (uint32_t j = 0; j < FW; ++j) {

            Value * lhs;
            Value * rhs;
            if (FW == 1) {
                lhs = b->loadInputStreamBlock("lhs", I, strideNo);
                rhs = b->loadInputStreamBlock("rhs", I, strideNo);
            } else {
                Constant * const J = b->getInt32(j);
                lhs = b->loadInputStreamPack("lhs", I, J, strideNo);
                rhs = b->loadInputStreamPack("rhs", I, J, strideNo);
            }

            Value * comp = b->CreateNot(b->CreateICmpEQ(lhs, rhs));
            const auto width = comp->getType()->getVectorNumElements();
            comp = b->CreateBitCast(comp, b->getIntNTy(width));
            comp = b->CreateIsNotNull(comp);
            Value * const newValue = b->CreateZExt(comp, scalarVal->getType());

            scalarVal = b->CreateOr(scalarVal, newValue);
        }
    }
    Value * const nextStrideNo = b->CreateAdd(strideNo, b->getSize(1));
    strideNo->addIncoming(nextStrideNo, loopBlock);
    scalarValPhi->addIncoming(scalarVal, loopBlock);
    b->CreateCondBr(b->CreateICmpNE(nextStrideNo, numOfStrides), loopBlock, exitBlock);

    b->SetInsertPoint(exitBlock);
    b->setScalarField("result", scalarVal);
}

} // namespace kernel

namespace testing {

Scalar * AssertEQ(TestEngine & T, StreamSet * lhs, StreamSet * rhs) {
    Scalar * carryIn = T.driver().CreateConstant(T.driver().getBuilder()->getInt32(0));
    auto k0 = T->CreateKernelCall<kernel::StreamEquivalenceKernel>(lhs, rhs, carryIn);
    Scalar * carryOut = k0->getOutputScalar("result");
    auto k1 = T->CreateKernelCall<kernel::StreamEquivalenceKernel>(rhs, lhs, carryOut);
    return k1->getOutputScalar("result");
}

} // namespace testing
