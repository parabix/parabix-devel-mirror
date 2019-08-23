/*
 * Copyright (c) 2019 International Characters.
 * This software is licensed to the public under the Open Software License 3.0.
 */

#include <testing/assert.h>

#include <kernel/core/kernel_builder.h>
#include <llvm/Support/raw_ostream.h>

using namespace llvm;

namespace kernel {

std::string KernelName(StreamEquivalenceKernel::Mode mode, StreamSet * x, StreamSet * y) {
    std::string backing;
    raw_string_ostream str(backing);
    str << "StreamEquivalenceKernel::["
        << "<i" << x->getFieldWidth() << ">"
        << "[" << x->getNumElements() << "],"
        << "<i" << y->getFieldWidth() << ">"
        << "[" << y->getNumElements() << "]]"
        << "@" << (mode == StreamEquivalenceKernel::Mode::EQ ? "EQ" : "NE");
    return str.str();
}

StreamEquivalenceKernel::StreamEquivalenceKernel(
    BuilderRef b,
    Mode mode,
    StreamSet * lhs,
    StreamSet * rhs,
    Scalar * outPtr)
: MultiBlockKernel(b, KernelName(mode, lhs, rhs),
    {{"lhs", lhs, FixedRate(), Principal()}, {"rhs", rhs, FixedRate(), ZeroExtended()}},
    {},
    {{"result_ptr", outPtr}},
    {},
    {InternalScalar(b->getInt1Ty(), "accum")})
, mMode(mode)
{
    assert(lhs->getFieldWidth() == rhs->getFieldWidth());
    assert(lhs->getNumElements() == rhs->getNumElements());
    setStride(b->getBitBlockWidth() / lhs->getFieldWidth());
    addAttribute(SideEffecting());
}

void StreamEquivalenceKernel::generateInitializeMethod(BuilderRef b) {
    b->setScalarField("accum", b->getInt1(true));
}

void StreamEquivalenceKernel::generateMultiBlockLogic(BuilderRef b, Value * const numOfStrides) {
    auto istreamset = getInputStreamSet("lhs");
    const uint32_t FW = istreamset->getFieldWidth();
    const uint32_t COUNT = istreamset->getNumElements();
    const uint32_t ITEMS_PER_BLOCK = b->getBitBlockWidth() / FW;

    BasicBlock * const entryBlock = b->GetInsertBlock();
    BasicBlock * const loopBlock = b->CreateBasicBlock("loop");
    BasicBlock * const exitBlock = b->CreateBasicBlock("exit");
    Value * const hasMoreItems = b->CreateICmpNE(b->getAccessibleItemCount("lhs"), b->getSize(0));
    Value * initialOffset = nullptr;
    if (FW != 1) {
        initialOffset = b->CreateUDiv(b->getProcessedItemCount("lhs"), b->getSize(ITEMS_PER_BLOCK));
    }
    b->CallPrintInt("accessible item count: lhs", b->getAccessibleItemCount("lhs"));
    b->CallPrintInt("accessible item count: rh", b->getAccessibleItemCount("rhs"));
    b->CreateLikelyCondBr(hasMoreItems, loopBlock, exitBlock);

    b->SetInsertPoint(loopBlock);
    PHINode * strideNo = b->CreatePHI(b->getSizeTy(), 2);
    strideNo->addIncoming(b->getSize(0), entryBlock);
    Value * blockOffset = nullptr;
    if (FW != 1) {
        blockOffset = b->CreateAdd(strideNo, initialOffset);
    }
    for (uint32_t i = 0; i < COUNT; ++i) {
        Value * lhs;
        Value * rhs;
        if (FW == 1) {
            lhs = b->loadInputStreamBlock("lhs", b->getInt32(i), strideNo);
            rhs = b->loadInputStreamBlock("rhs", b->getInt32(i), strideNo);
        } else {
            lhs = b->loadInputStreamPack("lhs", b->getInt32(i), blockOffset);
            rhs = b->loadInputStreamPack("rhs", b->getInt32(i), blockOffset);
        }
        // Perform vector comparison lhs != rhs.
        // Result will be a vector of all zeros if lhs == rhs
        Value * const vComp = b->CreateICmpNE(lhs, rhs);
        Value * const vCompAsInt = b->CreateBitCast(vComp, b->getIntNTy(vComp->getType()->getVectorNumElements()));
        // `comp` will be `true` iff lhs == rhs (i.e., `vComp` is a vector of all zeros)
        Value * const comp = b->CreateICmpEQ(vCompAsInt, Constant::getNullValue(vCompAsInt->getType()));
        // `and` `comp` into `accum` so that `accum` will be `true` iff lhs == rhs for all blocks in the two streams
        Value * const accum = b->getScalarField("accum");
        Value * const newAccum = b->CreateAnd(accum, comp);
        b->setScalarField("accum", newAccum);
    }
    Value * const nextStrideNo = b->CreateAdd(strideNo, b->getSize(1));
    strideNo->addIncoming(nextStrideNo, loopBlock);
    b->CreateCondBr(b->CreateICmpNE(nextStrideNo, numOfStrides), loopBlock, exitBlock);

    b->SetInsertPoint(exitBlock);
}

void StreamEquivalenceKernel::generateFinalizeMethod(BuilderRef b) {
    // a `result` value of `true` means the assertion passed
    Value * result = b->getScalarField("accum");
    if (mMode == Mode::NE) {
        result = b->CreateNot(result);
    }

    // A `ptrVal` value of `0` means that the test is currently passing and a
    // value of `1` means the test is failing. If the test is already failing,
    // then we don't need to update the test state.
    Value * const ptrVal = b->CreateLoad(b->getScalarField("result_ptr"));
    Value * resultState;
    if (mMode == Mode::EQ) {
        resultState = b->CreateSelect(result, b->getInt32(0), b->getInt32(1));
    } else {
        // To preserve commutativity of `NE` comparisons, two additional test
        // states are needed. State `2` represents a partial passing `NE`
        // comparison and state `3` represents a partial failing `NE` comparison.
        // `AssertNE` first checks `A != B` putting the test into a parital
        // state (`2` if the comparison returns `true` or `3` if it returns `false`).
        // The second comparison `B != A` resolves the partial state. If the second
        // comparison returns `false` and the first comparison did as well (i.e.,
        // the test is in state `3`) then the test is put into a failing state.
        // Otherwise, if either of the tests returned `true` the total assertion
        // passed.
        Value * const isParitalState = b->CreateOr(b->CreateICmpEQ(ptrVal, b->getInt32(2)), b->CreateICmpEQ(ptrVal, b->getInt32(3)));
        Value * const resolveToFail = b->CreateAnd(b->CreateICmpEQ(ptrVal, b->getInt32(3)), b->CreateNot(result));
        resultState = b->CreateSelect(
            isParitalState,
            b->CreateSelect(resolveToFail, b->getInt32(1), b->getInt32(0)),
            b->CreateSelect(result, b->getInt32(2), b->getInt32(3))
        );
    }
    Value * const newVal = b->CreateSelect(b->CreateICmpEQ(ptrVal, b->getInt32(1)), b->getInt32(1), resultState);
    b->CreateStore(newVal, b->getScalarField("result_ptr"));
}

} // namespace kernel

namespace testing {

void AssertEQ(TestEngine & T, StreamSet * lhs, StreamSet * rhs) {
    using namespace kernel;
    auto ptr = T->getInputScalar("__ptr_out");
    T->CreateKernelCall<StreamEquivalenceKernel>(StreamEquivalenceKernel::Mode::EQ, lhs, rhs, ptr);
    T->CreateKernelCall<StreamEquivalenceKernel>(StreamEquivalenceKernel::Mode::EQ, rhs, lhs, ptr);
}

void AssertNE(TestEngine & T, StreamSet * lhs, StreamSet * rhs) {
    using namespace kernel;
    auto ptr = T->getInputScalar("__ptr_out");
    T->CreateKernelCall<StreamEquivalenceKernel>(StreamEquivalenceKernel::Mode::NE, lhs, rhs, ptr);
    T->CreateKernelCall<StreamEquivalenceKernel>(StreamEquivalenceKernel::Mode::NE, rhs, lhs, ptr);
}

} // namespace testing
