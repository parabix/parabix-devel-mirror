/*
 *  Copyright (c) 2019 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 */

#include <kernel/scan/line_span_generator.h>

#include <llvm/Support/raw_ostream.h>
#include <kernel/core/kernel_builder.h>

using namespace llvm;

namespace kernel {

LineSpanGenerator::LineSpanGenerator(BuilderRef b, StreamSet * linebreakStream, StreamSet * output)
: SingleStreamScanKernelTemplate(b, "LineSpanGenerator", linebreakStream)
{
    assert (linebreakStream->getNumElements() == 1 && linebreakStream->getFieldWidth() == 1);
    addInternalScalar(b->getInt64Ty(), "lineBegin");
    mOutputStreamSets.push_back({"output", output, PopcountOf("scan")});
}

void LineSpanGenerator::generateProcessingLogic(BuilderRef b, Value * const absoluteIndex) {
    Value * const producedCount = b->getProducedItemCount("output");
    Value * const beginIdx = b->getScalarField("lineBegin");
    Value * const endIdx = b->CreateZExtOrBitCast(absoluteIndex, b->getInt64Ty());
    Value * const beginStorePtr = b->getRawOutputPointer("output", b->getInt32(0), producedCount);
    Value * const endStorePtr = b->getRawOutputPointer("output", b->getInt32(1), producedCount);
    b->CreateStore(beginIdx, beginStorePtr);
    b->CreateStore(endIdx, endStorePtr);
    b->setScalarField("lineBegin", b->CreateAdd(endIdx, b->getInt64(1)));
    b->setProducedItemCount("output", b->CreateAdd(producedCount, b->getSize(1)));
}

LineSpanFilterKernel::LineSpanFilterKernel(BuilderRef b, StreamSet * lineNumbers, StreamSet * spans, StreamSet * output)
: MultiBlockKernel(b, "LineSpanFilter",
    // Rate of consumption of `lineNumbers` and, in turn, production of `output`
    // cannot be determined statically as it relates to the actual values of the
    // items in the `lineNumber` stream.
    //
    // The rate of consumption of `spans` is also unknown as it relates to the
    // item values of `lineNumbers`.
    {{"lineNumbers", lineNumbers, BoundedRate(0, 1)}, {"spans", spans, BoundedRate(0, 1)}},
    {{"output", output, BoundedRate(0, 1)}},
    {}, {}, {})
{
    assert(lineNumbers->getFieldWidth() == 64 && lineNumbers->getNumElements() == 1);
    assert(spans->getFieldWidth() == 64 && spans->getNumElements() == 2);
    assert(output->getFieldWidth() == 64 && output->getNumElements() == 2);
    setStride(1);
}

void LineSpanFilterKernel::generateMultiBlockLogic(BuilderRef b, Value * const numOfStrides) {
    Type * const i64 = b->getInt64Ty();
    Value * const i64_ZERO = b->getInt64(0);
    Value * const i64_ONE = b->getInt64(1);

    BasicBlock * const block_Entry = b->GetInsertBlock();
    BasicBlock * const block_PreProcess = b->CreateBasicBlock("preprocess");
    BasicBlock * const block_UseSpan = b->CreateBasicBlock("useSpan");
    BasicBlock * const block_SkipSpan = b->CreateBasicBlock("skipSpan");
    BasicBlock * const block_Exit = b->CreateBasicBlock("exit");

    Value * const ic_LineNumbersProcessedCount = b->getProcessedItemCount("lineNumbers");
    Value * const ic_SpansProcessedCount = b->getProcessedItemCount("spans");
    Value * const ic_AvailableLineNumbers = b->getAvailableItemCount("lineNumbers");
    Value * const ic_AvailableSpans = b->getAvailableItemCount("spans");
    Value * const noAvailableLineNumbers = b->CreateICmpEQ(ic_AvailableLineNumbers, i64_ZERO);
    Value * const noAvailableSpans = b->CreateICmpEQ(ic_AvailableSpans, i64_ZERO);
    Value * const noAvailableStreams = b->CreateOr(noAvailableLineNumbers, noAvailableSpans);
    Value * const shouldExit = b->CreateOr(noAvailableStreams, mIsFinal);
    b->CreateUnlikelyCondBr(shouldExit, block_Exit, block_PreProcess);

    auto GenerateContinueCheck = [&](Value * lnIdx, Value * spanIdx) -> Value * {
        Value * const areMoreLineNumbers = b->CreateICmpULT(lnIdx, ic_AvailableLineNumbers);
        Value * const areMoreSpans = b->CreateICmpULT(spanIdx, ic_AvailableSpans);
        return b->CreateAnd(areMoreLineNumbers, areMoreSpans);
    };

    b->SetInsertPoint(block_PreProcess);
    PHINode * const spanIndex = b->CreatePHI(i64, 3);
    spanIndex->addIncoming(ic_SpansProcessedCount, block_Entry);
    PHINode * const lineNumberIndex = b->CreatePHI(i64, 3);
    lineNumberIndex->addIncoming(ic_LineNumbersProcessedCount, block_Entry);
    Value * const lineNumVal = b->CreateLoad(b->getRawInputPointer("lineNumbers", lineNumberIndex));
    if (codegen::DebugOptionIsSet(codegen::EnableAsserts)) {
        b->CreateAssert(b->CreateICmpUGE(lineNumVal, spanIndex), "spans have overtaken line numbers");
    }
    b->CreateCondBr(b->CreateICmpEQ(lineNumVal, spanIndex), block_UseSpan, block_SkipSpan);

    b->SetInsertPoint(block_UseSpan);
    Value * const startVal = b->CreateLoad(b->getRawInputPointer("spans", i64_ZERO, spanIndex));
    Value * const endVal = b->CreateLoad(b->getRawInputPointer("spans", i64_ONE, spanIndex));
    b->CreateStore(startVal, b->getRawOutputPointer("output", i64_ZERO, lineNumberIndex));
    b->CreateStore(endVal, b->getRawOutputPointer("output", i64_ONE, lineNumberIndex));
    b->setProducedItemCount("output", b->CreateAdd(lineNumberIndex, i64_ONE));
    Value * const nextLineNumberIndex = b->CreateAdd(lineNumberIndex, i64_ONE);
    b->setProcessedItemCount("lineNumbers", nextLineNumberIndex);
    lineNumberIndex->addIncoming(nextLineNumberIndex, block_UseSpan);
    spanIndex->addIncoming(spanIndex, block_UseSpan);
    b->CreateCondBr(GenerateContinueCheck(nextLineNumberIndex, spanIndex), block_PreProcess, block_Exit);

    b->SetInsertPoint(block_SkipSpan);
    Value * const nextSpanIndex = b->CreateAdd(spanIndex, i64_ONE);
    b->setProcessedItemCount("spans", nextSpanIndex);
    lineNumberIndex->addIncoming(lineNumberIndex, block_SkipSpan);
    spanIndex->addIncoming(nextSpanIndex, block_SkipSpan);
    b->CreateCondBr(GenerateContinueCheck(lineNumberIndex, nextSpanIndex), block_PreProcess, block_Exit);

    b->SetInsertPoint(block_Exit);
}

} // namespace kernel
