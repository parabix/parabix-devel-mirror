//
// Created by wxy325 on 2018/3/16.
//

#include "lz4_block_decoder.h"

#include <kernels/kernel_builder.h>
#include <iostream>
#include <string>
#include <llvm/Support/raw_ostream.h>
#include <kernels/streamset.h>

using namespace llvm;
using namespace kernel;
using namespace std;

namespace kernel{

LZ4BlockDecoderNewKernel::LZ4BlockDecoderNewKernel(const std::unique_ptr<kernel::KernelBuilder> &iBuilder)
: SegmentOrientedKernel("LZ4BlockDecoderNewKernel",
// Inputs
{
    Binding{iBuilder->getStreamSetTy(1, 8), "byteStream"},
},
//Outputs
{
    Binding{iBuilder->getStreamSetTy(1, 8), "isCompressed", BoundedRate(0, 1)},
    Binding{iBuilder->getStreamSetTy(1, 64), "blockStart", RateEqualTo("isCompressed")},
    Binding{iBuilder->getStreamSetTy(1, 64), "blockEnd", RateEqualTo("isCompressed")}},
//Arguments
{
    Binding{iBuilder->getInt1Ty(), "hasBlockChecksum"},
    Binding{iBuilder->getSizeTy(), "headerSize"},
    Binding{iBuilder->getSizeTy(), "fileSize"}
},
{},
//Internal states:
{
Binding{iBuilder->getInt1Ty(), "hasSkipHeader"},
Binding{iBuilder->getSizeTy(), "previousOffset"},
Binding{iBuilder->getInt1Ty(), "reachFinalBlock"},

Binding{iBuilder->getInt1Ty(), "pendingIsCompressed"},
Binding{iBuilder->getInt64Ty(), "pendingBlockStart"},
Binding{iBuilder->getInt64Ty(), "pendingBlockEnd"},
}) {

}

void LZ4BlockDecoderNewKernel::generateDoSegmentMethod(const std::unique_ptr<KernelBuilder> & iBuilder) {

    Constant* INT64_0 = iBuilder->getInt64(0);

    BasicBlock * entryBlock = iBuilder->GetInsertBlock();

    // Skip Header
    Value* hasSkipHeader = iBuilder->getScalarField("hasSkipHeader");
    iBuilder->setScalarField("hasSkipHeader", iBuilder->getTrue());
    Value* skipLength = iBuilder->CreateSelect(hasSkipHeader, iBuilder->getSize(0), iBuilder->getScalarField("headerSize"));
    Value* previousOffset = iBuilder->getScalarField("previousOffset");
    previousOffset = iBuilder->CreateAdd(skipLength, previousOffset);
    Value* initBlockStart = iBuilder->getScalarField("pendingBlockStart");
    Value* initBlockEnd = iBuilder->getScalarField("pendingBlockEnd");
    Value* initIsCompressed = iBuilder->getScalarField("pendingIsCompressed");
    Value * availableItemCount = iBuilder->getAvailableItemCount("byteStream");
    BasicBlock * processCon = iBuilder->CreateBasicBlock("process_con");
    iBuilder->CreateBr(processCon);

    iBuilder->SetInsertPoint(processCon);

    PHINode* phiIsCompressed = iBuilder->CreatePHI(initIsCompressed->getType(), 3);
    PHINode* phiBlockStart = iBuilder->CreatePHI(initBlockStart->getType(), 3);
    PHINode* phiBlockEnd = iBuilder->CreatePHI(initBlockEnd->getType(), 3);
    PHINode* sOffset = iBuilder->CreatePHI(previousOffset->getType(), 3);

    phiIsCompressed->addIncoming(initIsCompressed, entryBlock);
    phiBlockStart->addIncoming(initBlockStart, entryBlock);
    phiBlockEnd->addIncoming(initBlockEnd, entryBlock);
    sOffset->addIncoming(previousOffset, entryBlock);

    // Store Output
    BasicBlock* storeOutputBlock = iBuilder->CreateBasicBlock("storeOutputBlock");
    BasicBlock * block_decoder_con = iBuilder->CreateBasicBlock("block_decoder_con_block");

    iBuilder->CreateUnlikelyCondBr(
            iBuilder->CreateAnd(
                    iBuilder->CreateICmpULE(phiBlockEnd, availableItemCount),
                    iBuilder->CreateNot(iBuilder->CreateICmpEQ(phiBlockEnd, INT64_0))
            ),
            storeOutputBlock,
            block_decoder_con
    );

    iBuilder->SetInsertPoint(storeOutputBlock);

    appendOutput(iBuilder, phiIsCompressed, phiBlockStart, phiBlockEnd);

    phiIsCompressed->addIncoming(iBuilder->getFalse(), storeOutputBlock);
    phiBlockStart->addIncoming(INT64_0, storeOutputBlock);
    phiBlockEnd->addIncoming(INT64_0, storeOutputBlock);
    sOffset->addIncoming(sOffset, storeOutputBlock);

    iBuilder->CreateBr(processCon);


    // block decoder entry
    iBuilder->SetInsertPoint(block_decoder_con);

    BasicBlock * block_decoder_body = iBuilder->CreateBasicBlock("block_decoder_body_block");
    BasicBlock * block_decoder_exit = iBuilder->CreateBasicBlock("block_decoder_exit_block");

    Value * reachFinalBlock = iBuilder->getScalarField("reachFinalBlock");
    iBuilder->CreateCondBr(
        iBuilder->CreateAnd(
            iBuilder->CreateICmpULT(sOffset, availableItemCount),
            iBuilder->CreateNot(reachFinalBlock)
        ),
        block_decoder_body,
        block_decoder_exit);

    //block_decoder_body
    iBuilder->SetInsertPoint(block_decoder_body);
    Value* currentBlockSize = iBuilder->getSize(0);
    for (size_t i = 0; i < 4; i++) {
        Value * offset = iBuilder->CreateAdd(sOffset, iBuilder->getSize(i));
        Value * rawOffset = iBuilder->CreateZExt(generateLoadInput(iBuilder, offset), iBuilder->getSizeTy());
        currentBlockSize = iBuilder->CreateOr(currentBlockSize, iBuilder->CreateShl(rawOffset, iBuilder->getSize(8 * i)));
    }

    Value * realBlockSize = iBuilder->CreateAnd(currentBlockSize, 0x7fffffff);

    Value * isCompressed = iBuilder->CreateNot(currentBlockSize);
    isCompressed = iBuilder->CreateLShr(isCompressed, 31);
    isCompressed = iBuilder->CreateTrunc(isCompressed, iBuilder->getInt1Ty());

    Value * isFinalBlock = iBuilder->CreateICmpEQ(realBlockSize, iBuilder->getSize(0));
    iBuilder->setScalarField("reachFinalBlock", isFinalBlock);

    Value * blockStart = iBuilder->CreateAdd(sOffset, iBuilder->getSize(4));
    Value * blockEnd = iBuilder->CreateAdd(blockStart, realBlockSize);

    Value * newOffset = sOffset;
    newOffset = iBuilder->CreateAdd(newOffset, iBuilder->getSize(4)); // Block Size
    newOffset = iBuilder->CreateAdd(newOffset, realBlockSize); // Block Content
    Value * const blockChecksumOffset = iBuilder->CreateSelect(iBuilder->getScalarField("hasBlockChecksum"), iBuilder->getSize(4), iBuilder->getSize(0));
    newOffset = iBuilder->CreateAdd(newOffset, blockChecksumOffset);

    sOffset->addIncoming(newOffset, block_decoder_body);
    phiIsCompressed->addIncoming(isCompressed, block_decoder_body);
    phiBlockStart->addIncoming(blockStart, block_decoder_body);
    phiBlockEnd->addIncoming(blockEnd, block_decoder_body);
    iBuilder->CreateBr(processCon);

    // block_decoder_exit_block
    iBuilder->SetInsertPoint(block_decoder_exit);
    iBuilder->setScalarField("pendingIsCompressed", phiIsCompressed);
    iBuilder->setScalarField("pendingBlockStart", phiBlockStart);
    iBuilder->setScalarField("pendingBlockEnd", phiBlockEnd);
    iBuilder->setScalarField("previousOffset", sOffset);
    iBuilder->setProcessedItemCount("byteStream", availableItemCount);
    iBuilder->setTerminationSignal(mIsFinal);
}

void LZ4BlockDecoderNewKernel::appendOutput(const std::unique_ptr<KernelBuilder> & iBuilder, Value * const isCompressed, Value * const blockStart, Value * const blockEnd) {
    Value * const offset = iBuilder->getProducedItemCount("isCompressed");
    generateStoreNumberOutput(iBuilder, "isCompressed", offset, iBuilder->CreateZExt(isCompressed, iBuilder->getInt8Ty()));
    generateStoreNumberOutput(iBuilder, "blockStart", offset, blockStart);
    generateStoreNumberOutput(iBuilder, "blockEnd", offset, blockEnd);
    iBuilder->setProducedItemCount("isCompressed", iBuilder->CreateAdd(offset, iBuilder->getSize(1)));
}

Value* LZ4BlockDecoderNewKernel::generateLoadInput(const std::unique_ptr<KernelBuilder> & iBuilder, llvm::Value* offset) {
    return iBuilder->CreateLoad(iBuilder->getRawInputPointer("byteStream", offset));
}

void LZ4BlockDecoderNewKernel::generateStoreNumberOutput(const unique_ptr<KernelBuilder> &iBuilder, const string &outputBufferName, Value * offset, Value *value) {
    iBuilder->CreateStore(value, iBuilder->getRawOutputPointer(outputBufferName, offset));
}

}
