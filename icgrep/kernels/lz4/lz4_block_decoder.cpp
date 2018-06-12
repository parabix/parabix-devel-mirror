

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

LZ4BlockDecoderNewKernel::LZ4BlockDecoderNewKernel(const std::unique_ptr<kernel::KernelBuilder> &iBuilder, std::string&& kernelName)
: SegmentOrientedKernel(std::string(kernelName),
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

Binding{iBuilder->getInt8Ty(), "pendingIsCompressed"},
Binding{iBuilder->getInt64Ty(), "pendingBlockStart"},
Binding{iBuilder->getInt64Ty(), "pendingBlockEnd"},
}) {

}

void LZ4BlockDecoderNewKernel::generateDoSegmentMethod(const std::unique_ptr<KernelBuilder> & b) {

    Constant* INT64_0 = b->getInt64(0);

    BasicBlock * entryBlock = b->GetInsertBlock();

    // Skip Header
    Value* hasSkipHeader = b->getScalarField("hasSkipHeader");
    b->setScalarField("hasSkipHeader", b->getTrue());
    Value* skipLength = b->CreateSelect(hasSkipHeader, b->getSize(0), b->getScalarField("headerSize"));
    Value* previousOffset = b->getScalarField("previousOffset");
    previousOffset = b->CreateAdd(skipLength, previousOffset);
    Value* initBlockStart = b->getScalarField("pendingBlockStart");
    Value* initBlockEnd = b->getScalarField("pendingBlockEnd");
    Value* initIsCompressed = b->getScalarField("pendingIsCompressed");
    Value * availableItemCount = b->getAvailableItemCount("byteStream");
    BasicBlock * processCon = b->CreateBasicBlock("process_con");
    b->CreateBr(processCon);

    b->SetInsertPoint(processCon);

    PHINode* phiIsCompressed = b->CreatePHI(initIsCompressed->getType(), 3);
    PHINode* phiBlockStart = b->CreatePHI(initBlockStart->getType(), 3);
    PHINode* phiBlockEnd = b->CreatePHI(initBlockEnd->getType(), 3);
    PHINode* sOffset = b->CreatePHI(previousOffset->getType(), 3);

    phiIsCompressed->addIncoming(initIsCompressed, entryBlock);
    phiBlockStart->addIncoming(initBlockStart, entryBlock);
    phiBlockEnd->addIncoming(initBlockEnd, entryBlock);
    sOffset->addIncoming(previousOffset, entryBlock);

    // Store Output
    BasicBlock* storeOutputBlock = b->CreateBasicBlock("storeOutputBlock");
    BasicBlock * block_decoder_con = b->CreateBasicBlock("block_decoder_con_block");

    b->CreateUnlikelyCondBr(
            b->CreateAnd(
                    b->CreateICmpULE(phiBlockEnd, availableItemCount),
                    b->CreateNot(b->CreateICmpEQ(phiBlockEnd, INT64_0))
            ),
            storeOutputBlock,
            block_decoder_con
    );

    b->SetInsertPoint(storeOutputBlock);

    appendOutput(b, phiIsCompressed, phiBlockStart, phiBlockEnd);


    phiIsCompressed->addIncoming(b->getInt8(0), storeOutputBlock);
    phiBlockStart->addIncoming(INT64_0, storeOutputBlock);
    phiBlockEnd->addIncoming(INT64_0, storeOutputBlock);
    sOffset->addIncoming(sOffset, storeOutputBlock);

    b->CreateBr(processCon);


    // block decoder entry
    b->SetInsertPoint(block_decoder_con);

    BasicBlock * block_decoder_body = b->CreateBasicBlock("block_decoder_body_block");
    BasicBlock * block_decoder_exit = b->CreateBasicBlock("block_decoder_exit_block");

    Value * reachFinalBlock = b->getScalarField("reachFinalBlock");
    b->CreateCondBr(
        b->CreateAnd(
            b->CreateICmpULT(sOffset, availableItemCount),
            b->CreateNot(reachFinalBlock)
        ),
        block_decoder_body,
        block_decoder_exit);

    //block_decoder_body
    b->SetInsertPoint(block_decoder_body);
    Value* currentBlockSize = b->getSize(0);
    for (size_t i = 0; i < 4; i++) {
        Value * offset = b->CreateAdd(sOffset, b->getSize(i));
        Value * rawOffset = b->CreateZExt(generateLoadInput(b, offset), b->getSizeTy());
        currentBlockSize = b->CreateOr(currentBlockSize, b->CreateShl(rawOffset, b->getSize(8 * i)));
    }

    Value * realBlockSize = b->CreateAnd(currentBlockSize, 0x7fffffff);

    Value * isCompressed = b->CreateNot(currentBlockSize);
    isCompressed = b->CreateLShr(isCompressed, 31);
    isCompressed = b->CreateTrunc(isCompressed, b->getInt1Ty());

    Value * isFinalBlock = b->CreateICmpEQ(realBlockSize, b->getSize(0));
    b->setScalarField("reachFinalBlock", isFinalBlock);

    Value * blockStart = b->CreateAdd(sOffset, b->getSize(4));
    Value * blockEnd = b->CreateAdd(blockStart, realBlockSize);

    Value * newOffset = sOffset;
    newOffset = b->CreateAdd(newOffset, b->getSize(4)); // Block Size
    newOffset = b->CreateAdd(newOffset, realBlockSize); // Block Content
    Value * const blockChecksumOffset = b->CreateSelect(b->getScalarField("hasBlockChecksum"), b->getSize(4), b->getSize(0));
    newOffset = b->CreateAdd(newOffset, blockChecksumOffset);

    sOffset->addIncoming(newOffset, block_decoder_body);
    phiIsCompressed->addIncoming(b->CreateZExt(isCompressed, b->getInt8Ty()), block_decoder_body);
    phiBlockStart->addIncoming(blockStart, block_decoder_body);
    phiBlockEnd->addIncoming(blockEnd, block_decoder_body);
    b->CreateBr(processCon);

    // block_decoder_exit_block
    b->SetInsertPoint(block_decoder_exit);
    b->setScalarField("pendingIsCompressed", phiIsCompressed);
    b->setScalarField("pendingBlockStart", phiBlockStart);
    b->setScalarField("pendingBlockEnd", phiBlockEnd);
    b->setScalarField("previousOffset", sOffset);
    b->setProcessedItemCount("byteStream", availableItemCount);
    b->setTerminationSignal(mIsFinal);
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
