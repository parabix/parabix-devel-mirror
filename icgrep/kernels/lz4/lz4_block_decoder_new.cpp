//
// Created by wxy325 on 2018/3/16.
//

#include "lz4_block_decoder_new.h"

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
: MultiBlockKernel("LZ4BlockDecoderNewKernel",
    // Inputs
    {
                           Binding{iBuilder->getStreamSetTy(1, 8), "byteStream", FixedRate(1), AlwaysConsume()},
                           Binding{iBuilder->getStreamSetTy(1, 1), "extender", FixedRate(1), AlwaysConsume()}
                   },
    //Outputs
    {
        Binding{iBuilder->getStreamSetTy(1, 8), "isCompressed", BoundedRate(0, 1)},
        Binding{iBuilder->getStreamSetTy(1, 64), "blockStart", BoundedRate(0, 1)},
        Binding{iBuilder->getStreamSetTy(1, 64), "blockEnd", BoundedRate(0, 1)}},
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
        addAttribute(MustExplicitlyTerminate());
}

void LZ4BlockDecoderNewKernel::resetPreviousProducedMap(const std::unique_ptr<KernelBuilder> &iBuilder,
                                                        std::vector<std::string> outputList) {
    previousProducedMap.clear();
    for (auto iter = outputList.begin(); iter != outputList.end(); ++iter) {
        previousProducedMap.insert(std::make_pair(*iter, iBuilder->getProducedItemCount(*iter)));
    }
}

void LZ4BlockDecoderNewKernel::generateMultiBlockLogic(const std::unique_ptr<KernelBuilder> &iBuilder, Value * const numOfStrides) {
    // Constant
    Constant* INT8_0 = iBuilder->getInt8(0);
    Constant* INT8_1 = iBuilder->getInt8(1);
    Constant* INT64_0 = iBuilder->getInt64(0);


    BasicBlock * entryBlock = iBuilder->GetInsertBlock();
    BasicBlock * exitBlock = iBuilder->CreateBasicBlock("exit");

    this->resetPreviousProducedMap(iBuilder, {"isCompressed", "blockStart", "blockEnd"});

    // Skip Header
    Value* hasSkipHeader = iBuilder->getScalarField("hasSkipHeader");
    iBuilder->setScalarField("hasSkipHeader", iBuilder->getInt1(true));
    Value* skipLength = iBuilder->CreateSelect(hasSkipHeader, iBuilder->getSize(0), iBuilder->getScalarField("headerSize"));
    Value* previousOffset = iBuilder->getScalarField("previousOffset");
    previousOffset = iBuilder->CreateAdd(skipLength, previousOffset);
    Value* initBlockStart = iBuilder->getScalarField("pendingBlockStart");
    Value* initBlockEnd = iBuilder->getScalarField("pendingBlockEnd");
    Value* initIsCompressed = iBuilder->getScalarField("pendingIsCompressed");


    Value* availableItemCount = iBuilder->getAvailableItemCount("byteStream");
    Value* processedItemCount = iBuilder->getProcessedItemCount("byteStream");

    Value* totalItemCount = iBuilder->CreateAdd(availableItemCount, processedItemCount);
    Value* mIsFinalBlock = iBuilder->CreateICmpEQ(totalItemCount, iBuilder->getScalarField("fileSize"));
    iBuilder->setTerminationSignal(mIsFinalBlock);

    Value* totalItemCount2 = iBuilder->CreateAdd(iBuilder->getAvailableItemCount("extender"), iBuilder->getProcessedItemCount("extender"));

//    iBuilder->CallPrintInt("===totalItemCount2", totalItemCount2);

    BasicBlock* processCon = iBuilder->CreateBasicBlock("process_con");
    iBuilder->CreateBr(processCon);

    iBuilder->SetInsertPoint(processCon);

    PHINode* phiIsCompressed = iBuilder->CreatePHI(iBuilder->getInt8Ty(), 3);
    PHINode* phiBlockStart = iBuilder->CreatePHI(iBuilder->getInt64Ty(), 3);
    PHINode* phiBlockEnd = iBuilder->CreatePHI(iBuilder->getInt64Ty(), 3);
    PHINode* sOffset = iBuilder->CreatePHI(iBuilder->getSizeTy(), 3);

    phiIsCompressed->addIncoming(initIsCompressed, entryBlock);
    phiBlockStart->addIncoming(initBlockStart, entryBlock);
    phiBlockEnd->addIncoming(initBlockEnd, entryBlock);
    sOffset->addIncoming(previousOffset, entryBlock);

    // Store Output
    BasicBlock* storeOutputBlock = iBuilder->CreateBasicBlock("storeOutputBlock");
    BasicBlock * block_decoder_con = iBuilder->CreateBasicBlock("block_decoder_con_block");

    iBuilder->CreateUnlikelyCondBr(
            iBuilder->CreateAnd(
                    iBuilder->CreateAnd(iBuilder->CreateICmpULE(phiBlockEnd, totalItemCount), iBuilder->CreateICmpULE(phiBlockEnd, totalItemCount2)),
                    iBuilder->CreateNot(iBuilder->CreateICmpEQ(phiBlockEnd, INT64_0))
            ),
            storeOutputBlock,
            block_decoder_con
    );

    iBuilder->SetInsertPoint(storeOutputBlock);
    this->appendOutput(iBuilder, phiIsCompressed, phiBlockStart, phiBlockEnd);
    phiIsCompressed->addIncoming(INT8_0, storeOutputBlock);
    phiBlockStart->addIncoming(INT64_0, storeOutputBlock);
    phiBlockEnd->addIncoming(INT64_0, storeOutputBlock);
    sOffset->addIncoming(sOffset, storeOutputBlock);
    iBuilder->CreateBr(processCon);


    // block decoder entry
    iBuilder->SetInsertPoint(block_decoder_con);

    BasicBlock * block_decoder_body = iBuilder->CreateBasicBlock("block_decoder_body_block");
    BasicBlock * block_decoder_exit = iBuilder->CreateBasicBlock("block_decoder_exit_block");

    Value* reachFinalBlock = iBuilder->getScalarField("reachFinalBlock");

    iBuilder->CreateCondBr(
        iBuilder->CreateAnd(
            iBuilder->CreateICmpULT(sOffset, totalItemCount),
            iBuilder->CreateNot(reachFinalBlock)
        ),
        block_decoder_body,
        block_decoder_exit);

    //block_decoder_body
    iBuilder->SetInsertPoint(block_decoder_body);
    Value* currentBlockSize = iBuilder->getSize(0);
    for (size_t i = 0; i < 4; i++) {
        Value* offset = iBuilder->CreateAdd(sOffset, iBuilder->getSize(i));
        Value* rawOffset = iBuilder->CreateZExt(this->generateLoadInput(iBuilder, offset), iBuilder->getSizeTy());

        currentBlockSize = iBuilder->CreateAdd(currentBlockSize, iBuilder->CreateShl(rawOffset, iBuilder->getSize(8 * i)));
    }

    Value* realBlockSize = iBuilder->CreateAnd(currentBlockSize, 0x7fffffff);
    Value* highestBit = iBuilder->CreateTrunc(iBuilder->CreateLShr(currentBlockSize, 31), iBuilder->getInt1Ty());
    Value* isCompressed = iBuilder->CreateNot(highestBit);

    Value* isFinalBlock = iBuilder->CreateICmpEQ(realBlockSize, iBuilder->getSize(0));
    iBuilder->setScalarField("reachFinalBlock", isFinalBlock);

    Value* blockStart = iBuilder->CreateAdd(sOffset, iBuilder->getSize(4));
    Value* blockEnd = iBuilder->CreateAdd(blockStart, realBlockSize);

    Value* newOffset = sOffset;
    newOffset = iBuilder->CreateAdd(newOffset, iBuilder->getSize(4)); // Block Size
    newOffset = iBuilder->CreateAdd(newOffset, realBlockSize); // Block Content
    newOffset = iBuilder->CreateAdd(
            newOffset,
            iBuilder->CreateSelect(
                    iBuilder->getScalarField("hasBlockChecksum"),
                    iBuilder->getSize(4),
                    iBuilder->getSize(0))
    ); // Block Checksum

    sOffset->addIncoming(newOffset, iBuilder->GetInsertBlock());
    phiIsCompressed->addIncoming(iBuilder->CreateSelect(isCompressed, INT8_1, INT8_0), iBuilder->GetInsertBlock());
    phiBlockStart->addIncoming(blockStart, iBuilder->GetInsertBlock());
    phiBlockEnd->addIncoming(blockEnd, iBuilder->GetInsertBlock());
    iBuilder->CreateBr(processCon);

    // block_decoder_exit_block
    iBuilder->SetInsertPoint(block_decoder_exit);

    iBuilder->setScalarField("pendingIsCompressed", phiIsCompressed);
    iBuilder->setScalarField("pendingBlockStart", phiBlockStart);
    iBuilder->setScalarField("pendingBlockEnd", phiBlockEnd);
    iBuilder->setScalarField("previousOffset", sOffset);

    iBuilder->CreateBr(exitBlock);
    iBuilder->SetInsertPoint(exitBlock);
}


    Value* LZ4BlockDecoderNewKernel::generateLoadInput(const std::unique_ptr<KernelBuilder> & iBuilder, llvm::Value* offset) {
        // The external buffer is always linear accessible, so the GEP here is safe
        Value * inputBufferBasePtr = iBuilder->getRawInputPointer("byteStream", iBuilder->getSize(0));
        Value* targetPtr = iBuilder->CreateGEP(inputBufferBasePtr, offset);
        return iBuilder->CreateLoad(targetPtr);
    }

    void LZ4BlockDecoderNewKernel::appendOutput(const std::unique_ptr<KernelBuilder> & iBuilder, Value* isCompressed, Value* blockStart, Value* blockEnd) {
        // Constant
        this->generateStoreNumberOutput(iBuilder, "isCompressed", iBuilder->getInt8Ty()->getPointerTo(), isCompressed);
        this->generateStoreNumberOutput(iBuilder, "blockStart", iBuilder->getInt64Ty()->getPointerTo(), blockStart);
        this->generateStoreNumberOutput(iBuilder, "blockEnd", iBuilder->getInt64Ty()->getPointerTo(), blockEnd);
    }

    void LZ4BlockDecoderNewKernel::generateStoreNumberOutput(const unique_ptr<KernelBuilder> &iBuilder,
                                                             const string &outputBufferName, Type *pointerType,
                                                             Value *value) {
        Value* SIZE_BIT_BLOCK_WIDTH = iBuilder->getSize(iBuilder->getBitBlockWidth());
        Value* SIZE_ZERO = iBuilder->getSize(0);
        Value* SIZE_ONE = iBuilder->getSize(1);

        Value* previousProduced = previousProducedMap.find(outputBufferName)->second;

        Value* blockIndexBase = iBuilder->CreateUDiv(previousProduced, SIZE_BIT_BLOCK_WIDTH);
        Value* outputOffset = iBuilder->getProducedItemCount(outputBufferName);
        Value* blockIndex = iBuilder->CreateUDiv(outputOffset, SIZE_BIT_BLOCK_WIDTH);

        Value* blockOffset = iBuilder->CreateURem(outputOffset, SIZE_BIT_BLOCK_WIDTH);

        // i8, [8 x <4 x i64>]*
        // i64, [64 x <4 x i64>]*
        Value* ptr = iBuilder->getOutputStreamBlockPtr(outputBufferName, SIZE_ZERO, iBuilder->CreateSub(blockIndex, blockIndexBase));
        ptr = iBuilder->CreatePointerCast(ptr, pointerType);
        // GEP here is safe
        iBuilder->CreateStore(value, iBuilder->CreateGEP(ptr, blockOffset));

        iBuilder->setProducedItemCount(outputBufferName, iBuilder->CreateAdd(outputOffset, SIZE_ONE));
    }

    size_t LZ4BlockDecoderNewKernel::getOutputBufferSize(const unique_ptr<KernelBuilder> &iBuilder, const string& bufferName) {
//        size_t s = this->getOutputStreamSetBuffer(bufferName)->getBufferBlocks();
        return this->getOutputStreamSetBuffer(bufferName)->getBufferBlocks() * iBuilder->getStride();
    }
}