//
// Created by wxy325 on 2017/6/25.
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

LZ4BlockDecoderKernel::LZ4BlockDecoderKernel(const std::unique_ptr<kernel::KernelBuilder> &iBuilder)
: MultiBlockKernel("lz4BlockDecoder",
    // Inputs
    {Binding{iBuilder->getStreamSetTy(1, 8), "byteStream"}},
    //Outputs
    {
        Binding{iBuilder->getStreamSetTy(1, 1), "isCompressed", BoundedRate(0, 1)},
        Binding{iBuilder->getStreamSetTy(1, 64), "blockStart", BoundedRate(0, 1)},
        Binding{iBuilder->getStreamSetTy(1, 64), "blockEnd", BoundedRate(0, 1)}},
    //Arguments
    {
        Binding{iBuilder->getInt1Ty(), "hasBlockChecksum"},
        Binding{iBuilder->getSizeTy(), "headerSize"}
    },
    {},
    //Internal states:
    {
    Binding{iBuilder->getInt1Ty(), "hasSkipHeader"},
    Binding{iBuilder->getSizeTy(), "previousOffset"},
    Binding{iBuilder->getInt1Ty(), "reachFinalBlock"},
    })
, wordWidth{iBuilder->getSizeTy()->getBitWidth()} {
//    setNoTerminateAttribute(true);
}

void LZ4BlockDecoderKernel::generateMultiBlockLogic(const std::unique_ptr<KernelBuilder> &iBuilder, Value * const numOfStrides) {
//    BasicBlock * entry_block = iBuilder->GetInsertBlock();
//    iBuilder->CallPrintInt("block_available", iBuilder->getAvailableItemCount("byteStream"));
    BasicBlock * exit_block = iBuilder->CreateBasicBlock("exit");

//    BasicBlock * assert_fail_block = iBuilder->CreateBasicBlock("assert_fail_block");
//    BasicBlock * real_entry_block = iBuilder->CreateBasicBlock("real_entry_block");

    Value* hasSkipHeader = iBuilder->getScalarField("hasSkipHeader");
    Value* skipLength = iBuilder->CreateSelect(hasSkipHeader, iBuilder->getSize(0), iBuilder->getScalarField("headerSize"));
    iBuilder->setScalarField("hasSkipHeader", iBuilder->getInt1(true));
    skipLength = iBuilder->CreateAdd(skipLength, iBuilder->getScalarField("previousOffset"));

    Value* availableItemCount = iBuilder->getAvailableItemCount("byteStream");
    Value* processedItemCount = iBuilder->getProcessedItemCount("byteStream");
    Value* totalItemCount = iBuilder->CreateAdd(availableItemCount, processedItemCount);


    BasicBlock * block_decoder_entry = iBuilder->CreateBasicBlock("block_decoder_entry_block");
    iBuilder->CreateBr(block_decoder_entry);

    // block decoder entry
    iBuilder->SetInsertPoint(block_decoder_entry);


    BasicBlock * block_decoder_con = iBuilder->CreateBasicBlock("block_decoder_con_block");
    iBuilder->CreateBr(block_decoder_con);
    iBuilder->SetInsertPoint(block_decoder_con);

    PHINode* sOffset = iBuilder->CreatePHI(iBuilder->getSizeTy(), 2);
    iBuilder->setScalarField("previousOffset", sOffset);
    sOffset->addIncoming(skipLength, block_decoder_entry);

    BasicBlock * block_decoder_body = iBuilder->CreateBasicBlock("block_decoder_body_block");
    BasicBlock * block_decoder_exit = iBuilder->CreateBasicBlock("block_decoder_exit_block");
    BasicBlock * block_decoder_final = iBuilder->CreateBasicBlock("block_decoder_final_block");


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

    BasicBlock * block_decoder_output_block = iBuilder->CreateBasicBlock("block_decoder_output_block");

    iBuilder->CreateCondBr(isFinalBlock, block_decoder_final, block_decoder_output_block);

    // block_decoder_output_block
    iBuilder->SetInsertPoint(block_decoder_output_block);
    Value* blockStart = iBuilder->CreateAdd(sOffset, iBuilder->getSize(4));
    Value* blockEnd = iBuilder->CreateAdd(blockStart, realBlockSize);
    this->appendOutput(iBuilder, isCompressed, blockStart, blockEnd);
    iBuilder->CreateBr(block_decoder_final);

    // block_decoder_final_block
    iBuilder->SetInsertPoint(block_decoder_final);

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

    sOffset->addIncoming(newOffset, block_decoder_final);
    iBuilder->CreateBr(block_decoder_con);

    // block_decoder_exit_block
    iBuilder->SetInsertPoint(block_decoder_exit);


    iBuilder->CreateBr(exit_block);
    iBuilder->SetInsertPoint(exit_block);
}


    Value* LZ4BlockDecoderKernel::generateLoadInput(const std::unique_ptr<KernelBuilder> & iBuilder, llvm::Value* offset) {
        // TODO adjust input loading
        Value * inputBufferBasePtr = iBuilder->getRawInputPointer("byteStream", iBuilder->getSize(0));
//        offset = iBuilder->CreateSub(iBuilder->getProcessedItemCount("byteStream"), offset);
        Value* targetPtr = iBuilder->CreateGEP(inputBufferBasePtr, offset);
        return iBuilder->CreateLoad(targetPtr);
    }

    void LZ4BlockDecoderKernel::appendOutput(const std::unique_ptr<KernelBuilder> & iBuilder, Value* isCompressed, Value* blockStart, Value* blockEnd) {
        // TODO adjust output storing
        this->generateStoreCircularOutput(iBuilder, "isCompressed", iBuilder->getInt1Ty()->getPointerTo(), isCompressed);
        this->generateStoreCircularOutput(iBuilder, "blockStart", iBuilder->getInt64Ty()->getPointerTo(), iBuilder->CreateTruncOrBitCast(blockStart, iBuilder->getInt64Ty()));
        this->generateStoreCircularOutput(iBuilder, "blockEnd", iBuilder->getInt64Ty()->getPointerTo(), blockEnd);
    }

    void LZ4BlockDecoderKernel::generateStoreCircularOutput(const unique_ptr<KernelBuilder> &iBuilder, const string& outputBufferName, Type* pointerType, Value* value) {
        Value* offset = iBuilder->getProducedItemCount(outputBufferName);

        size_t inputSize = this->getOutputBufferSize(iBuilder, outputBufferName);
        Value* offsetMask = iBuilder->getSize(inputSize - 1);
        Value* maskedOffset = iBuilder->CreateAnd(offsetMask, offset);

        Value* outputBufferPtr = iBuilder->getRawOutputPointer(outputBufferName, iBuilder->getSize(0));

        outputBufferPtr = iBuilder->CreatePointerCast(outputBufferPtr, pointerType);
        iBuilder->CreateStore(value, iBuilder->CreateGEP(outputBufferPtr, maskedOffset));

        offset = iBuilder->CreateAdd(offset, iBuilder->getSize(1));
        iBuilder->setProducedItemCount(outputBufferName, offset);
    }

    size_t LZ4BlockDecoderKernel::getOutputBufferSize(const unique_ptr<KernelBuilder> &iBuilder, const string& bufferName) {
//        size_t s = this->getOutputStreamSetBuffer(bufferName)->getBufferBlocks();
        return this->getOutputStreamSetBuffer(bufferName)->getBufferBlocks() * iBuilder->getStride();
    }
}
