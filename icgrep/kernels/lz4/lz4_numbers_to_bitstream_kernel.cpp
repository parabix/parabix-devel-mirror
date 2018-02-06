//
// Created by wxy325 on 2017/8/9.
//

#include "lz4_numbers_to_bitstream_kernel.h"
#include <kernels/kernel_builder.h>
#include <iostream>
#include <string>
#include <llvm/Support/raw_ostream.h>
#include <kernels/streamset.h>

#define CURRENT_PROCESS_INDEX_KEY ("currentProcessIndex")
#define CURRENT_PACK_INDEX_KEY ("currentPackIndex")
#define CURRENT_PACK_START_VALUE_KEY ("currentPackStartValue")
#define CURRENT_PACK_END_VALUE_KEY ("currentPackEndValue")
#define CARRY_BIT_KEY ("carryBit")

#define START_NUM_STREAM_NAME ("startNumberStream")
#define END_NUM_STREAM_NAME ("endNumberStream")
#define OUTPUT_BIT_STREAM_NAME ("outputBitStream")


#define WORD_WIDTH (64)
#define LOG_2_WORD_WIDTH (std::log2(WORD_WIDTH))

using namespace llvm;
using namespace kernel;
using namespace std;

namespace kernel {
    void LZ4NumbersToBitstreamKernel::generateMultiBlockLogic(const std::unique_ptr<KernelBuilder> &iBuilder, Value * const numOfStrides) {
        //TODO
//    void LZ4NumbersToBitstreamKernel::generateDoSegmentMethod(const std::unique_ptr<KernelBuilder> &iBuilder) {
        BasicBlock* entryBlock = iBuilder->GetInsertBlock();
        BasicBlock* dataLoopCon = iBuilder->CreateBasicBlock("data_loop_con");
        BasicBlock* dataLoopBody = iBuilder->CreateBasicBlock("data_loop_body");
        BasicBlock* dataLoopExit = iBuilder->CreateBasicBlock("data_loop_exit");

        Value* initPackStartValue = iBuilder->getScalarField(CURRENT_PACK_START_VALUE_KEY);
        Value* initPackEndValue = iBuilder->getScalarField(CURRENT_PACK_END_VALUE_KEY);

        Value* initPackIndex = iBuilder->getScalarField(CURRENT_PACK_INDEX_KEY);
        Value* initProcessIndex = iBuilder->getScalarField(CURRENT_PROCESS_INDEX_KEY);
        Value* initCarryBit = iBuilder->getScalarField(CARRY_BIT_KEY);

        //EntryBlock
//        Value* numItemAvailable = iBuilder->CreateAdd(
//                iBuilder->getAvailableItemCount(START_NUM_STREAM_NAME),
//                iBuilder->getProcessedItemCount(START_NUM_STREAM_NAME)
//        );

        Value* numItemTotal = iBuilder->CreateAdd(iBuilder->getAvailableItemCount(START_NUM_STREAM_NAME), iBuilder->getProcessedItemCount(START_NUM_STREAM_NAME));
//        iBuilder->CallPrintInt("numItemAvailable", numItemAvailable);


        //dataLoopCon
        iBuilder->CreateBr(dataLoopCon);
        iBuilder->SetInsertPoint(dataLoopCon);
        PHINode* currentPackIndex = iBuilder->CreatePHI(iBuilder->getSizeTy(), 5);
        PHINode* currentPackStartValue = iBuilder->CreatePHI(iBuilder->getInt64Ty(), 5);
        PHINode* currentPackEndValue = iBuilder->CreatePHI(iBuilder->getInt64Ty(), 5);
        PHINode* currentProcessIndex = iBuilder->CreatePHI(iBuilder->getSizeTy(), 5);
        PHINode* isHandleStart = iBuilder->CreatePHI(iBuilder->getInt1Ty(), 5);
        PHINode* carryBit = iBuilder->CreatePHI(iBuilder->getInt64Ty(), 5);

        currentPackIndex->addIncoming(initPackIndex, entryBlock);
        currentPackStartValue->addIncoming(initPackStartValue, entryBlock);
        currentPackEndValue->addIncoming(initPackEndValue, entryBlock);
        currentProcessIndex->addIncoming(initProcessIndex, entryBlock);
        isHandleStart->addIncoming(iBuilder->getInt1(true), entryBlock);
        carryBit->addIncoming(initCarryBit, entryBlock);

        Value* notReachMaxNum = iBuilder->CreateICmpULT(currentProcessIndex, numItemTotal);
        iBuilder->CreateCondBr(notReachMaxNum, dataLoopBody, dataLoopExit);

        //dataLoopBody
        iBuilder->SetInsertPoint(dataLoopBody);

        BasicBlock* dataStartLoopBlock = iBuilder->CreateBasicBlock("data_start_loop_block");
        BasicBlock* dataEndLoopBlock = iBuilder->CreateBasicBlock("data_end_loop_block");


        iBuilder->CreateCondBr(isHandleStart, dataStartLoopBlock, dataEndLoopBlock);


///////////Handle Index Start
        //dataStartIndexBlock
        iBuilder->SetInsertPoint(dataStartLoopBlock);
        Value* bitOneStartIndex = this->generateLoadCircularInput(iBuilder, START_NUM_STREAM_NAME, currentProcessIndex, iBuilder->getInt64Ty()->getPointerTo());
        Value* bitOneStartTargetPackIndex = iBuilder->CreateLShr(bitOneStartIndex, iBuilder->getSize(LOG_2_WORD_WIDTH));

        BasicBlock* markBitOneStartBlock = iBuilder->CreateBasicBlock("mark_bit_one_start_block");
        BasicBlock* increasePackIndexStartBlock = iBuilder->CreateBasicBlock("increase_pack_index_start_block");

        iBuilder->CreateCondBr(iBuilder->CreateICmpULT(currentPackIndex, bitOneStartTargetPackIndex), increasePackIndexStartBlock, markBitOneStartBlock);

        //IncreasePackIndexStartBlock
        iBuilder->SetInsertPoint(increasePackIndexStartBlock);
        Value* currentPackOutputPtr = this->getPackOutputPtr(iBuilder, currentPackIndex);

        iBuilder->CreateStore(iBuilder->CreateSub(
                currentPackEndValue,
                iBuilder->CreateAdd(currentPackStartValue, carryBit)
        ), currentPackOutputPtr);


        Value* newCarryBit = iBuilder->CreateSelect(
                iBuilder->CreateICmpUGT(iBuilder->CreateAdd(currentPackStartValue, carryBit), currentPackEndValue),
                iBuilder->getInt64(1),
                iBuilder->getInt64(0)
        );

        iBuilder->setProducedItemCount(OUTPUT_BIT_STREAM_NAME, iBuilder->CreateShl(currentPackIndex, LOG_2_WORD_WIDTH));

        currentPackStartValue->addIncoming(iBuilder->getInt64(0), increasePackIndexStartBlock);
        currentPackEndValue->addIncoming(iBuilder->getInt64(0), increasePackIndexStartBlock);
        currentPackIndex->addIncoming(iBuilder->CreateAdd(currentPackIndex, iBuilder->getSize(1)), increasePackIndexStartBlock);
        currentProcessIndex->addIncoming(currentProcessIndex, increasePackIndexStartBlock);
        carryBit->addIncoming(newCarryBit, increasePackIndexStartBlock);


        isHandleStart->addIncoming(iBuilder->getInt1(true), increasePackIndexStartBlock);
        iBuilder->CreateBr(dataLoopCon);

        //markBitOneStartBlock
        iBuilder->SetInsertPoint(markBitOneStartBlock);
        Value* maskedBitOneIndex = iBuilder->CreateAnd(bitOneStartIndex, iBuilder->getSize(WORD_WIDTH - 1));
        Value* newPackValue = iBuilder->CreateOr(
                currentPackStartValue,
                iBuilder->CreateShl(iBuilder->getInt64(1), maskedBitOneIndex)
        );
        currentPackStartValue->addIncoming(newPackValue, markBitOneStartBlock);
        currentPackEndValue->addIncoming(currentPackEndValue, markBitOneStartBlock);
        currentPackIndex->addIncoming(currentPackIndex, markBitOneStartBlock);
        currentProcessIndex->addIncoming(currentProcessIndex, markBitOneStartBlock);
        isHandleStart->addIncoming(iBuilder->getInt1(false), markBitOneStartBlock);
        carryBit->addIncoming(carryBit, markBitOneStartBlock);
        iBuilder->CreateBr(dataLoopCon);


///////////Handle Index End
        iBuilder->SetInsertPoint(dataEndLoopBlock);
        Value* bitOneEndIndex = this->generateLoadCircularInput(iBuilder, END_NUM_STREAM_NAME, currentProcessIndex, iBuilder->getInt64Ty()->getPointerTo());
        Value* bitOneEndTargetPackIndex = iBuilder->CreateLShr(bitOneEndIndex, iBuilder->getSize(LOG_2_WORD_WIDTH));


        BasicBlock* markBitOneEndBlock = iBuilder->CreateBasicBlock("mark_bit_one_end_block");
        BasicBlock* increasePackIndexEndBlock = iBuilder->CreateBasicBlock("increase_pack_index_end_block");

        iBuilder->CreateCondBr(iBuilder->CreateICmpULT(currentPackIndex, bitOneEndTargetPackIndex), increasePackIndexEndBlock, markBitOneEndBlock);

        //IncreasePackIndexEndBlock
        iBuilder->SetInsertPoint(increasePackIndexEndBlock);
        currentPackOutputPtr = this->getPackOutputPtr(iBuilder, currentPackIndex);
        iBuilder->CreateStore(iBuilder->CreateSub(
                currentPackEndValue,
                iBuilder->CreateAdd(currentPackStartValue, carryBit)
        ), currentPackOutputPtr);



        newCarryBit = iBuilder->CreateSelect(
                iBuilder->CreateICmpUGT(iBuilder->CreateAdd(currentPackStartValue, carryBit), currentPackEndValue),
                iBuilder->getInt64(1),
                iBuilder->getInt64(0)
        );
        iBuilder->setProducedItemCount(OUTPUT_BIT_STREAM_NAME, iBuilder->CreateShl(currentPackIndex, LOG_2_WORD_WIDTH));

        currentPackStartValue->addIncoming(iBuilder->getInt64(0), increasePackIndexEndBlock);
        currentPackEndValue->addIncoming(iBuilder->getInt64(0), increasePackIndexEndBlock);
        currentPackIndex->addIncoming(iBuilder->CreateAdd(currentPackIndex, iBuilder->getSize(1)), increasePackIndexEndBlock);
        currentProcessIndex->addIncoming(currentProcessIndex, increasePackIndexEndBlock);
        isHandleStart->addIncoming(iBuilder->getInt1(false), increasePackIndexEndBlock);
        carryBit->addIncoming(newCarryBit, increasePackIndexEndBlock);
        iBuilder->CreateBr(dataLoopCon);

        //markBitOneEndBlock
        iBuilder->SetInsertPoint(markBitOneEndBlock);
        Value* maskedBitOneEndIndex = iBuilder->CreateAnd(bitOneEndIndex, iBuilder->getSize(WORD_WIDTH - 1));
        Value* newPackEndValue = iBuilder->CreateOr(
                currentPackEndValue,
                iBuilder->CreateShl(iBuilder->getInt64(1), maskedBitOneEndIndex)
        );
        currentPackStartValue->addIncoming(currentPackStartValue, markBitOneEndBlock);
        currentPackEndValue->addIncoming(newPackEndValue, markBitOneEndBlock);
        currentPackIndex->addIncoming(currentPackIndex, markBitOneEndBlock);
        currentProcessIndex->addIncoming(iBuilder->CreateAdd(currentProcessIndex, iBuilder->getSize(1)), markBitOneEndBlock);
        isHandleStart->addIncoming(iBuilder->getInt1(true), markBitOneEndBlock);
        carryBit->addIncoming(carryBit, markBitOneEndBlock);
        iBuilder->CreateBr(dataLoopCon);

        //dataLoopExit
        iBuilder->SetInsertPoint(dataLoopExit);
        iBuilder->setScalarField(CURRENT_PROCESS_INDEX_KEY, currentProcessIndex);
        iBuilder->setScalarField(CURRENT_PACK_INDEX_KEY, currentPackIndex);
        iBuilder->setScalarField(CURRENT_PACK_START_VALUE_KEY, currentPackStartValue);
        iBuilder->setScalarField(CURRENT_PACK_END_VALUE_KEY, currentPackEndValue);
        iBuilder->setScalarField(CARRY_BIT_KEY, carryBit);


        iBuilder->CreateStore(iBuilder->CreateSub(
                currentPackEndValue,
                iBuilder->CreateAdd(currentPackStartValue, carryBit)
        ), this->getPackOutputPtr(iBuilder, currentPackIndex));
        Value* lastBitOneIndex = this->generateLoadCircularInput(iBuilder, START_NUM_STREAM_NAME, iBuilder->CreateSub(currentProcessIndex, iBuilder->getSize(1)), iBuilder->getInt64Ty()->getPointerTo());
        iBuilder->setProducedItemCount(OUTPUT_BIT_STREAM_NAME, lastBitOneIndex);

        BasicBlock* exitBlock = iBuilder->CreateBasicBlock("exit_block");
//        iBuilder->setProcessedItemCount(START_NUM_STREAM_NAME, numItemAvailable);
        iBuilder->CreateBr(exitBlock);

        iBuilder->SetInsertPoint(exitBlock);
    }

    Value* LZ4NumbersToBitstreamKernel::generateLoadCircularInput(const unique_ptr<KernelBuilder> &iBuilder, string inputBufferName, Value* offset, Type* pointerType) {
        size_t inputSize = this->getInputBufferSize(iBuilder, inputBufferName);
        Value* offsetMask = iBuilder->getSize(inputSize - 1);
        Value* maskedOffset = iBuilder->CreateAnd(offsetMask, offset);

        Value* inputBufferPtr = iBuilder->getRawInputPointer(inputBufferName, iBuilder->getSize(0));

        inputBufferPtr = iBuilder->CreatePointerCast(inputBufferPtr, pointerType);
        return iBuilder->CreateLoad(iBuilder->CreateGEP(inputBufferPtr, maskedOffset));
    }

    size_t LZ4NumbersToBitstreamKernel::getInputBufferSize(const unique_ptr<KernelBuilder> &iBuilder, string bufferName) {
        return this->getInputStreamSetBuffer(bufferName)->getBufferBlocks() * iBuilder->getStride();
    }

    size_t LZ4NumbersToBitstreamKernel::getOutputBufferSize(const unique_ptr<KernelBuilder> &iBuilder, string bufferName) {
        return this->getOutputStreamSetBuffer(bufferName)->getBufferBlocks() * iBuilder->getStride();
    }

    inline Value* LZ4NumbersToBitstreamKernel::getPackOutputPtr(const std::unique_ptr<KernelBuilder> & iBuilder, Value* packIndex) {
        Value* outputBasePtr = iBuilder->getRawOutputPointer(OUTPUT_BIT_STREAM_NAME, iBuilder->getSize(0));
        outputBasePtr = iBuilder->CreatePointerCast(outputBasePtr, iBuilder->getInt64Ty()->getPointerTo());

        size_t outputBufferSize = this->getOutputBufferSize(iBuilder, OUTPUT_BIT_STREAM_NAME);
        size_t outputPackSize = outputBufferSize / 64;

        Value* maskedPackIndex = iBuilder->CreateAnd(packIndex, iBuilder->getSize(outputPackSize - 1));
        return iBuilder->CreateGEP(outputBasePtr, maskedPackIndex);
    }

    LZ4NumbersToBitstreamKernel::LZ4NumbersToBitstreamKernel(std::string kernelName, const std::unique_ptr<kernel::KernelBuilder> &iBuilder)
    : MultiBlockKernel(string(kernelName),
    // Inputs
    {
        Binding{iBuilder->getStreamSetTy(1, 64), START_NUM_STREAM_NAME},
        Binding{iBuilder->getStreamSetTy(1, 64), END_NUM_STREAM_NAME}
    },
    //Outputs
    {
        Binding{iBuilder->getStreamSetTy(1, 1), OUTPUT_BIT_STREAM_NAME, UnknownRate()}
    },
    //Arguments
    {
        //TODO may need total length
    },
    {},
    //Internal states:
    {
        Binding{iBuilder->getSizeTy(), CURRENT_PROCESS_INDEX_KEY},
        Binding{iBuilder->getSizeTy(), CURRENT_PACK_INDEX_KEY},
        Binding{iBuilder->getInt64Ty(), CURRENT_PACK_START_VALUE_KEY},
        Binding{iBuilder->getInt64Ty(), CURRENT_PACK_END_VALUE_KEY},
        Binding{iBuilder->getInt64Ty(), CARRY_BIT_KEY}

    }) {
//        setNoTerminateAttribute(true);
    }

}

