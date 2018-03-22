
#include "lz4_numbers_to_bitstream_kernel.h"
#include <kernels/kernel_builder.h>
#include <iostream>
#include <string>
#include <llvm/Support/raw_ostream.h>
#include <kernels/streamset.h>

#define START_NUM_STREAM_NAME ("startNumberStream")
#define END_NUM_STREAM_NAME ("endNumberStream")
#define OUTPUT_BIT_STREAM_NAME ("outputBitStream__")

#define PENDING_START_DATA_KEY ("pendingStartData")
#define PENDING_END_DATA_KEY ("pendingEndData")

using namespace llvm;
using namespace kernel;
using namespace std;

namespace kernel {

    Value* LZ4NumbersToBitstreamKernel::loadInt64NumberInput(const unique_ptr<KernelBuilder> &iBuilder, string bufferName, Value* offset) {
        // GEP here is safe
        Value* SIZE_BIT_BLOCK_WIDTH = iBuilder->getSize(iBuilder->getBitBlockWidth());
        Value* inputLocalBlockIndex = iBuilder->CreateUDiv(offset, SIZE_BIT_BLOCK_WIDTH);
        Value* inputLocalBlockOffset = iBuilder->CreateURem(offset, SIZE_BIT_BLOCK_WIDTH);

        Value* blockBasePtr = iBuilder->getInputStreamBlockPtr(bufferName, iBuilder->getSize(0), inputLocalBlockIndex);
        blockBasePtr = iBuilder->CreatePointerCast(blockBasePtr, iBuilder->getInt64Ty()->getPointerTo());
        // GEP here is safe
        return iBuilder->CreateLoad(iBuilder->CreateGEP(blockBasePtr, inputLocalBlockOffset));
    }

    void LZ4NumbersToBitstreamKernel::generateMultiBlockLogic(const std::unique_ptr<KernelBuilder> &iBuilder,
                                                               llvm::Value *const numOfStrides) {
        // Const
        Constant *SIZE_ZERO = iBuilder->getSize(0);
        Constant *SIZE_ONE = iBuilder->getSize(1);
        Constant *INT64_ZERO = iBuilder->getInt64(0);
        Constant *INT64_ONE = iBuilder->getInt64(1);

        unsigned int BIT_BLOCK_WIDTH = iBuilder->getBitBlockWidth();
        Type * const INT_BIT_BLOCK_TY = iBuilder->getIntNTy(BIT_BLOCK_WIDTH);
        Constant *SIZE_BIT_BLOCK_WIDTH = iBuilder->getSize(BIT_BLOCK_WIDTH);
        Constant* INT_BIT_BLOCK_ZERO = ConstantInt::get(INT_BIT_BLOCK_TY, 0);
        Value* BIT_BLOCK_ZERO = iBuilder->CreateBitCast(INT_BIT_BLOCK_ZERO, iBuilder->getBitBlockType());


        size_t outputBufferBlocks = this->getAnyBufferSize(iBuilder, OUTPUT_BIT_STREAM_NAME) / iBuilder->getStride();
        Value *outputRawBeginPtr = iBuilder->CreatePointerCast(
                iBuilder->getRawOutputPointer(OUTPUT_BIT_STREAM_NAME, SIZE_ZERO),
                iBuilder->getBitBlockType()->getPointerTo());
        Value *outputCurrentPtr = iBuilder->getOutputStreamBlockPtr(OUTPUT_BIT_STREAM_NAME, SIZE_ZERO);
        Value *offset = iBuilder->CreatePtrDiff(outputCurrentPtr, outputRawBeginPtr);
        Value *remainSpace = iBuilder->CreateSub(iBuilder->getSize(outputBufferBlocks), offset);
//        iBuilder->CallPrintInt("remainSpace",
//                               remainSpace); //TODO workaround here, kernel infrastructure should provide the information about how much data we can produced


        BasicBlock *entryBlock = iBuilder->GetInsertBlock();


        Value *itemsToDo = mAvailableItemCount[0];
//        iBuilder->CallPrintInt("itemsToDo", itemsToDo);
        Value *isFinalBlock = iBuilder->CreateICmpEQ(itemsToDo, iBuilder->getSize(0));
        iBuilder->setTerminationSignal(isFinalBlock);

        Value *itemProcessed = iBuilder->getProcessedItemCount(START_NUM_STREAM_NAME);
        Value *oldProducedItemCount = iBuilder->getProducedItemCount(OUTPUT_BIT_STREAM_NAME);
        Value *oldProducedOutputBlockIndex = iBuilder->CreateUDiv(oldProducedItemCount,
                                                                  SIZE_BIT_BLOCK_WIDTH); // always produce full block except for final block

        Value *initCurrentItemIndex = iBuilder->CreateURem(itemProcessed, SIZE_BIT_BLOCK_WIDTH);

        Value *initOutputIndex = SIZE_ZERO;


        Value *availableOutputBlocks = iBuilder->CreateUMin(remainSpace, numOfStrides);

//        Value *inputStartBasePtr = iBuilder->getInputStreamBlockPtr(START_NUM_STREAM_NAME, SIZE_ZERO);
//        inputStartBasePtr = iBuilder->CreatePointerCast(inputStartBasePtr, iBuilder->getInt64Ty()->getPointerTo());
//        Value *inputEndBasePtr = iBuilder->getInputStreamBlockPtr(END_NUM_STREAM_NAME, SIZE_ZERO);
//        inputEndBasePtr = iBuilder->CreatePointerCast(inputEndBasePtr, iBuilder->getInt64Ty()->getPointerTo());
        Value *outputBasePtr = iBuilder->getOutputStreamBlockPtr(OUTPUT_BIT_STREAM_NAME, SIZE_ZERO);
        Value *initCarryBit = iBuilder->getScalarField("carryBit");

//        iBuilder->CallPrintInt("itemProcessed", itemProcessed);
//        iBuilder->CallPrintInt("inputStartBasePtr", inputStartBasePtr);

        Value *initCurrentBlockStartData = iBuilder->getScalarField(PENDING_START_DATA_KEY);
        Value *initCurrentBlockEndData = iBuilder->getScalarField(PENDING_END_DATA_KEY);


        BasicBlock *multiBlockLoopConBlock = iBuilder->CreateBasicBlock("multiBlockLoopConBlock");
        BasicBlock *multiBlockLoopBodyBlock = iBuilder->CreateBasicBlock("multiBlockLoopBodyBlock");
        BasicBlock *multiBlockLoopExitBlock = iBuilder->CreateBasicBlock("multiBlockLoopExitBlock");

        iBuilder->CreateBr(multiBlockLoopConBlock);

        // multiBlockLoopConBlock
        iBuilder->SetInsertPoint(multiBlockLoopConBlock);
        PHINode *phiCurrentItemIndex = iBuilder->CreatePHI(iBuilder->getSizeTy(), 2);
        phiCurrentItemIndex->addIncoming(initCurrentItemIndex, entryBlock);

        PHINode *phiCurrentOutputIndex = iBuilder->CreatePHI(iBuilder->getSizeTy(), 2);
        phiCurrentOutputIndex->addIncoming(initOutputIndex, entryBlock);

        PHINode *phiCurrentBlockStartData = iBuilder->CreatePHI(iBuilder->getBitBlockType(), 2);
        phiCurrentBlockStartData->addIncoming(initCurrentBlockStartData, entryBlock);

        PHINode *phiCurrentBlockEndData = iBuilder->CreatePHI(iBuilder->getBitBlockType(), 2);
        phiCurrentBlockEndData->addIncoming(initCurrentBlockEndData, entryBlock);

        PHINode *phiCarryBit = iBuilder->CreatePHI(iBuilder->getInt64Ty(), 2);
        phiCarryBit->addIncoming(initCarryBit, entryBlock);

        iBuilder->CreateCondBr(
                iBuilder->CreateAnd(
                        iBuilder->CreateICmpULT(phiCurrentItemIndex, iBuilder->CreateAdd(itemsToDo,
                                                                                         initCurrentItemIndex)),
                        iBuilder->CreateICmpULT(phiCurrentOutputIndex, availableOutputBlocks)
                ),
                multiBlockLoopBodyBlock,
                multiBlockLoopExitBlock
        );

        // multiBlockLoopBodyBlock
        iBuilder->SetInsertPoint(multiBlockLoopBodyBlock);

        Value *currentOutputGlobalIndex = iBuilder->CreateAdd(phiCurrentOutputIndex, oldProducedOutputBlockIndex);
        // StartBits
        Value *currentStartPos = this->loadInt64NumberInput(iBuilder, START_NUM_STREAM_NAME, phiCurrentItemIndex);
        Value *currentStartGlobalBlockIndex = iBuilder->CreateUDiv(currentStartPos, SIZE_BIT_BLOCK_WIDTH);

        Value *currentStartLocalBlockOffset = iBuilder->CreateURem(currentStartPos,
                                                                   SIZE_BIT_BLOCK_WIDTH); // 0 ~ BIT_BLOCK_WIDTH

        Value *newBlockStartData = this->setIntVectorBitOne(iBuilder, phiCurrentBlockStartData,
                                                            currentStartLocalBlockOffset,
                                                            iBuilder->CreateICmpEQ(currentStartGlobalBlockIndex,
                                                                                   currentOutputGlobalIndex));

        // EndBits
        Value *currentEndPos = this->loadInt64NumberInput(iBuilder, END_NUM_STREAM_NAME, phiCurrentItemIndex);
        Value *currentEndGlobalBlockIndex = iBuilder->CreateUDiv(currentEndPos, SIZE_BIT_BLOCK_WIDTH);

        Value *currentEndLocalBlockOffset = iBuilder->CreateURem(currentEndPos,
                                                                 SIZE_BIT_BLOCK_WIDTH); // 0 ~ BIT_BLOCK_WIDTH


        Value *newBlockEndData = this->setIntVectorBitOne(iBuilder, phiCurrentBlockEndData, currentEndLocalBlockOffset,
                                                          iBuilder->CreateICmpEQ(currentEndGlobalBlockIndex,
                                                                                 currentOutputGlobalIndex));

        Value *enterNewOutputBlock = iBuilder->CreateOr(
                iBuilder->CreateICmpUGT(currentStartGlobalBlockIndex, currentOutputGlobalIndex),
                iBuilder->CreateICmpUGT(currentEndGlobalBlockIndex, currentOutputGlobalIndex)
        );


        Value *carryBitIntVec = iBuilder->CreateInsertElement(BIT_BLOCK_ZERO, phiCarryBit, (uint64_t) 0);
        Value *newBlockStartWithCarry = iBuilder->simd_add(BIT_BLOCK_WIDTH, newBlockStartData, carryBitIntVec);


        // Avoid branch mis-prediction by always storing output block
        Value *outputData = iBuilder->simd_sub(BIT_BLOCK_WIDTH, newBlockEndData, newBlockStartWithCarry);

        iBuilder->CreateBlockAlignedStore(outputData, iBuilder->getOutputStreamBlockPtr(OUTPUT_BIT_STREAM_NAME, SIZE_ZERO, phiCurrentOutputIndex));

        // Handle PHINodes

        // When currentStartLocalBlockIndex < phiCurrentOutputIndex && currentEndLocalBlockIndex < phiCurrentOutputIndex
        // this round of loop will do nothing, and currentItemIndex += 1
        phiCurrentItemIndex->addIncoming(
                iBuilder->CreateSelect(
                        enterNewOutputBlock,
                        phiCurrentItemIndex,
                        iBuilder->CreateAdd(phiCurrentItemIndex, SIZE_ONE)
                ),
                iBuilder->GetInsertBlock()
        );

        phiCurrentOutputIndex->addIncoming(
                iBuilder->CreateSelect(
                        enterNewOutputBlock,
                        iBuilder->CreateAdd(phiCurrentOutputIndex, SIZE_ONE),
                        phiCurrentOutputIndex
                ),
                iBuilder->GetInsertBlock()
        );

        phiCurrentBlockStartData->addIncoming(
                iBuilder->CreateSelect(
                        enterNewOutputBlock,
                        BIT_BLOCK_ZERO,
                        newBlockStartData
                ),
                iBuilder->GetInsertBlock()
        );

        phiCurrentBlockEndData->addIncoming(
                iBuilder->CreateSelect(
                        enterNewOutputBlock,
                        BIT_BLOCK_ZERO,
                        newBlockEndData
                ),
                iBuilder->GetInsertBlock()
        );

        Value *newCarryBit = iBuilder->CreateSelect(this->intVecGT(iBuilder, newBlockStartWithCarry, newBlockEndData),
                                                    INT64_ONE, INT64_ZERO);

//        iBuilder->CallPrintInt("newCarryBit", newCarryBit );

        phiCarryBit->addIncoming(
                iBuilder->CreateSelect(
                        enterNewOutputBlock,
                        newCarryBit,
                        phiCarryBit
                ),
                iBuilder->GetInsertBlock()
        );


        iBuilder->CreateBr(multiBlockLoopConBlock);

        // multiBlockLoopExitBlock
        iBuilder->SetInsertPoint(multiBlockLoopExitBlock);

        iBuilder->setScalarField(PENDING_START_DATA_KEY, phiCurrentBlockStartData);
        iBuilder->setScalarField(PENDING_END_DATA_KEY, phiCurrentBlockEndData);
        iBuilder->setScalarField("carryBit", phiCarryBit);

        carryBitIntVec = iBuilder->CreateInsertElement(BIT_BLOCK_ZERO, phiCarryBit, (uint64_t) 0);
        Value *finalOutputData = iBuilder->simd_sub(
                BIT_BLOCK_WIDTH,
                phiCurrentBlockEndData,
                iBuilder->simd_add(BIT_BLOCK_WIDTH, phiCurrentBlockStartData, carryBitIntVec)
        );

        BasicBlock *storeFinalBlock = iBuilder->CreateBasicBlock("storeFinalBlock");
        BasicBlock *storeFinalBlockEnd = iBuilder->CreateBasicBlock("storeFinalBlockEnd");

        iBuilder->CreateUnlikelyCondBr(isFinalBlock, storeFinalBlock, storeFinalBlockEnd);
        iBuilder->SetInsertPoint(storeFinalBlock);

//        iBuilder->CallPrintRegister("finalOutputData", finalOutputData);
        iBuilder->CreateBlockAlignedStore(finalOutputData, iBuilder->getOutputStreamBlockPtr(OUTPUT_BIT_STREAM_NAME, SIZE_ZERO, phiCurrentOutputIndex)); //Possible overflow here if this store always happen
        iBuilder->CreateBr(storeFinalBlockEnd);
        iBuilder->SetInsertPoint(storeFinalBlockEnd);

        // Processed Item Count and Produced Item Count
        Value *newProcessedItemCount = iBuilder->CreateAdd(iBuilder->getProcessedItemCount(START_NUM_STREAM_NAME),
                                                           iBuilder->CreateSub(phiCurrentItemIndex,
                                                                               initCurrentItemIndex));


        iBuilder->setProcessedItemCount(START_NUM_STREAM_NAME, newProcessedItemCount);
        iBuilder->setProcessedItemCount(END_NUM_STREAM_NAME, newProcessedItemCount);

        Value *lastEndPos = this->loadInt64NumberInput(iBuilder, END_NUM_STREAM_NAME, iBuilder->CreateSub(phiCurrentItemIndex, SIZE_ONE));

        iBuilder->setProducedItemCount(OUTPUT_BIT_STREAM_NAME,
                                       iBuilder->CreateSelect(
                                               isFinalBlock,
                                               lastEndPos,
                                               iBuilder->CreateAdd(
                                                       iBuilder->CreateMul(phiCurrentOutputIndex, SIZE_BIT_BLOCK_WIDTH),
                                                       iBuilder->getProducedItemCount(OUTPUT_BIT_STREAM_NAME)
                                               )
                                       )
        );
//        iBuilder->CallPrintInt("isFinalBlock", isFinalBlock);
//        iBuilder->CallPrintInt("producedItemCount", iBuilder->getProducedItemCount(OUTPUT_BIT_STREAM_NAME));
    }

    size_t LZ4NumbersToBitstreamKernel::getAnyBufferSize(const std::unique_ptr<KernelBuilder> &iBuilder,
                                                          std::string bufferName) {
        return this->getAnyStreamSetBuffer(bufferName)->getBufferBlocks() * iBuilder->getStride();
    }

    /*
     * iBuilder: kernel builder
     * intVec: BitBlockType
     * pos: size_t, 0 - bitBlockWidth, position of bit 1
     * isSet: i1, when isSet == true, bit 1 will be set, otherwise this function do nothing
     * */
    Value *LZ4NumbersToBitstreamKernel::setIntVectorBitOne(const std::unique_ptr<KernelBuilder> &iBuilder,
                                                            llvm::Value *intVec, llvm::Value *pos, llvm::Value *isSet) {
        Type* BIT_BLOCK_TYPE = iBuilder->getBitBlockType();
        Type* BIT_BLOCK_WIDTH_INT_TYPE = iBuilder->getIntNTy(iBuilder->getBitBlockWidth());

        Value* sourceInt = iBuilder->CreateBitCast(intVec, BIT_BLOCK_WIDTH_INT_TYPE);
        Value *oneBit = iBuilder->CreateShl(
                iBuilder->CreateSelect(isSet, ConstantInt::get(BIT_BLOCK_WIDTH_INT_TYPE, 1),
                                       ConstantInt::get(BIT_BLOCK_WIDTH_INT_TYPE, 0)),
                iBuilder->CreateZExt(pos, BIT_BLOCK_WIDTH_INT_TYPE)
        );
        return iBuilder->CreateBitCast(iBuilder->CreateOr(sourceInt, oneBit), BIT_BLOCK_TYPE);
    }

    Value *LZ4NumbersToBitstreamKernel::intVecGT(const std::unique_ptr<KernelBuilder> &iBuilder, llvm::Value *intVec1,
                                                  llvm::Value *intVec2) {
        unsigned int BIT_BLOCK_WIDTH = iBuilder->getBitBlockWidth();
        Value *gt = iBuilder->simd_ugt(BIT_BLOCK_WIDTH, intVec1, intVec2);
        return iBuilder->CreateNot(iBuilder->CreateICmpEQ(iBuilder->CreateExtractElement(gt, (uint64_t) 0),
                                                          iBuilder->getIntN(BIT_BLOCK_WIDTH, 0)));
    }


    LZ4NumbersToBitstreamKernel::LZ4NumbersToBitstreamKernel(std::string kernelName,
                                                               const std::unique_ptr<kernel::KernelBuilder> &iBuilder)
            : MultiBlockKernel(string(kernelName),
            // Inputs
                               {
                                       Binding{iBuilder->getStreamSetTy(1, 64), START_NUM_STREAM_NAME,
                                               BoundedRate(0, 1), AlwaysConsume()},
                                       Binding{iBuilder->getStreamSetTy(1, 64), END_NUM_STREAM_NAME, BoundedRate(0, 1),
                                               AlwaysConsume()}
                               },
            //Outputs
                               {
//                                       Binding{iBuilder->getStreamSetTy(1, 1), OUTPUT_BIT_STREAM_NAME,
//                                           UnknownRate()}
                                       Binding{iBuilder->getStreamSetTy(1, 1), OUTPUT_BIT_STREAM_NAME,
                                                   BoundedRate(0, 1)}   //TODO BoundedRate is a workaround, it should be UnknownRate in the future
                               },
            //Arguments
                               {
                               },
                               {},
            //Internal states:
                               {
                    Binding(iBuilder->getBitBlockType(), PENDING_START_DATA_KEY),
                    Binding(iBuilder->getBitBlockType(), PENDING_END_DATA_KEY),
                    Binding(iBuilder->getIntNTy(64), "carryBit"),
            }) {
//        addAttribute(CanTerminateEarly());
//        setNoTerminateAttribute(true);
        addAttribute(MustExplicitlyTerminate());
    }
}