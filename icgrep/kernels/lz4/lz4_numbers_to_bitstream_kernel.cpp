
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

    void LZ4NumbersToBitstreamKernel::generateMultiBlockLogic(const std::unique_ptr<KernelBuilder> &iBuilder,
                                                               llvm::Value *const numOfStrides) {

//        iBuilder->CallPrintInt("======Entry", iBuilder->getSize(0));
//        iBuilder->CallPrintInt("mIsFinal", mIsFinal);
//        iBuilder->CallPrintInt("numOfStrides", numOfStrides);

        // Const
        Constant *SIZE_ZERO = iBuilder->getSize(0);
        Constant *SIZE_ONE = iBuilder->getSize(1);
        Constant *INT64_ZERO = iBuilder->getInt64(0);
        Constant *INT64_ONE = iBuilder->getInt64(1);
        Constant *BIT_BLOCK_ZERO = llvm::ConstantVector::get(
                {INT64_ZERO, INT64_ZERO, INT64_ZERO, INT64_ZERO}); // TODO Assumed bit block type is always <4 * i64>
        unsigned int BIT_BLOCK_WIDTH = iBuilder->getBitBlockWidth();
        Constant *SIZE_BIT_BLOCK_WIDTH = iBuilder->getSize(BIT_BLOCK_WIDTH);


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


//        Value *initCurrentItemIndex = iBuilder->CreateSelect(
//                isFinalBlock,
//                SIZE_ZERO,
//                iBuilder->CreateURem(itemProcessed, SIZE_BIT_BLOCK_WIDTH)
//        );

        Value *initCurrentItemIndex = iBuilder->CreateURem(itemProcessed, SIZE_BIT_BLOCK_WIDTH);

        Value *initOutputIndex = SIZE_ZERO;

//        Value *availableOutputBlocks = iBuilder->CreateSelect(mIsFinal, iBuilder->getSize(32), numOfStrides); //TODO workaround here
//        Value *availableOutputBlocks = numOfStrides;
//        Value *availableOutputBlocks = remainSpace;
        Value *availableOutputBlocks = iBuilder->CreateUMin(remainSpace, numOfStrides);

        // TODO handle input pointer
        Value *inputStartBasePtr = iBuilder->getInputStreamBlockPtr(START_NUM_STREAM_NAME, SIZE_ZERO);
        inputStartBasePtr = iBuilder->CreatePointerCast(inputStartBasePtr, iBuilder->getInt64Ty()->getPointerTo());
        Value *inputEndBasePtr = iBuilder->getInputStreamBlockPtr(END_NUM_STREAM_NAME, SIZE_ZERO);
        inputEndBasePtr = iBuilder->CreatePointerCast(inputEndBasePtr, iBuilder->getInt64Ty()->getPointerTo());
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


        // TODO It is possible that in final block, not all items have been processed, while the output buffer is not enough. This situation need to be verified later
        // phiCurrentItemIndex < itemsToDo && currentOutputIndex < availableOutputBlocks
//        iBuilder->CallPrintInt("phiCurrentItemIndex", phiCurrentItemIndex);
//        iBuilder->CallPrintInt("aaa", iBuilder->CreateAdd(itemsToDo, initCurrentItemIndex));
        iBuilder->CreateCondBr(
                iBuilder->CreateAnd(
                        iBuilder->CreateICmpULT(phiCurrentItemIndex, iBuilder->CreateAdd(itemsToDo,
                                                                                         initCurrentItemIndex)), //TODO should not be itemsToDo here, may be itemsToDo + initCurrentItemIndex
                        iBuilder->CreateICmpULT(phiCurrentOutputIndex, availableOutputBlocks)
                ),
                multiBlockLoopBodyBlock,
                multiBlockLoopExitBlock
        );

        // multiBlockLoopBodyBlock
        iBuilder->SetInsertPoint(multiBlockLoopBodyBlock);

        Value *currentOutputGlobalIndex = iBuilder->CreateAdd(phiCurrentOutputIndex, oldProducedOutputBlockIndex);

        // StartBits
        Value *currentStartPos = iBuilder->CreateLoad(iBuilder->CreateGEP(inputStartBasePtr, phiCurrentItemIndex));
        Value *currentStartGlobalBlockIndex = iBuilder->CreateUDiv(currentStartPos, SIZE_BIT_BLOCK_WIDTH);
//        Value *currentStartLocalBlockIndex = iBuilder->CreateSub(currentStartGlobalBlockIndex,
//                                                                 oldProducedOutputBlockIndex);
//        iBuilder->CallPrintInt("currentStartLocalBlockIndex", currentStartLocalBlockIndex); //TODO overflow here


        Value *currentStartLocalBlockOffset = iBuilder->CreateURem(currentStartPos,
                                                                   SIZE_BIT_BLOCK_WIDTH); // 0 ~ BIT_BLOCK_WIDTH

        Value *newBlockStartData = this->setIntVectorBitOne(iBuilder, phiCurrentBlockStartData,
                                                            currentStartLocalBlockOffset,
                                                            iBuilder->CreateICmpEQ(currentStartGlobalBlockIndex,
                                                                                   currentOutputGlobalIndex));
//        iBuilder->CallPrintRegister("phiCurrentBlockStartData", phiCurrentBlockStartData);
//        iBuilder->CallPrintRegister("newBlockStartData", newBlockStartData);
//        iBuilder->CallPrintInt("currentStartPos", currentStartPos);
//        iBuilder->CallPrintInt("----", SIZE_ZERO);


        // EndBits
        Value *currentEndPos = iBuilder->CreateLoad(iBuilder->CreateGEP(inputEndBasePtr, phiCurrentItemIndex));
        Value *currentEndGlobalBlockIndex = iBuilder->CreateUDiv(currentEndPos, SIZE_BIT_BLOCK_WIDTH);
//        Value *currentEndLocalBlockIndex = iBuilder->CreateSub(currentEndGlobalBlockIndex, oldProducedOutputBlockIndex);

        Value *currentEndLocalBlockOffset = iBuilder->CreateURem(currentEndPos,
                                                                 SIZE_BIT_BLOCK_WIDTH); // 0 ~ BIT_BLOCK_WIDTH


        Value *newBlockEndData = this->setIntVectorBitOne(iBuilder, phiCurrentBlockEndData, currentEndLocalBlockOffset,
                                                          iBuilder->CreateICmpEQ(currentEndGlobalBlockIndex,
                                                                                 currentOutputGlobalIndex));
//            iBuilder->CallPrintInt("%%%currentEndPos", currentEndPos);
//            iBuilder->CallPrintRegister("%%%newBlockEndData", newBlockEndData);
//        iBuilder->CallPrintInt("currentEndPos", currentEndPos);

        Value *enterNewOutputBlock = iBuilder->CreateOr(
                iBuilder->CreateICmpUGT(currentStartGlobalBlockIndex, currentOutputGlobalIndex),
                iBuilder->CreateICmpUGT(currentEndGlobalBlockIndex, currentOutputGlobalIndex)
        );


        Value *carryBitIntVec = iBuilder->CreateInsertElement(BIT_BLOCK_ZERO, phiCarryBit, (uint64_t) 0);
        Value *newBlockStartWithCarry = iBuilder->simd_add(BIT_BLOCK_WIDTH, newBlockStartData, carryBitIntVec);


        // Avoid branch mis-prediction by always storing output block
        Value *outputData = iBuilder->simd_sub(BIT_BLOCK_WIDTH, newBlockEndData, newBlockStartWithCarry);
//        iBuilder->CallPrintInt("----store", iBuilder->getSize(0));
//        iBuilder->CallPrintInt("carry", phiCarryBit);
//        iBuilder->CallPrintRegister("newBlockEndData", newBlockEndData);
//        iBuilder->CallPrintRegister("newBlockStartWithCarry", newBlockStartWithCarry);
//        iBuilder->CallPrintInt("----outputPtr", iBuilder->CreateGEP(outputBasePtr, phiCurrentOutputIndex));
//        iBuilder->CallPrintRegister("outputData", outputData);
        iBuilder->CreateBlockAlignedStore(outputData, iBuilder->CreateGEP(outputBasePtr, phiCurrentOutputIndex));

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
//        iBuilder->CallPrintRegister("%%%phiCurrentBlockEndData", phiCurrentBlockEndData);
//            iBuilder->CallPrintInt("----outputPtrFinal", iBuilder->CreateGEP(outputBasePtr, phiCurrentOutputIndex));

        BasicBlock *storeFinalBlock = iBuilder->CreateBasicBlock("storeFinalBlock");
        BasicBlock *storeFinalBlockEnd = iBuilder->CreateBasicBlock("storeFinalBlockEnd");

        iBuilder->CreateUnlikelyCondBr(isFinalBlock, storeFinalBlock, storeFinalBlockEnd);
        iBuilder->SetInsertPoint(storeFinalBlock);

//        iBuilder->CallPrintRegister("finalOutputData", finalOutputData);
        iBuilder->CreateBlockAlignedStore(finalOutputData, iBuilder->CreateGEP(outputBasePtr,
                                                                   phiCurrentOutputIndex)); //Possible overflow here if this store always happen
        iBuilder->CreateBr(storeFinalBlockEnd);
        iBuilder->SetInsertPoint(storeFinalBlockEnd);

        // Processed Item Count and Produced Item Count
        Value *newProcessedItemCount = iBuilder->CreateAdd(iBuilder->getProcessedItemCount(START_NUM_STREAM_NAME),
                                                           iBuilder->CreateSub(phiCurrentItemIndex,
                                                                               initCurrentItemIndex));


        iBuilder->setProcessedItemCount(START_NUM_STREAM_NAME, newProcessedItemCount);
        iBuilder->setProcessedItemCount(END_NUM_STREAM_NAME, newProcessedItemCount);

        Value *lastEndPos = iBuilder->CreateLoad(
                iBuilder->CreateGEP(inputEndBasePtr, iBuilder->CreateSub(phiCurrentItemIndex, SIZE_ONE)));
//        iBuilder->CallPrintInt("lastEndPos", lastEndPos);

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
     * intVec: BitBlockType, <4 * i64>
     * pos: size_t, 0 - 256, position of bit 1
     * isSet: i1, when isSet == true, bit 1 will be set, otherwise this function do nothing
     * */
    Value *LZ4NumbersToBitstreamKernel::setIntVectorBitOne(const std::unique_ptr<KernelBuilder> &iBuilder,
                                                            llvm::Value *intVec, llvm::Value *pos, llvm::Value *isSet) {
        Value *SIZE_64 = iBuilder->getSize(64); //TODO assume bit block type will always be <4 * i64>
        Value *blockIndex = iBuilder->CreateUDiv(pos, SIZE_64);
        Value *blockOffset = iBuilder->CreateURem(pos, SIZE_64);

        Value *oldValue = iBuilder->CreateExtractElement(intVec, blockIndex);
        // Use select to avoid branch misprediction
        Value *bitOneValue = iBuilder->CreateShl(
                iBuilder->CreateSelect(isSet, iBuilder->getInt64(1), iBuilder->getInt64(0)),
                blockOffset
        );
        Value *newValue = iBuilder->CreateOr(oldValue, bitOneValue);
        return iBuilder->CreateInsertElement(intVec, newValue, blockIndex);
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