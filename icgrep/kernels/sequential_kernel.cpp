
#include "sequential_kernel.h"
#include <kernels/kernel_builder.h>
#include <kernels/streamset.h>
#include <iostream>
#include <string>
#include <llvm/Support/raw_ostream.h>


using namespace llvm;
using namespace kernel;
using namespace parabix;
using namespace std;

#define SequentialSegmentStateKey ("SequentialSegment_State")
#define CountForwardMaxPosTempKey ("CountForwardMaxPosTempKey")


namespace kernel {
    SequentialKernel::SequentialKernel(
            const std::unique_ptr<kernel::KernelBuilder> &iBuilder,
            std::string &&kernelName,
            std::vector<Binding> &&stream_inputs,
            std::vector<Binding> &&stream_outputs,
            std::vector<Binding> &&scalar_parameters,
            std::vector<Binding> &&scalar_outputs,
            std::vector<Binding> &&internal_scalars) :
            MultiBlockKernel(std::move(kernelName), std::move(stream_inputs), std::move(stream_outputs),
                             std::move(scalar_parameters), std::move(scalar_outputs), std::move(internal_scalars)) {
        addScalar(iBuilder->getSizeTy(), SequentialSegmentStateKey);
        addScalar(iBuilder->getSizeTy(), CountForwardMaxPosTempKey);
        addScalar(iBuilder->getSizeTy(), "tempClear");

    }


    void SequentialKernel::recordCountForwardTempMaxPos(const std::unique_ptr<KernelBuilder> &iBuilder, Value *maxPos) {
        if (maxPos) {
            iBuilder->setScalarField(CountForwardMaxPosTempKey, maxPos);
        }
    }

    Value *SequentialKernel::restoreCountForwardTempMaxPos(const std::unique_ptr<KernelBuilder> &iBuilder,
                                                           Value *currentMaxPos) {
        if (currentMaxPos) {
            return iBuilder->getScalarField(CountForwardMaxPosTempKey);
        }
        return NULL;
    }

    void SequentialKernel::generateMultiBlockLogic(const std::unique_ptr<KernelBuilder> &iBuilder,
                                                   Value *const numOfStrides) {
        BasicBlock *entryBlock = iBuilder->GetInsertBlock();
//        iBuilder->CallPrintInt("entry", iBuilder->getSize(1));
//        iBuilder->CallPrintInt("available", iBuilder->getAvailableItemCount("byteStream"));

        // AfterEntryBlock will be the entry block of subclass if it is initial state
        BasicBlock *afterEntryBlock = iBuilder->CreateBasicBlock("afterEntryBlock");
        this->exitBlock = iBuilder->CreateBasicBlock("exitBlock");

        this->stateBlocks.push_back(afterEntryBlock); // index 0 will be initial state
        iBuilder->SetInsertPoint(afterEntryBlock);
        this->generateDoSequentialSegmentMethod(iBuilder);

        iBuilder->CreateBr(this->exitBlock);
        iBuilder->SetInsertPoint(this->exitBlock);


        iBuilder->SetInsertPoint(entryBlock);


        // Create Indirect Branch
        std::vector<Constant *> blockAddressVector = std::vector<Constant *>();
        for (BasicBlock *bb : this->stateBlocks) {
            blockAddressVector.push_back(BlockAddress::get(bb));
        }
        Constant *labels = ConstantVector::get(blockAddressVector);

        Value *target = iBuilder->CreateExtractElement(labels, iBuilder->getScalarField(SequentialSegmentStateKey));
        IndirectBrInst *indirectBr = iBuilder->CreateIndirectBr(target);
        for (BasicBlock *bb : this->stateBlocks) {
            indirectBr->addDestination(bb);
        }

        iBuilder->SetInsertPoint(this->exitBlock);
    }

    void SequentialKernel::generateDoSequentialSegmentMethod(const std::unique_ptr<KernelBuilder> &iBuilder) {
        // Will be override by subclass
    }

    // Cursor
    std::string SequentialKernel::generateCursorFullname(std::string cursorName) {
        return "Cursor_" + cursorName;
    }

    void SequentialKernel::initBufferCursor(const std::unique_ptr<KernelBuilder> &iBuilder,
                                            std::vector<std::string> cursorNames) {
        for (std::string name : cursorNames) {
            addScalar(iBuilder->getSizeTy(), this->generateCursorFullname(name));
        }
    }

    Value *SequentialKernel::getCursorValue(const std::unique_ptr<KernelBuilder> &iBuilder, std::string cursorName) {
        return iBuilder->getScalarField(this->generateCursorFullname(cursorName));
    }

    void SequentialKernel::setCursorValue(const std::unique_ptr<KernelBuilder> &iBuilder, std::string cursorName,
                                          Value *value) {
        iBuilder->setScalarField(this->generateCursorFullname(cursorName), value);
    }

    void SequentialKernel::advanceCursor(const std::unique_ptr<KernelBuilder> &iBuilder, std::string cursorName,
                                         llvm::Value *nums) {
        std::string fullname = this->generateCursorFullname(cursorName);
        Value *cursorValue = iBuilder->getScalarField(fullname);
        cursorValue = iBuilder->CreateAdd(cursorValue, nums);
        iBuilder->setScalarField(fullname, cursorValue);
    }

    void SequentialKernel::advanceCursorUntilPos(const std::unique_ptr<KernelBuilder> &iBuilder, std::string cursorName,
                                                 llvm::Value *position) {
        std::string fullname = this->generateCursorFullname(cursorName);
        Value *cursorValue = iBuilder->getScalarField(fullname);
        iBuilder->CreateAssert(iBuilder->CreateICmpSLE(cursorValue, position),
                               cursorName + " Cursor can only move forward");
        iBuilder->setScalarField(fullname, position);
    }


    // forwardBits, packEnd, exceedAvailable
    std::pair<llvm::Value *, std::pair<llvm::Value *, llvm::Value *>>
    SequentialKernel::genereateCountForwardBitsOnePack(
            const std::unique_ptr<KernelBuilder> &iBuilder,
            std::string inputStreamBufferName,
            llvm::Value *cursorValue,
            bool isZero
    ) {
        size_t bufferSize = this->getInputBufferSize(iBuilder, inputStreamBufferName);
        Value *bufferOffsetMask = iBuilder->getSize(bufferSize - 1);

        Value *actualBufferOffset = iBuilder->CreateAnd(bufferOffsetMask, cursorValue);

        Value *packIndex = iBuilder->CreateLShr(actualBufferOffset, iBuilder->getSize(std::log2(64)));

        Value *countStartBitIndex = iBuilder->CreateAnd(actualBufferOffset, iBuilder->getSize(64 - 1));

        Value *inputStreamPtr = iBuilder->getInputStreamBlockPtr(inputStreamBufferName, iBuilder->getInt32(0));
        inputStreamPtr = iBuilder->CreatePointerCast(inputStreamPtr, iBuilder->getInt64Ty()->getPointerTo());
        Value *packData = iBuilder->CreateLoad(iBuilder->CreateGEP(inputStreamPtr, packIndex));


        packData = iBuilder->CreateLShr(packData, countStartBitIndex);

        if (!isZero) {
            packData = iBuilder->CreateNot(packData);
        }
        Value *forwardZeroCount = iBuilder->CreateCountForwardZeroes(packData);


        Value *isEndOfPack = iBuilder->CreateICmpUGE(iBuilder->CreateAdd(countStartBitIndex, forwardZeroCount),
                                                     iBuilder->getSize(64));
        forwardZeroCount = iBuilder->CreateSelect(
                isEndOfPack,
                iBuilder->CreateSub(iBuilder->getSize(64), countStartBitIndex),
                forwardZeroCount
        );

        Value *newCursorValue = iBuilder->CreateAdd(cursorValue, forwardZeroCount);
        Value *itemTotal = iBuilder->CreateAdd(iBuilder->getAvailableItemCount(inputStreamBufferName),
                                               iBuilder->getProcessedItemCount(inputStreamBufferName));

        Value *isExceedAvailable = iBuilder->CreateICmpUGE(newCursorValue, itemTotal);

        newCursorValue = iBuilder->CreateSelect(isExceedAvailable, itemTotal, newCursorValue);

//        Value* isNotFinished = iBuilder->CreateOr(isEndOfPack, isExceedAvailable);
//        Value* isFinished = iBuilder->CreateNot(isNotFinished);
        return std::make_pair(iBuilder->CreateSub(newCursorValue, cursorValue),
                              make_pair(isEndOfPack, isExceedAvailable));
    };

    // pair<forwardZeros, isFinished>
    std::pair<llvm::Value *, llvm::Value *> SequentialKernel::generateCountForwardBits(
            const std::unique_ptr<KernelBuilder> &iBuilder,
            std::string inputStreamBufferName,
            llvm::Value *cursorValue,
            bool isZero,
            llvm::Value *maxPos
    ) {
        BasicBlock *entryBlock = iBuilder->CreateBasicBlock("count_forward_bit_entry");
        iBuilder->CreateBr(entryBlock);
        iBuilder->SetInsertPoint(entryBlock);

        BasicBlock *exitBlock = iBuilder->CreateBasicBlock("count_forward_bit_exit");


        auto onePackResult = genereateCountForwardBitsOnePack(iBuilder, inputStreamBufferName, cursorValue, isZero);

        Value *forwardCount = onePackResult.first;
        Value *isEndOfPack = onePackResult.second.first;
        Value *isExceedAvailable = onePackResult.second.second;
        Value *newCursorValue = iBuilder->CreateAdd(cursorValue, forwardCount);

        Value *isNotFinished = iBuilder->CreateOr(isEndOfPack, isExceedAvailable);
        Value *isFinished = iBuilder->CreateNot(isNotFinished);

        if (maxPos) {
            Value *reachMaxPos = iBuilder->CreateICmpUGE(newCursorValue, maxPos);
            isFinished = iBuilder->CreateSelect(
                    reachMaxPos,
                    iBuilder->getInt1(true),
                    isFinished
            );
            newCursorValue = iBuilder->CreateSelect(
                    reachMaxPos,
                    maxPos,
                    newCursorValue
            );

        }

        iBuilder->CreateBr(exitBlock);
        iBuilder->SetInsertPoint(exitBlock);

        return std::make_pair(iBuilder->CreateSub(newCursorValue, cursorValue), isFinished);

    };

    std::pair<llvm::Value *, llvm::Value *>
    SequentialKernel::generateCountForwardOnes(const unique_ptr<KernelBuilder> &iBuilder, string inputStreamBufferName,
                                               Value *beginOffset, Value *maxPos) {
        return this->generateCountForwardBits(iBuilder, inputStreamBufferName, beginOffset, false, maxPos);
    };

    std::pair<llvm::Value *, llvm::Value *>
    SequentialKernel::generateCountForwardZeros(const unique_ptr<KernelBuilder> &iBuilder, string inputStreamBufferName,
                                                Value *beginOffset, Value *maxPos) {
        return this->generateCountForwardBits(iBuilder, inputStreamBufferName, beginOffset, true, maxPos);
    }


    BasicBlock *
    SequentialKernel::advanceCursorUntilNextOne(const std::unique_ptr<KernelBuilder> &iBuilder, std::string cursorName,
                                                std::string inputStreamBufferName, Value *maxPos) {
        BasicBlock *entryBlock = iBuilder->CreateBasicBlock("advance_cursor_until_next_one_entry");

        this->recordCountForwardTempMaxPos(iBuilder, maxPos);

        iBuilder->CreateBr(entryBlock);
        iBuilder->SetInsertPoint(entryBlock);


        // StateIndex will be increased in waitCursorUntilInputAvailable
        this->waitCursorUntilInputAvailable(iBuilder, cursorName, inputStreamBufferName);

        BasicBlock *countForwareZeroBlock = iBuilder->CreateBasicBlock("advance_cursor_until_next_one_count_block");
        BasicBlock *exitBlock = iBuilder->CreateBasicBlock("advance_cursor_until_next_one_exit_block");

        iBuilder->CreateBr(countForwareZeroBlock);
        iBuilder->SetInsertPoint(countForwareZeroBlock);

        Value *cursorValue = this->getCursorValue(iBuilder, cursorName);

        maxPos = this->restoreCountForwardTempMaxPos(iBuilder, maxPos);

        auto retValue = this->generateCountForwardZeros(iBuilder, inputStreamBufferName, cursorValue, maxPos);

        cursorValue = iBuilder->CreateAdd(cursorValue, retValue.first);
        Value *isFinished = retValue.second;


        //TODO Add additional handle for isFinish (is isFinish === false, the next pack will always start from index 0), avoid using waitCursorUntilInputAvailable in the second loop

        this->setCursorValue(iBuilder, cursorName, cursorValue);

        iBuilder->CreateCondBr(isFinished, exitBlock, entryBlock);
        //TODO add index bits for count forward zeros and ones
        iBuilder->SetInsertPoint(exitBlock);
        return exitBlock;
    }


    BasicBlock *SequentialKernel::advanceCursorUntilNextZero(
            const std::unique_ptr<KernelBuilder> &iBuilder, std::string cursorName, std::string inputStreamBufferName,
            Value *maxPos) {
        BasicBlock *entryBlock = iBuilder->CreateBasicBlock("advance_cursor_until_next_zero_entry");

        this->recordCountForwardTempMaxPos(iBuilder, maxPos);

        iBuilder->CreateBr(entryBlock);
        iBuilder->SetInsertPoint(entryBlock);

        this->waitCursorUntilInputAvailable(iBuilder, cursorName, inputStreamBufferName);

        BasicBlock *countForwareOneBlock = iBuilder->CreateBasicBlock("advance_cursor_until_next_zero_count_block");
        BasicBlock *exitBlock = iBuilder->CreateBasicBlock("advance_cursor_until_next_zero_exit_block");

        iBuilder->CreateBr(countForwareOneBlock);
        iBuilder->SetInsertPoint(countForwareOneBlock);

        Value *cursorValue = this->getCursorValue(iBuilder, cursorName);

        maxPos = this->restoreCountForwardTempMaxPos(iBuilder, maxPos);

        auto retValue = this->generateCountForwardOnes(iBuilder, inputStreamBufferName, cursorValue, maxPos);

        this->advanceCursor(iBuilder, cursorName, retValue.first);

        Value *isFinished = retValue.second;

        iBuilder->CreateCondBr(isFinished, exitBlock, entryBlock);
        //TODO add index bits for count forward zeros and ones
        iBuilder->SetInsertPoint(exitBlock);
        return exitBlock;
    }


    BasicBlock *SequentialKernel::waitCursorUntilInputAvailable(const std::unique_ptr<KernelBuilder> &iBuilder,
                                                                std::string cursorName,
                                                                std::string inputStreamBufferName) {
//        BasicBlock* entryBlock = iBuilder->GetInsertBlock();
        Value *nextStateValue = iBuilder->getSize(this->stateBlocks.size());

        BasicBlock *restoreBlock = iBuilder->CreateBasicBlock("wait_cursor_until_input_available_restore");
        BasicBlock *continueBlock = iBuilder->CreateBasicBlock("wait_cursor_until_input_available_continue");

        this->stateBlocks.push_back(restoreBlock);

        iBuilder->CreateBr(restoreBlock);

        iBuilder->SetInsertPoint(restoreBlock);

        Value *cursorValue = this->getCursorValue(iBuilder, cursorName);
        Value *itemTotal = iBuilder->CreateAdd(iBuilder->getAvailableItemCount(inputStreamBufferName),
                                               iBuilder->getProcessedItemCount(inputStreamBufferName));
        Value *isAvailable = iBuilder->CreateICmpULT(cursorValue, itemTotal);

        Value *nextState = iBuilder->CreateSelect(isAvailable, iBuilder->getSize(0), nextStateValue);
        iBuilder->setScalarField(SequentialSegmentStateKey, nextState);

        iBuilder->CreateCondBr(isAvailable, continueBlock, this->exitBlock);

        iBuilder->SetInsertPoint(continueBlock);

        return continueBlock;

    }

    size_t SequentialKernel::getInputBufferSize(const unique_ptr<KernelBuilder> &iBuilder, string bufferName) {
        //TODO codegen::BlockSize == iBuilder->getStride() ?
        return this->getInputStreamSetBuffer(bufferName)->getBufferBlocks() * iBuilder->getStride();
    }

    size_t SequentialKernel::getOutputBufferSize(const unique_ptr<KernelBuilder> &iBuilder, string bufferName) {
        return this->getOutputStreamSetBuffer(bufferName)->getBufferBlocks() * iBuilder->getStride();
    }

    Value *SequentialKernel::offsetToPackBaseOffset(const unique_ptr<KernelBuilder> &iBuilder, Value *offset) {
        return iBuilder->CreateShl(
                this->offsetToPackIndex(iBuilder, offset),
                iBuilder->getSize(std::log2(64))
        );
    }

    Value *SequentialKernel::offsetToPackIndex(const unique_ptr<KernelBuilder> &iBuilder, Value *offset) {
        return iBuilder->CreateLShr(offset, iBuilder->getSize(std::log2(64)));
    }

    Value *SequentialKernel::offsetToPackOffset(const unique_ptr<KernelBuilder> &iBuilder, Value *offset) {
        return iBuilder->CreateAnd(offset, iBuilder->getSize(64 - 1));
    }

    Value *
    SequentialKernel::offsetToActualBufferOffset(const unique_ptr<KernelBuilder> &iBuilder, string inputBufferName,
                                                 Value *offset) {
        size_t bufferSize = this->getInputBufferSize(iBuilder, inputBufferName);
        Value *bufferOffsetMask = iBuilder->getSize(bufferSize - 1);
        return iBuilder->CreateAnd(bufferOffsetMask, offset);
    }

    Value *
    SequentialKernel::generateLoadCircularInputPack(const unique_ptr<KernelBuilder> &iBuilder, string inputBufferName,
                                                    Value *offset) {
        Value *actualBufferOffset = this->offsetToActualBufferOffset(iBuilder, inputBufferName, offset);
        Value *packIndex = this->offsetToPackIndex(iBuilder, actualBufferOffset);
//        Value* countStartBitIndex = this->offsetToPackOffset(iBuilder, actualBufferOffset);


        Value *inputStreamPtr = iBuilder->getInputStreamBlockPtr(inputBufferName, iBuilder->getInt32(0));
        inputStreamPtr = iBuilder->CreatePointerCast(inputStreamPtr, iBuilder->getInt64Ty()->getPointerTo());
        return iBuilder->CreateLoad(iBuilder->CreateGEP(inputStreamPtr, packIndex));

//        packData = iBuilder->CreateLShr(packData, countStartBitIndex);

    }

    Value *
    SequentialKernel::generateLoadCircularInput(const unique_ptr<KernelBuilder> &iBuilder, string inputBufferName,
                                                Value *offset, Type *pointerType) {
        size_t inputSize = this->getInputBufferSize(iBuilder, inputBufferName);
        Value *offsetMask = iBuilder->getSize(inputSize - 1);
        Value *maskedOffset = iBuilder->CreateAnd(offsetMask, offset);

        Value *inputBufferPtr = iBuilder->getRawInputPointer(inputBufferName, iBuilder->getSize(0));

        inputBufferPtr = iBuilder->CreatePointerCast(inputBufferPtr, pointerType);
        return iBuilder->CreateLoad(iBuilder->CreateGEP(inputBufferPtr, maskedOffset));
    }

    Value *SequentialKernel::generateLoadSourceInputByte(const std::unique_ptr<KernelBuilder> &iBuilder,
                                                         string sourceBufferName, Value *offset) {
        Value *blockStartPtr = iBuilder->CreatePointerCast(
                iBuilder->getInputStreamBlockPtr(sourceBufferName, iBuilder->getInt32(0)),
                iBuilder->getInt8PtrTy()
        );
        Value *ptr = iBuilder->CreateGEP(blockStartPtr, offset);


        return iBuilder->CreateLoad(ptr);
    }


    void
    SequentialKernel::generateStoreCircularOutput(const unique_ptr<KernelBuilder> &iBuilder, string outputBufferName,
                                                  Type *pointerType, Value *value) {
        Value *offset = iBuilder->getProducedItemCount(outputBufferName);

        size_t inputSize = this->getOutputBufferSize(iBuilder, outputBufferName);
        Value *offsetMask = iBuilder->getSize(inputSize - 1);
        Value *maskedOffset = iBuilder->CreateAnd(offsetMask, offset);

        Value *outputBufferPtr = iBuilder->getRawOutputPointer(outputBufferName, iBuilder->getSize(0));

        outputBufferPtr = iBuilder->CreatePointerCast(outputBufferPtr, pointerType);
        iBuilder->CreateStore(value, iBuilder->CreateGEP(outputBufferPtr, maskedOffset));

        offset = iBuilder->CreateAdd(offset, iBuilder->getSize(1));
        iBuilder->setProducedItemCount(outputBufferName, offset);
    }

    void
    SequentialKernel::increaseScalarField(const std::unique_ptr<KernelBuilder> &iBuilder, const std::string &fieldName,
                                          llvm::Value *value) {
        Value *fieldValue = iBuilder->getScalarField(fieldName);
        fieldValue = iBuilder->CreateAdd(fieldValue, value);
        iBuilder->setScalarField(fieldName, fieldValue);
    }


    void SequentialKernel::markCircularOutputBitstreamOnePack(const std::unique_ptr<KernelBuilder> &iBuilder,
                                                              const std::string &bitstreamName, llvm::Value *start,
                                                              llvm::Value *end, bool isOne) {
        Value *outputBasePtr = iBuilder->getRawOutputPointer(bitstreamName, iBuilder->getSize(0));

        outputBasePtr = iBuilder->CreatePointerCast(outputBasePtr, iBuilder->getInt64Ty()->getPointerTo());

        size_t outputBufferSize = this->getOutputBufferSize(iBuilder, bitstreamName);
        Value *outputMask = iBuilder->getSize(outputBufferSize / 64 - 1);


        Value *startOffset = iBuilder->CreateLShr(start, iBuilder->getSize(std::log2(64)), "startOffset");
        Value *curOffset = startOffset;


        Value *outputLowestBitValue = iBuilder->CreateSelect(
                iBuilder->CreateICmpULE(
                        iBuilder->CreateShl(curOffset, std::log2(64)),
                        start
                ),
                iBuilder->CreateShl(iBuilder->getSize(1), iBuilder->CreateAnd(start, iBuilder->getSize(64 - 1))),
                iBuilder->getSize(1)
        );

        Value *outputHighestBitValue = iBuilder->CreateShl(
                iBuilder->getSize(1),
                iBuilder->CreateAnd(end, iBuilder->getSize(64 - 1))
        );


        Value *bitMask = iBuilder->CreateSub(
                outputHighestBitValue,
                outputLowestBitValue
        );

        if (!isOne) {
            bitMask = iBuilder->CreateNot(bitMask);
        }
    }

    // Assume we have enough output buffer
    llvm::BasicBlock *SequentialKernel::markCircularOutputBitstream(const std::unique_ptr<KernelBuilder> &iBuilder,
                                                                    const std::string &bitstreamName,
                                                                    llvm::Value *start, llvm::Value *end, bool isOne,
                                                                    bool setProduced) {
        BasicBlock *entryBlock = iBuilder->GetInsertBlock();

        Value *outputBasePtr = iBuilder->getRawOutputPointer(bitstreamName, iBuilder->getSize(0));

        outputBasePtr = iBuilder->CreatePointerCast(outputBasePtr, iBuilder->getInt64Ty()->getPointerTo());

        size_t outputBufferSize = this->getOutputBufferSize(iBuilder, bitstreamName);
        Value *outputMask = iBuilder->getSize(outputBufferSize / 64 - 1);

        BasicBlock *conBlock = iBuilder->CreateBasicBlock("mark_bit_one_con");
        BasicBlock *bodyBlock = iBuilder->CreateBasicBlock("mark_bit_one_body");
        BasicBlock *exitBlock = iBuilder->CreateBasicBlock("mark_bit_one_exit");

        Value *startOffset = iBuilder->CreateLShr(start, iBuilder->getSize(std::log2(64)), "startOffset");

        iBuilder->CreateBr(conBlock);

        // Con
        iBuilder->SetInsertPoint(conBlock);


        PHINode *curOffset = iBuilder->CreatePHI(iBuilder->getSizeTy(), 2);
        curOffset->addIncoming(startOffset, entryBlock);

        iBuilder->CreateCondBr(
                iBuilder->CreateICmpULT(iBuilder->CreateShl(curOffset, std::log2(64)), end),
                bodyBlock,
                exitBlock
        );

        // Body
        iBuilder->SetInsertPoint(bodyBlock);
        Value *maskedOffset = iBuilder->CreateAnd(curOffset, outputMask);

        Value *outputLowestBitValue = iBuilder->CreateSelect(
                iBuilder->CreateICmpULE(
                        iBuilder->CreateShl(curOffset, std::log2(64)),
                        start
                ),
                iBuilder->CreateShl(iBuilder->getSize(1), iBuilder->CreateAnd(start, iBuilder->getSize(64 - 1))),
                iBuilder->getSize(1)
        );

        Value *hasNotReachEnd = iBuilder->CreateICmpULE(
                iBuilder->CreateShl(iBuilder->CreateAdd(curOffset, iBuilder->getSize(1)), std::log2(64)),
                end
        );
        Value *producedItemsCount = iBuilder->CreateSelect(
                hasNotReachEnd,
                iBuilder->CreateShl(iBuilder->CreateAdd(curOffset, iBuilder->getSize(1)), std::log2(64)),
                end
        );

        Value *outputHighestBitValue = iBuilder->CreateSelect(
                hasNotReachEnd,
                iBuilder->getSize(0),
                iBuilder->CreateShl(
                        iBuilder->getSize(1),
                        iBuilder->CreateAnd(end, iBuilder->getSize(64 - 1))
                )
        );


        Value *bitMask = iBuilder->CreateSub(
                outputHighestBitValue,
                outputLowestBitValue
        );

        if (!isOne) {
            bitMask = iBuilder->CreateNot(bitMask);
        }

        Value *targetPtr = iBuilder->CreateGEP(outputBasePtr, maskedOffset);
        Value *oldValue = iBuilder->CreateLoad(targetPtr);
        Value *newValue = NULL;
        if (isOne) {
            newValue = iBuilder->CreateOr(oldValue, bitMask);
        } else {
            newValue = iBuilder->CreateAnd(oldValue, bitMask);
        }
        iBuilder->CreateStore(
                newValue,
                targetPtr
        );
        if (setProduced) {
            iBuilder->setProducedItemCount(bitstreamName, producedItemsCount);
        }

        curOffset->addIncoming(iBuilder->CreateAdd(curOffset, iBuilder->getSize(1)), bodyBlock);
        iBuilder->CreateBr(conBlock);

        // Exit
        iBuilder->SetInsertPoint(exitBlock);
        return exitBlock;
    }
}