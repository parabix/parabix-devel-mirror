
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
#define ModifyInputTempKey ("ModifyInput_Temp")
#define MemCpyUntilZeroCopyOffsetTempKey ("MemCpyUntilZeroCopyOffsetTempKey")
#define CountForwardMaxPosTempKey ("CountForwardMaxPosTempKey")



namespace kernel {
    SequentialKernel::SequentialKernel(
            const std::unique_ptr<kernel::KernelBuilder> & iBuilder,
            std::string && kernelName,
            std::vector<Binding> && stream_inputs,
            std::vector<Binding> && stream_outputs,
            std::vector<Binding> && scalar_parameters,
            std::vector<Binding> && scalar_outputs,
            std::vector<Binding> && internal_scalars):
            MultiBlockKernel(std::move(kernelName), std::move(stream_inputs), std::move(stream_outputs), std::move(scalar_parameters), std::move(scalar_outputs), std::move(internal_scalars)) {
        addScalar(iBuilder->getSizeTy(), SequentialSegmentStateKey);
        addScalar(iBuilder->getInt1Ty(), ModifyInputTempKey);
        addScalar(iBuilder->getSizeTy(), MemCpyUntilZeroCopyOffsetTempKey);
        addScalar(iBuilder->getSizeTy(), CountForwardMaxPosTempKey);
        addScalar(iBuilder->getSizeTy(), "tempClear");

    }


    void SequentialKernel::recordCountForwardTempMaxPos(const std::unique_ptr<KernelBuilder> &iBuilder, Value* maxPos) {
        if (maxPos) {
            iBuilder->setScalarField(CountForwardMaxPosTempKey, maxPos);
        }
    }
    Value* SequentialKernel::restoreCountForwardTempMaxPos(const std::unique_ptr<KernelBuilder> &iBuilder, Value* currentMaxPos) {
        if (currentMaxPos) {
            return iBuilder->getScalarField(CountForwardMaxPosTempKey);
        }
        return NULL;
    }

    void SequentialKernel::generateMultiBlockLogic(const std::unique_ptr<KernelBuilder> &iBuilder, Value * const numOfStrides) {
        BasicBlock* entryBlock = iBuilder->GetInsertBlock();
//        iBuilder->CallPrintInt("entry", iBuilder->getSize(1));
//        iBuilder->CallPrintInt("available", iBuilder->getAvailableItemCount("byteStream"));

        // AfterEntryBlock will be the entry block of subclass if it is initial state
        BasicBlock* afterEntryBlock = iBuilder->CreateBasicBlock("afterEntryBlock");
        this->exitBlock = iBuilder->CreateBasicBlock("exitBlock");

        this->stateBlocks.push_back(afterEntryBlock); // index 0 will be initial state
        iBuilder->SetInsertPoint(afterEntryBlock);
        this->generateDoSequentialSegmentMethod(iBuilder);

        iBuilder->CreateBr(this->exitBlock);
        iBuilder->SetInsertPoint(this->exitBlock);


        iBuilder->SetInsertPoint(entryBlock);
        this->generateBuildIndexBits(iBuilder);
        this->generateClearBuffer(iBuilder);


        // Create Indirect Branch
        std::vector<Constant*> blockAddressVector = std::vector<Constant*>();
        for (BasicBlock* bb : this->stateBlocks) {
            blockAddressVector.push_back(BlockAddress::get(bb));
        }
        Constant * labels = ConstantVector::get(blockAddressVector);

        Value * target = iBuilder->CreateExtractElement(labels, iBuilder->getScalarField(SequentialSegmentStateKey));
        IndirectBrInst * indirectBr = iBuilder->CreateIndirectBr(target);
        for (BasicBlock* bb : this->stateBlocks) {
            indirectBr->addDestination(bb);
        }

        iBuilder->SetInsertPoint(this->exitBlock);
    }

    bool SequentialKernel::hasIndexBits(const std::string& streamName) {
        return inputStreamIndexMap.find(streamName) != inputStreamIndexMap.end();
    }

    void SequentialKernel::configOutputBufferToBeClear(const std::map<string, string>& clearMap) {
        this->clearBufferMap = clearMap;
    }

    void SequentialKernel::generateClearBuffer(const std::unique_ptr<KernelBuilder> &iBuilder) {
        BasicBlock* entryBlock = iBuilder->CreateBasicBlock("clear_buffer_entry");
        BasicBlock* exitBlock = iBuilder->CreateBasicBlock("clear_buffer_exit");

        iBuilder->CreateBr(entryBlock);
        iBuilder->SetInsertPoint(entryBlock);

        for (auto iter = this->clearBufferMap.begin(); iter != this->clearBufferMap.end(); iter++) {
            string inputName = iter->first;
            string outputName = iter->second;

            BasicBlock* clearEntry = iBuilder->CreateBasicBlock("clear_" + outputName + "_entry");
            BasicBlock* clearCon = iBuilder->CreateBasicBlock("clear_" + outputName + "_con");
            BasicBlock* clearBody = iBuilder->CreateBasicBlock("clear_" + outputName + "_body");
            BasicBlock* clearExit = iBuilder->CreateBasicBlock("clear_" + outputName + "_exit");

            iBuilder->CreateBr(clearEntry);
            iBuilder->SetInsertPoint(clearEntry);

            Value* itemProduced = iBuilder->getScalarField("tempClear");
            Value* itemsTotal = iBuilder->CreateAdd(iBuilder->getAvailableItemCount(inputName), iBuilder->getProcessedItemCount(inputName));
            iBuilder->setScalarField("tempClear", itemsTotal);

            size_t outputSize = this->getOutputBufferSize(iBuilder, outputName);
            size_t outputPackNum = outputSize / 64;

            Value* startPackIndex = iBuilder->CreateLShr(itemProduced, iBuilder->getSize(std::log2(64)));
            Value* endPackIndex = iBuilder->CreateLShr(itemsTotal, iBuilder->getSize(std::log2(64)));

            iBuilder->CreateBr(clearCon);
            iBuilder->SetInsertPoint(clearCon);

            PHINode* currentPackIndex = iBuilder->CreatePHI(iBuilder->getSizeTy(), 2);
            currentPackIndex->addIncoming(startPackIndex, clearEntry);
            iBuilder->CreateCondBr(iBuilder->CreateICmpULT(currentPackIndex, endPackIndex), clearBody, clearExit);

            iBuilder->SetInsertPoint(clearBody);
            Value* outputBasePtr = iBuilder->getRawOutputPointer(outputName, iBuilder->getSize(0));
            outputBasePtr = iBuilder->CreatePointerCast(outputBasePtr, iBuilder->getInt64Ty()->getPointerTo());
            Value* maskedPackIndex = iBuilder->CreateAnd(currentPackIndex, iBuilder->getSize(outputPackNum - 1));
            iBuilder->CreateStore(iBuilder->getInt64(0), iBuilder->CreateGEP(outputBasePtr, maskedPackIndex));

            currentPackIndex->addIncoming(iBuilder->CreateAdd(currentPackIndex, iBuilder->getSize(1)), clearBody);
            iBuilder->CreateBr(clearCon);

            iBuilder->SetInsertPoint(clearExit);
        }
        iBuilder->CreateBr(exitBlock);
        iBuilder->SetInsertPoint(exitBlock);

    }

    void SequentialKernel::generateBuildIndexBits(const std::unique_ptr<KernelBuilder> &iBuilder) {
//        iBuilder->CallPrintInt("entry", iBuilder->getSize(0));
        BasicBlock* entryBlock = iBuilder->CreateBasicBlock("build_index_bits_entry");
        BasicBlock* exitBlock = iBuilder->CreateBasicBlock("build_index_bits_exit");

        iBuilder->CreateBr(entryBlock);

        // Entry Block
        iBuilder->SetInsertPoint(entryBlock);

        for (auto iter = inputStreamIndexMap.begin(); iter != inputStreamIndexMap.end(); iter++) {
            string streamName = iter->first;
//            size_t indexArraySize = iter->second;

            BasicBlock* indexUpdateEntryBlock = iBuilder->CreateBasicBlock(streamName + "_index_update_entry");
            iBuilder->CreateBr(indexUpdateEntryBlock);

            iBuilder->SetInsertPoint(indexUpdateEntryBlock);

            Value* previousItemsAvailable = iBuilder->getScalarField(this->generateInputPreviousAvailableName(streamName));
            Value* itemsTotal = iBuilder->CreateAdd(iBuilder->getAvailableItemCount(streamName), iBuilder->getProcessedItemCount(streamName));
            iBuilder->setScalarField(this->generateInputPreviousAvailableName(streamName), itemsTotal);

            size_t bufferSize = this->getInputBufferSize(iBuilder, streamName);
            size_t indexBitsCount = bufferSize / 64;

            Value* indexBitToBeUpdateStart = iBuilder->CreateLShr(previousItemsAvailable, std::log2(64));
            Value* indexBitToBeUpdateEnd = iBuilder->CreateLShr(iBuilder->CreateAdd(itemsTotal, iBuilder->getSize(63)), std::log2(64));


            BasicBlock* updateLoopCon = iBuilder->CreateBasicBlock(streamName + "_index_update_loop_con");
            BasicBlock* updateLoopBody = iBuilder->CreateBasicBlock(streamName + "_index_update_loop_body");
            BasicBlock* updateLoopFinal = iBuilder->CreateBasicBlock(streamName + "_index_update_loop_final");
            BasicBlock* updateLoopExit = iBuilder->CreateBasicBlock(streamName + "_index_update_loop_exit");

            iBuilder->CreateBr(updateLoopCon);


            // Update Loop Con
            iBuilder->SetInsertPoint(updateLoopCon);
            PHINode* currentUpdateBitIndex = iBuilder->CreatePHI(iBuilder->getSizeTy(), 2);
            currentUpdateBitIndex->addIncoming(indexBitToBeUpdateStart, indexUpdateEntryBlock);

            iBuilder->CreateCondBr(
                    iBuilder->CreateICmpULT(currentUpdateBitIndex, indexBitToBeUpdateEnd),
                    updateLoopBody,
                    updateLoopExit
            );

            // Update Loop Body
            iBuilder->SetInsertPoint(updateLoopBody);
            Value* bitIndex = iBuilder->CreateURem(currentUpdateBitIndex, iBuilder->getSize(indexBitsCount)); // TODO replace with and
            Value* arrayIndex = iBuilder->CreateLShr(
                    bitIndex,
                    iBuilder->getSize(std::log2(64)));
            Value* indexIndex = iBuilder->CreateAnd(bitIndex, iBuilder->getSize(63));

            Value* inputStreamPtr = iBuilder->getRawInputPointer(streamName, iBuilder->getSize(0));
            inputStreamPtr = iBuilder->CreatePointerCast(inputStreamPtr, iBuilder->getInt64Ty()->getPointerTo());

            Value* targetInputValue = iBuilder->CreateLoad(iBuilder->CreateGEP(inputStreamPtr, bitIndex));

            // handle bit 0 index
            Value* index0OldValue = iBuilder->CreateExtractElement(
                    iBuilder->getScalarField(this->generateInputZeroIndexName(streamName)),
                    arrayIndex
            );

            Value* newBit0Value = iBuilder->CreateNot(
                    iBuilder->CreateICmpEQ(
                            targetInputValue,
                            iBuilder->CreateNot(
                                    iBuilder->getInt64(0x0)
                            )
                    )
            );



            newBit0Value = iBuilder->CreateZExt(newBit0Value, iBuilder->getInt64Ty());

            Value* index0NewValue = index0OldValue;
            index0NewValue = iBuilder->CreateAnd(
                    index0NewValue,
                    iBuilder->CreateNot(
                            iBuilder->CreateShl(
                                    iBuilder->getInt64(1),
                                    indexIndex
                            )
                    )
            );
            index0NewValue = iBuilder->CreateOr(
                    index0NewValue,
                    iBuilder->CreateShl(
                            newBit0Value,
                            indexIndex
                    )
            );
            iBuilder->setScalarField(
                    this->generateInputZeroIndexName(streamName),
                    iBuilder->CreateInsertElement(
                            iBuilder->getScalarField(this->generateInputZeroIndexName(streamName)),
                            index0NewValue,
                            arrayIndex
                    )
            );


            // handle bit 1 index

            Value* index1OldValue = iBuilder->CreateExtractElement(
                    iBuilder->getScalarField(this->generateInputOneIndexName(streamName)),
                    arrayIndex
            );

            Value* newBit1Value = iBuilder->CreateNot(iBuilder->CreateICmpEQ(targetInputValue, iBuilder->getInt64(0)));
            newBit1Value = iBuilder->CreateZExt(newBit1Value, iBuilder->getInt64Ty());

            Value* index1NewValue = index1OldValue;
            index1NewValue = iBuilder->CreateAnd(
                    index1NewValue,
                    iBuilder->CreateNot(
                            iBuilder->CreateShl(
                                    iBuilder->getInt64(1),
                                    indexIndex
                            )
                    )
            );
            index1NewValue = iBuilder->CreateOr(
                    index1NewValue,
                    iBuilder->CreateShl(
                            newBit1Value,
                            indexIndex
                    )
            );

            iBuilder->setScalarField(
                    this->generateInputOneIndexName(streamName),
                    iBuilder->CreateInsertElement(
                            iBuilder->getScalarField(this->generateInputOneIndexName(streamName)),
                            index1NewValue,
                            arrayIndex
                    )
            );

            iBuilder->CreateBr(updateLoopFinal);


            // Update Loop Final
            iBuilder->SetInsertPoint(updateLoopFinal);
            currentUpdateBitIndex->addIncoming(iBuilder->CreateAdd(currentUpdateBitIndex, iBuilder->getSize(1)), updateLoopFinal);
            iBuilder->CreateBr(updateLoopCon);

            //Update Loop Exit
            iBuilder->SetInsertPoint(updateLoopExit);

        }

        iBuilder->CreateBr(exitBlock);
        iBuilder->SetInsertPoint(exitBlock);

    }

    void SequentialKernel::generateDoSequentialSegmentMethod(const std::unique_ptr<KernelBuilder> &iBuilder) {
        // Will be override by subclass
    }

    // Initialize

    // Index
    void SequentialKernel::configIndexBits(const std::unique_ptr<KernelBuilder> &iBuilder, const std::map<std::string, size_t>& inputIndexMap) {
        for (auto iter = inputIndexMap.begin(); iter != inputIndexMap.end(); iter++ ) {
            string inputBufferName = iter->first;
            size_t indexBitNum = iter->second; // blockSize = size / iBuilder->getStride()
            size_t indexArraySize = ((indexBitNum * iBuilder->getStride() / 64 ) + 63) / 64;
            inputStreamIndexMap.insert(make_pair(inputBufferName, indexArraySize));

            this->addScalar(VectorType::get(iBuilder->getInt64Ty(), indexArraySize), generateInputZeroIndexName(inputBufferName));
            this->addScalar(VectorType::get(iBuilder->getInt64Ty(), indexArraySize), generateInputOneIndexName(inputBufferName));
            this->addScalar(iBuilder->getSizeTy(), generateInputPreviousAvailableName(inputBufferName));
        }

    }
    inline string SequentialKernel::generateInputZeroIndexName(string inputStreamName) {
        return "index_" + inputStreamName + "_zero_index";
    }
    inline string SequentialKernel::generateInputOneIndexName(string inputStreamName) {
        return "index_" + inputStreamName + "_one_index";
    }

    inline string SequentialKernel::generateInputPreviousAvailableName(std::string inputStreamName) {
        return "index_" + inputStreamName + "_previous_item_available";
    }

    // Cursor
    std::string SequentialKernel::generateCursorFullname(std::string cursorName) {
        return "Cursor_" + cursorName;
    }
    void SequentialKernel::initBufferCursor(const std::unique_ptr<KernelBuilder> &iBuilder, std::vector<std::string> cursorNames) {
        for (std::string name : cursorNames) {
            addScalar(iBuilder->getSizeTy(), this->generateCursorFullname(name));
        }
    }

    Value* SequentialKernel::getCursorValue(const std::unique_ptr<KernelBuilder> &iBuilder, std::string cursorName) {
        return iBuilder->getScalarField(this->generateCursorFullname(cursorName));
    }

    void SequentialKernel::setCursorValue(const std::unique_ptr<KernelBuilder> &iBuilder, std::string cursorName, Value* value) {
        iBuilder->setScalarField(this->generateCursorFullname(cursorName), value);
    }

    void SequentialKernel::advanceCursor(const std::unique_ptr<KernelBuilder> &iBuilder, std::string cursorName, llvm::Value* nums) {
        std::string fullname = this->generateCursorFullname(cursorName);
        Value* cursorValue = iBuilder->getScalarField(fullname);
        cursorValue = iBuilder->CreateAdd(cursorValue, nums);
        iBuilder->setScalarField(fullname, cursorValue);
    }

    void SequentialKernel::advanceCursorUntilPos(const std::unique_ptr<KernelBuilder> &iBuilder, std::string cursorName, llvm::Value* position) {
        std::string fullname = this->generateCursorFullname(cursorName);
        Value* cursorValue = iBuilder->getScalarField(fullname);
        iBuilder->CreateAssert(iBuilder->CreateICmpSLE(cursorValue, position), cursorName + " Cursor can only move forward");
        iBuilder->setScalarField(fullname, position);
    }


    // forwardBits, packEnd, exceedAvailable
    std::pair<llvm::Value*, std::pair<llvm::Value*, llvm::Value*>> SequentialKernel::genereateCountForwardBitsOnePack(
            const std::unique_ptr<KernelBuilder> &iBuilder,
            std::string inputStreamBufferName,
            llvm::Value* cursorValue,
            bool isZero
    ){
        size_t bufferSize = this->getInputBufferSize(iBuilder, inputStreamBufferName);
        Value* bufferOffsetMask = iBuilder->getSize(bufferSize - 1);

        Value* actualBufferOffset = iBuilder->CreateAnd(bufferOffsetMask, cursorValue);

        Value* packIndex = iBuilder->CreateLShr(actualBufferOffset, iBuilder->getSize(std::log2(64)));

        Value* countStartBitIndex = iBuilder->CreateAnd(actualBufferOffset, iBuilder->getSize(64 - 1));

        Value* inputStreamPtr = iBuilder->getInputStreamBlockPtr(inputStreamBufferName, iBuilder->getInt32(0));
        inputStreamPtr = iBuilder->CreatePointerCast(inputStreamPtr, iBuilder->getInt64Ty()->getPointerTo());
        Value* packData = iBuilder->CreateLoad(iBuilder->CreateGEP(inputStreamPtr, packIndex));



        packData = iBuilder->CreateLShr(packData, countStartBitIndex);

        if (!isZero) {
            packData = iBuilder->CreateNot(packData);
        }
        Value* forwardZeroCount = iBuilder->CreateCountForwardZeroes(packData);



        Value* isEndOfPack = iBuilder->CreateICmpUGE(iBuilder->CreateAdd(countStartBitIndex, forwardZeroCount), iBuilder->getSize(64));
        forwardZeroCount = iBuilder->CreateSelect(
                isEndOfPack,
                iBuilder->CreateSub(iBuilder->getSize(64), countStartBitIndex),
                forwardZeroCount
        );

        Value* newCursorValue = iBuilder->CreateAdd(cursorValue, forwardZeroCount);
        Value* itemTotal = iBuilder->CreateAdd(iBuilder->getAvailableItemCount(inputStreamBufferName), iBuilder->getProcessedItemCount(inputStreamBufferName));

        Value* isExceedAvailable = iBuilder->CreateICmpUGE(newCursorValue, itemTotal);

        newCursorValue = iBuilder->CreateSelect(isExceedAvailable, itemTotal, newCursorValue);

//        Value* isNotFinished = iBuilder->CreateOr(isEndOfPack, isExceedAvailable);
//        Value* isFinished = iBuilder->CreateNot(isNotFinished);
        return std::make_pair(iBuilder->CreateSub(newCursorValue, cursorValue), make_pair(isEndOfPack, isExceedAvailable));
    };

    // pair<forwardZeros, isFinished>
    std::pair<llvm::Value*, llvm::Value*> SequentialKernel::generateCountForwardBits(
            const std::unique_ptr<KernelBuilder> &iBuilder,
            std::string inputStreamBufferName,
            llvm::Value* cursorValue,
            bool isZero,
            llvm::Value* maxPos
    ) {
        BasicBlock* entryBlock = iBuilder->CreateBasicBlock("count_forward_bit_entry");
        iBuilder->CreateBr(entryBlock);
        iBuilder->SetInsertPoint(entryBlock);

        BasicBlock* exitBlock = iBuilder->CreateBasicBlock("count_forward_bit_exit");


        auto onePackResult = genereateCountForwardBitsOnePack(iBuilder, inputStreamBufferName, cursorValue, isZero);

        Value* forwardCount = onePackResult.first;
        Value* isEndOfPack = onePackResult.second.first;
        Value* isExceedAvailable = onePackResult.second.second;
        Value* newCursorValue = iBuilder->CreateAdd(cursorValue, forwardCount);

        if (!hasIndexBits(inputStreamBufferName)) {
            Value* isNotFinished = iBuilder->CreateOr(isEndOfPack, isExceedAvailable);
            Value* isFinished = iBuilder->CreateNot(isNotFinished);

            if (maxPos) {
                Value* reachMaxPos = iBuilder->CreateICmpUGE(newCursorValue, maxPos);
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
        } else {
            BasicBlock* countIndexBitConBlock = iBuilder->CreateBasicBlock("count_forward_bit_count_index_con");
            BasicBlock* countIndexBitBodyBlock = iBuilder->CreateBasicBlock("count_forward_bit_count_index_body");
            BasicBlock* countFinalPackBlock = iBuilder->CreateBasicBlock("count_forward_bit_count_final_pack");

            BasicBlock* beforeExitBlock = iBuilder->CreateBasicBlock("count_forward_bit_before_exit");
            iBuilder->CreateBr(countIndexBitConBlock);

            // beforeExitBlock
            iBuilder->SetInsertPoint(beforeExitBlock);
            PHINode* finalNewCursorValue = iBuilder->CreatePHI(iBuilder->getSizeTy(), 3);
            PHINode* isFinish = iBuilder->CreatePHI(iBuilder->getInt1Ty(), 3);


            Value* retCursorValue = finalNewCursorValue;
            Value* retIsFinish = isFinish;
            if (maxPos) {
                Value* exceedMaxPos = iBuilder->CreateICmpUGE(retCursorValue, maxPos);
                retCursorValue = iBuilder->CreateSelect(exceedMaxPos, maxPos, retCursorValue);
                retIsFinish = iBuilder->CreateSelect(exceedMaxPos, iBuilder->getInt1(true), retIsFinish);
            }

            iBuilder->CreateBr(exitBlock);


            // countIndexBitConBlock
            iBuilder->SetInsertPoint(countIndexBitConBlock);

            // isEndOfPack && !isExceedAvailable
            Value* shouldCountIndexBit = isEndOfPack;

            if (maxPos) {
                Value* reachMaxPos = iBuilder->CreateICmpUGE(newCursorValue, maxPos);
                shouldCountIndexBit = iBuilder->CreateSelect(reachMaxPos, iBuilder->getInt1(false), shouldCountIndexBit);
            }

            finalNewCursorValue->addIncoming(newCursorValue, countIndexBitConBlock);
            isFinish->addIncoming(iBuilder->CreateNot(shouldCountIndexBit), countIndexBitConBlock);

            iBuilder->CreateCondBr(shouldCountIndexBit, countIndexBitBodyBlock, beforeExitBlock);

            // countIndexBitBodyBlock
            iBuilder->SetInsertPoint(countIndexBitBodyBlock);
            Value* countBeginBitIndex = iBuilder->CreateLShr(newCursorValue, iBuilder->getSize(std::log2(64)));


            Value* indexCount = this->generateCountIndexBit(iBuilder, inputStreamBufferName, !isZero, countBeginBitIndex);

            newCursorValue = iBuilder->CreateAdd(
                    newCursorValue,
                    iBuilder->CreateShl(
                            indexCount,
                            std::log2(64)
                    )
            );

            Value* itemsTotal = iBuilder->CreateAdd(iBuilder->getAvailableItemCount(inputStreamBufferName), iBuilder->getProcessedItemCount(inputStreamBufferName));
            isExceedAvailable = iBuilder->CreateICmpUGE(newCursorValue, itemsTotal);
            newCursorValue =  iBuilder->CreateSelect(
                    isExceedAvailable,
                    itemsTotal,
                    newCursorValue
            );
            BasicBlock* countIndexBitBodyExitBlock = iBuilder->GetInsertBlock();

            finalNewCursorValue->addIncoming(newCursorValue, countIndexBitBodyExitBlock);
            isFinish->addIncoming(iBuilder->CreateNot(isExceedAvailable), countIndexBitBodyExitBlock);

            iBuilder->CreateCondBr(
                    isExceedAvailable,
                    beforeExitBlock,
                    countFinalPackBlock
            );

            // CountFinalPackBlock
            iBuilder->SetInsertPoint(countFinalPackBlock);
            auto onePackResult = genereateCountForwardBitsOnePack(iBuilder, inputStreamBufferName, newCursorValue, isZero);

            forwardCount = onePackResult.first;
            //isEndOfPack = onePackResult.second.first;  // should always be false
            //isExceedAvailable = onePackResult.second.second; // should always be false
            Value* finalCursorValue = iBuilder->CreateAdd(newCursorValue, forwardCount);

            finalNewCursorValue->addIncoming(finalCursorValue, countFinalPackBlock);
            isFinish->addIncoming(iBuilder->getInt1(true), countFinalPackBlock);

            iBuilder->CreateBr(beforeExitBlock);

            // exit block
            iBuilder->SetInsertPoint(exitBlock);
            return std::make_pair(iBuilder->CreateSub(retCursorValue, cursorValue), retIsFinish);
        }

    };

    Value* SequentialKernel::generateCountIndexBit(const std::unique_ptr<KernelBuilder> &iBuilder, std::string streamName, bool isZero, llvm::Value* beginBitIndex) {
        string indexBitScalarName = isZero? this->generateInputZeroIndexName(streamName) : this->generateInputOneIndexName(streamName);
        BasicBlock* countIndexBitEntryBlock = iBuilder->CreateBasicBlock("count_index_bit_entry_block");

        BasicBlock* countIndexBitConBlock = iBuilder->CreateBasicBlock("count_index_bit_con_block");
        BasicBlock* countIndexBitBodyBlock = iBuilder->CreateBasicBlock("count_index_bit_body_block");
//        BasicBlock* countIndexBitFinalBlock = iBuilder->CreateBasicBlock("count_index_bit_final_block");
        BasicBlock* countIndexBitExitBlock = iBuilder->CreateBasicBlock("count_index_bit_exit_block");


        iBuilder->CreateBr(countIndexBitEntryBlock);

        // CountIndexBitEntry
        iBuilder->SetInsertPoint(countIndexBitEntryBlock);
        auto info = this->inputStreamIndexMap.find(streamName);
        //TODO
//        assert(( "index bit of " + streamName + " not exists") && (info != this->inputStreamIndexMap.end()));

        Value* itemsTotal = iBuilder->CreateAdd(iBuilder->getAvailableItemCount(streamName), iBuilder->getProcessedItemCount(streamName));
        Value* maxIndexBitCount = iBuilder->CreateLShr(
                iBuilder->CreateAdd(itemsTotal, iBuilder->getSize(63)),
                std::log2(64)
        );

        iBuilder->CreateBr(countIndexBitConBlock);
        //Con Block
        iBuilder->SetInsertPoint(countIndexBitConBlock);
        PHINode* currentBitIndex = iBuilder->CreatePHI(iBuilder->getSizeTy(), 2);
        currentBitIndex->addIncoming(beginBitIndex, countIndexBitEntryBlock);
        iBuilder->CreateCondBr(
                iBuilder->CreateICmpULT(
                        currentBitIndex,
                        maxIndexBitCount
                ),
                countIndexBitBodyBlock,
                countIndexBitExitBlock
        );

        // Body Block
        iBuilder->SetInsertPoint(countIndexBitBodyBlock);


        Value* countArrayIndex = iBuilder->CreateLShr(
                iBuilder->CreateAnd(
                        currentBitIndex,
                        iBuilder->getSize(this->getInputBufferSize(iBuilder, streamName) / 64 - 1)
                ),
                iBuilder->getSize(std::log2(64))
        );
        Value* countStartBitIndex = iBuilder->CreateAnd(currentBitIndex, iBuilder->getSize(63));

        Value* packData = iBuilder->CreateExtractElement(
                iBuilder->getScalarField(indexBitScalarName),
                countArrayIndex
        );

        packData = iBuilder->CreateSelect(
                iBuilder->CreateICmpEQ(countStartBitIndex, iBuilder->getSize(0)),
                packData,
                iBuilder->CreateLShr(packData, countStartBitIndex)
        );

        Value* forwardZeroCount = iBuilder->CreateCountForwardZeroes(packData);

        Value* isEndOfPack = iBuilder->CreateICmpUGE(iBuilder->CreateAdd(countStartBitIndex, forwardZeroCount), iBuilder->getSize(64));
        forwardZeroCount = iBuilder->CreateSelect(
                isEndOfPack,
                iBuilder->CreateSub(iBuilder->getSize(64), countStartBitIndex),
                forwardZeroCount
        );


        Value* newBitIndex = iBuilder->CreateAdd(currentBitIndex, forwardZeroCount);
        currentBitIndex->addIncoming(newBitIndex, countIndexBitBodyBlock);

        iBuilder->CreateCondBr(
                isEndOfPack,
                countIndexBitConBlock,
                countIndexBitExitBlock
        );


        //Exit Block
        iBuilder->SetInsertPoint(countIndexBitExitBlock);
        PHINode* finalBitIndex = iBuilder->CreatePHI(iBuilder->getSizeTy(), 2);
        finalBitIndex->addIncoming(currentBitIndex, countIndexBitConBlock);
        finalBitIndex->addIncoming(newBitIndex, countIndexBitBodyBlock);

        return iBuilder->CreateSub(finalBitIndex, beginBitIndex);
    }

    std::pair<llvm::Value*, llvm::Value*> SequentialKernel::generateCountForwardOnes(const unique_ptr<KernelBuilder> &iBuilder, string inputStreamBufferName, Value* beginOffset, Value* maxPos) {
        return this->generateCountForwardBits(iBuilder, inputStreamBufferName, beginOffset, false, maxPos);
    };

    std::pair<llvm::Value*, llvm::Value*> SequentialKernel::generateCountForwardZeros(const unique_ptr<KernelBuilder> &iBuilder, string inputStreamBufferName, Value* beginOffset, Value* maxPos) {
        return this->generateCountForwardBits(iBuilder, inputStreamBufferName, beginOffset, true, maxPos);
    }


    BasicBlock* SequentialKernel::advanceCursorUntilNextOne(const std::unique_ptr<KernelBuilder> &iBuilder, std::string cursorName, std::string inputStreamBufferName, Value* maxPos) {
        BasicBlock* entryBlock = iBuilder->CreateBasicBlock("advance_cursor_until_next_one_entry");

        this->recordCountForwardTempMaxPos(iBuilder, maxPos);

        iBuilder->CreateBr(entryBlock);
        iBuilder->SetInsertPoint(entryBlock);


        // StateIndex will be increased in waitCursorUntilInputAvailable
        this->waitCursorUntilInputAvailable(iBuilder, cursorName, inputStreamBufferName);

        BasicBlock* countForwareZeroBlock = iBuilder->CreateBasicBlock("advance_cursor_until_next_one_count_block");
        BasicBlock* exitBlock = iBuilder->CreateBasicBlock("advance_cursor_until_next_one_exit_block");

        iBuilder->CreateBr(countForwareZeroBlock);
        iBuilder->SetInsertPoint(countForwareZeroBlock);

        Value* cursorValue = this->getCursorValue(iBuilder, cursorName);

        maxPos = this->restoreCountForwardTempMaxPos(iBuilder, maxPos);

        auto retValue = this->generateCountForwardZeros(iBuilder, inputStreamBufferName, cursorValue, maxPos);

        cursorValue = iBuilder->CreateAdd(cursorValue, retValue.first);
        Value* isFinished = retValue.second;


        //TODO Add additional handle for isFinish (is isFinish === false, the next pack will always start from index 0), avoid using waitCursorUntilInputAvailable in the second loop

        this->setCursorValue(iBuilder, cursorName, cursorValue);

        iBuilder->CreateCondBr(isFinished, exitBlock, entryBlock);
        //TODO add index bits for count forward zeros and ones
        iBuilder->SetInsertPoint(exitBlock);
        return exitBlock;
    }


    BasicBlock* SequentialKernel::advanceCursorUntilNextZero(
            const std::unique_ptr<KernelBuilder> &iBuilder, std::string cursorName, std::string inputStreamBufferName, Value* maxPos) {
        BasicBlock* entryBlock = iBuilder->CreateBasicBlock("advance_cursor_until_next_zero_entry");

        this->recordCountForwardTempMaxPos(iBuilder, maxPos);

        iBuilder->CreateBr(entryBlock);
        iBuilder->SetInsertPoint(entryBlock);

        this->waitCursorUntilInputAvailable(iBuilder, cursorName, inputStreamBufferName);

        BasicBlock* countForwareOneBlock = iBuilder->CreateBasicBlock("advance_cursor_until_next_zero_count_block");
        BasicBlock* exitBlock = iBuilder->CreateBasicBlock("advance_cursor_until_next_zero_exit_block");

        iBuilder->CreateBr(countForwareOneBlock);
        iBuilder->SetInsertPoint(countForwareOneBlock);

        Value* cursorValue = this->getCursorValue(iBuilder, cursorName);

        maxPos = this->restoreCountForwardTempMaxPos(iBuilder, maxPos);

        auto retValue = this->generateCountForwardOnes(iBuilder, inputStreamBufferName, cursorValue, maxPos);

        this->advanceCursor(iBuilder, cursorName, retValue.first);

        Value* isFinished = retValue.second;

        iBuilder->CreateCondBr(isFinished, exitBlock, entryBlock);
        //TODO add index bits for count forward zeros and ones
        iBuilder->SetInsertPoint(exitBlock);
        return exitBlock;
    }

    void SequentialKernel::memcpyCircularBuffer(
            const std::unique_ptr<KernelBuilder> &iBuilder,
            string sourceBufferName,
            llvm::Value* sourceOffset,
            string dstBufferName,
            llvm::Value* outputOffset,
            llvm::Value* distance
    ) {

        size_t outputBufferSize = this->getOutputBufferSize(iBuilder, dstBufferName);
        Value* outputBufferSizeValue = iBuilder->getSize(outputBufferSize);
        Value* outputBufferSizeMask = iBuilder->getSize(outputBufferSize - 1);
        Value* maskedOutputOffset = iBuilder->CreateAnd(outputOffset, outputBufferSizeMask);
        Value* remainBuffer = iBuilder->CreateSub(outputBufferSizeValue, maskedOutputOffset);
        Value* copyLength1 = iBuilder->CreateSelect(iBuilder->CreateICmpUGE(remainBuffer, distance), distance, remainBuffer);
        Value* copyLength2 = iBuilder->CreateSub(distance, copyLength1);


        Value* inputBufferBasePtr = iBuilder->getRawInputPointer(sourceBufferName, iBuilder->getSize(0));
        Value* outputBufferBasePtr = iBuilder->getRawOutputPointer(dstBufferName, iBuilder->getSize(0));

        iBuilder->CreateMemCpy(
                iBuilder->CreateGEP(outputBufferBasePtr, maskedOutputOffset),
                iBuilder->CreateGEP(inputBufferBasePtr, sourceOffset),
                copyLength1,
                1); // no alignment guaranteed
        // Assumed output buffer is Circular buffer
        iBuilder->CreateMemCpy(
                outputBufferBasePtr,
                iBuilder->CreateGEP(inputBufferBasePtr, iBuilder->CreateAdd(sourceOffset, copyLength1)),
                copyLength2,
                8
        );
        iBuilder->setProducedItemCount(dstBufferName, iBuilder->CreateAdd(outputOffset, distance));
    }

    BasicBlock* SequentialKernel::memcpy2CursorsUntilNextZero(
            const std::unique_ptr<KernelBuilder> &iBuilder,
            string sourceBufferName,
            string sourceCursorName,
            string dstBufferName,
            string dstCursorName,
            string sourceMarkerName,
            Value* maxPos
    ) {
        BasicBlock* previousEntryBlock = iBuilder->GetInsertBlock();

        BasicBlock* entryBlock = iBuilder->CreateBasicBlock("memcpy_2_cursors_until_next_zero_entry");
        this->recordCountForwardTempMaxPos(iBuilder, maxPos);

        iBuilder->CreateBr(entryBlock);
        iBuilder->SetInsertPoint(entryBlock);

        this->waitCursorUntilInputAvailable(iBuilder, sourceCursorName, sourceMarkerName);

        BasicBlock* bodyBlock = iBuilder->CreateBasicBlock("memcpy_2_cursors_until_next_zero_body");
        BasicBlock* exitBlock = iBuilder->CreateBasicBlock("memcpy_2_cursors_until_next_zero_exit");

        iBuilder->CreateBr(bodyBlock);
        iBuilder->SetInsertPoint(bodyBlock);

        // Count Forward Zero in this pack
        Value* sourceCursorValue = this->getCursorValue(iBuilder, sourceCursorName);

        maxPos = this->restoreCountForwardTempMaxPos(iBuilder, maxPos);
        auto retValue = this->generateCountForwardOnes(iBuilder, sourceMarkerName, sourceCursorValue, maxPos);
        Value* distance = retValue.first;

        // Memcpy from sourceBuffer[sourceCursor : sourceCursor + distance] to dstBuffer[dstCursor : dstCursor + distance]
        Value* inputBufferBasePtr = iBuilder->getRawInputPointer(sourceBufferName, iBuilder->getSize(0));
        Value* outputBufferBasePtr = iBuilder->getRawOutputPointer(dstBufferName, iBuilder->getSize(0));

        Value* outputOffset = this->getCursorValue(iBuilder, dstCursorName);
        size_t outputBufferSize = this->getOutputBufferSize(iBuilder, dstBufferName);
        Value* outputBufferSizeValue = iBuilder->getSize(outputBufferSize);
        Value* outputBufferSizeMask = iBuilder->getSize(outputBufferSize - 1);
        Value* maskedOutputOffset = iBuilder->CreateAnd(outputOffset, outputBufferSizeMask);
        Value* remainBuffer = iBuilder->CreateSub(outputBufferSizeValue, maskedOutputOffset);
        Value* copyLength1 = iBuilder->CreateSelect(iBuilder->CreateICmpUGE(remainBuffer, distance), distance, remainBuffer);
        Value* copyLength2 = iBuilder->CreateSub(distance, copyLength1);

        iBuilder->CreateMemCpy(
                iBuilder->CreateGEP(outputBufferBasePtr, maskedOutputOffset),
                iBuilder->CreateGEP(inputBufferBasePtr, sourceCursorValue),
                copyLength1,
                1); // no alignment guaranteed
        // Assumed output buffer is Circular buffer
        iBuilder->CreateMemCpy(
                outputBufferBasePtr,
                iBuilder->CreateGEP(inputBufferBasePtr, iBuilder->CreateAdd(sourceCursorValue, copyLength1)),
                copyLength2,
                8
        );

        // Update cursor value and producedItemCount
        this->advanceCursor(iBuilder, sourceCursorName, distance);
        this->advanceCursor(iBuilder, dstCursorName, distance);
        iBuilder->setProducedItemCount(dstBufferName, this->getCursorValue(iBuilder, dstCursorName));

        // Finish
        Value* isFinished = retValue.second;
        iBuilder->CreateCondBr(isFinished, exitBlock, entryBlock);
        //TODO should not use index bits for count forward zeros in this case
        iBuilder->SetInsertPoint(exitBlock);
        return exitBlock;
    }

    BasicBlock* SequentialKernel::memcpyOutputDst(
            const unique_ptr<KernelBuilder> &iBuilder,
            string outputBufferName,
            Value* copyOffset,
            Value* copyLength

    ) {
        Value* distance = copyLength;

        BasicBlock* matchCopyEntryBlock = iBuilder->CreateBasicBlock("memcpy_output_dst_cursor_until_next_zero_matchcpy_entry");
        BasicBlock* matchCopyExitBlock = iBuilder->CreateBasicBlock("memcpy_output_dst_cursor_until_next_zero_matchcpy_exit");

        Value* outputOffset = iBuilder->getProducedItemCount(outputBufferName);

        iBuilder->CreateBr(matchCopyEntryBlock);

        iBuilder->SetInsertPoint(matchCopyEntryBlock);
        this->generateDstMatchCopy(iBuilder, matchCopyEntryBlock, matchCopyExitBlock, outputBufferName, copyOffset, distance, outputOffset);

        iBuilder->SetInsertPoint(matchCopyExitBlock);
        // Update Cursor Value and producedItemCount
        iBuilder->setProducedItemCount(outputBufferName, iBuilder->CreateAdd(outputOffset, copyLength));

        return matchCopyExitBlock;
    }

    llvm::BasicBlock* SequentialKernel::memcpyOutputDstCursorUntilNextZero(
            const std::unique_ptr<KernelBuilder> &iBuilder,
            std::string outputBufferName,
            llvm::Value* copyOffset,
            std::string dstCursorName,
            std::string dstMarkerName,
            llvm::Value* maxPos
    ) {
        iBuilder->setScalarField(MemCpyUntilZeroCopyOffsetTempKey, copyOffset);
        this->recordCountForwardTempMaxPos(iBuilder, maxPos);

        BasicBlock* entryBlock = iBuilder->CreateBasicBlock("memcpy_ooutput_dst_cursor_until_next_zero_entry");
        iBuilder->CreateBr(entryBlock);
        iBuilder->SetInsertPoint(entryBlock);

        this->waitCursorUntilInputAvailable(iBuilder, dstCursorName, dstMarkerName);

        BasicBlock* bodyBlock = iBuilder->CreateBasicBlock("memcpy_output_dst_cursor_until_next_zero_body");
        BasicBlock* exitBlock = iBuilder->CreateBasicBlock("memcpy_output_dst_cursor_until_next_zero_exit");

        iBuilder->CreateBr(bodyBlock);
        iBuilder->SetInsertPoint(bodyBlock);

        // Count Forward Zero in this pack
        Value* cursorValue = this->getCursorValue(iBuilder, dstCursorName);
        maxPos = this->restoreCountForwardTempMaxPos(iBuilder, maxPos);
        auto retValue = this->generateCountForwardOnes(iBuilder, dstMarkerName, cursorValue, maxPos);
        Value* distance = retValue.first;

        // Memcpy from outputBuffer[cursorValue - copyOffset : cursorValue - copyOffset + distance] to outputBuffer[cursorValue : cursorValue + distance]
        BasicBlock* matchCopyEntryBlock = iBuilder->CreateBasicBlock("memcpy_output_dst_cursor_until_next_zero_matchcpy_entry");
        BasicBlock* matchCopyExitBlock = iBuilder->CreateBasicBlock("memcpy_output_dst_cursor_until_next_zero_matchcpy_exit");
        Value* outputOffset = this->getCursorValue(iBuilder, dstCursorName);

        iBuilder->CreateBr(matchCopyEntryBlock);

        iBuilder->SetInsertPoint(matchCopyEntryBlock);
        copyOffset = iBuilder->getScalarField(MemCpyUntilZeroCopyOffsetTempKey);
        this->generateDstMatchCopy(iBuilder, matchCopyEntryBlock, matchCopyExitBlock, outputBufferName, copyOffset, distance, outputOffset);

        iBuilder->SetInsertPoint(matchCopyExitBlock);
        // Update Cursor Value and producedItemCount
        this->advanceCursor(iBuilder, dstCursorName, distance);
        iBuilder->setProducedItemCount(outputBufferName, this->getCursorValue(iBuilder, dstCursorName));

        // Finish
        Value* isFinished = retValue.second;
        iBuilder->CreateCondBr(isFinished, exitBlock, entryBlock);


        iBuilder->SetInsertPoint(exitBlock);

        return exitBlock;
    }

    void SequentialKernel::generateDstMatchCopy(const std::unique_ptr<KernelBuilder> & iBuilder, BasicBlock* entry, BasicBlock* exit, string outputBufferName, Value* matchOffset, Value* matchLength, Value* outputOffset) {
        iBuilder->SetInsertPoint(entry);

        Value * outputBufferBasePtr = iBuilder->getRawOutputPointer(outputBufferName, iBuilder->getSize(0));

        Value* bufferSize = iBuilder->getSize(this->getOutputBufferSize(iBuilder, outputBufferName));
        Value* bufferSizeMask = iBuilder->CreateSub(bufferSize, iBuilder->getSize(1));


        Value* matchStart = iBuilder->CreateSub(outputOffset, matchOffset);
        Value * baseSrcOffset = iBuilder->CreateAnd(matchStart, bufferSizeMask);
        Value * baseDstOffset = iBuilder->CreateAnd(outputOffset, bufferSizeMask);


        Value * copyStep = iBuilder->CreateSelect(
                iBuilder->CreateICmpULT(matchOffset, iBuilder->getSize(4)),
                iBuilder->getSize(1),
                iBuilder->getSize(4)
        );


        BasicBlock * cpyLoopCond = iBuilder->CreateBasicBlock("matchcopy_loop_cond");
        BasicBlock * cpyLoopBody = iBuilder->CreateBasicBlock("matchcopy_loop_body");
        BasicBlock * cpyLoopExit = iBuilder->CreateBasicBlock("matchcopy_loop_exit");


        iBuilder->CreateBr(cpyLoopCond);

        iBuilder->SetInsertPoint(cpyLoopCond);

        PHINode * phiSrcOffset = iBuilder->CreatePHI(iBuilder->getSizeTy(), 3);
        PHINode * phiDstOffset = iBuilder->CreatePHI(iBuilder->getSizeTy(), 3);
        PHINode * phiIter = iBuilder->CreatePHI(iBuilder->getSizeTy(), 3);
        phiSrcOffset->addIncoming(baseSrcOffset, entry);
        phiDstOffset->addIncoming(baseDstOffset, entry);
        phiIter->addIncoming(iBuilder->getSize(0), entry);

        iBuilder->CreateCondBr(
                iBuilder->CreateICmpUGE(phiIter, matchLength),
                cpyLoopExit,
                cpyLoopBody
        );

        iBuilder->SetInsertPoint(cpyLoopBody);
        BasicBlock * reachingBufferEnd_then = iBuilder->CreateBasicBlock("matchcopy_reaching_buf_end_then");
        BasicBlock * reachingBufferEnd_else = iBuilder->CreateBasicBlock("matchcopy_reaching_buf_end_else");


        Value * distSrcEnd = iBuilder->CreateSub(bufferSize, phiSrcOffset);
        Value * distDstEnd = iBuilder->CreateSub(bufferSize, phiDstOffset);
        Value * minDist = iBuilder->CreateSelect(iBuilder->CreateICmpULT(distSrcEnd, distDstEnd), distSrcEnd, distDstEnd);
        iBuilder->CreateUnlikelyCondBr(
                iBuilder->CreateICmpULE(minDist, iBuilder->getSize(4)),
                reachingBufferEnd_then,
                reachingBufferEnd_else
        );

        iBuilder->SetInsertPoint(reachingBufferEnd_then);

        Value * src8 = iBuilder->CreateGEP(outputBufferBasePtr, phiSrcOffset);
        Value * dst8 = iBuilder->CreateGEP(outputBufferBasePtr, phiDstOffset);
        iBuilder->CreateStore(iBuilder->CreateLoad(src8), dst8);
        Value * newSrcOffset = iBuilder->CreateAnd(
                iBuilder->CreateAdd(phiSrcOffset, iBuilder->getSize(1)),
                bufferSizeMask
        );
        Value * newDstOffset = iBuilder->CreateAnd(
                iBuilder->CreateAdd(phiDstOffset, iBuilder->getSize(1)),
                bufferSizeMask
        );
        phiSrcOffset->addIncoming(newSrcOffset, reachingBufferEnd_then);
        phiDstOffset->addIncoming(newDstOffset, reachingBufferEnd_then);
        phiIter->addIncoming(iBuilder->CreateAdd(phiIter, iBuilder->getSize(1)), reachingBufferEnd_then);
        iBuilder->CreateBr(cpyLoopCond);


        iBuilder->SetInsertPoint(reachingBufferEnd_else);
        // Copy 4 bytes at a time (regardless of step length).
        Value * src32 = iBuilder->CreatePointerCast(
                iBuilder->CreateGEP(outputBufferBasePtr, phiSrcOffset),
                iBuilder->getInt32Ty()->getPointerTo());
        Value * dst32 = iBuilder->CreatePointerCast(
                iBuilder->CreateGEP(outputBufferBasePtr, phiDstOffset),
                iBuilder->getInt32Ty()->getPointerTo());
        // Force unaligned load/store of an int32.
        iBuilder->CreateAlignedStore(iBuilder->CreateAlignedLoad(src32, 1), dst32, 1);
        newSrcOffset = iBuilder->CreateAnd(
                iBuilder->CreateAdd(phiSrcOffset, copyStep),
                bufferSizeMask
        );
        newDstOffset = iBuilder->CreateAnd(
                iBuilder->CreateAdd(phiDstOffset, copyStep),
                bufferSizeMask
        );
        phiSrcOffset->addIncoming(newSrcOffset, reachingBufferEnd_else);
        phiDstOffset->addIncoming(newDstOffset, reachingBufferEnd_else);
        phiIter->addIncoming(iBuilder->CreateAdd(phiIter, copyStep), reachingBufferEnd_else);
        iBuilder->CreateBr(cpyLoopCond);

        iBuilder->SetInsertPoint(cpyLoopExit);
        outputOffset = iBuilder->CreateAdd(outputOffset, matchLength);

        iBuilder->CreateBr(exit);
    }


    BasicBlock* SequentialKernel::waitCursorUntilInputAvailable(const std::unique_ptr<KernelBuilder> &iBuilder, std::string cursorName, std::string inputStreamBufferName) {
//        BasicBlock* entryBlock = iBuilder->GetInsertBlock();
        Value* nextStateValue = iBuilder->getSize(this->stateBlocks.size());

        BasicBlock* restoreBlock = iBuilder->CreateBasicBlock("wait_cursor_until_input_available_restore");
        BasicBlock* continueBlock = iBuilder->CreateBasicBlock("wait_cursor_until_input_available_continue");

        this->stateBlocks.push_back(restoreBlock);

        iBuilder->CreateBr(restoreBlock);

        iBuilder->SetInsertPoint(restoreBlock);

        Value* cursorValue = this->getCursorValue(iBuilder, cursorName);
        Value* itemTotal = iBuilder->CreateAdd(iBuilder->getAvailableItemCount(inputStreamBufferName), iBuilder->getProcessedItemCount(inputStreamBufferName));
        Value* isAvailable = iBuilder->CreateICmpULT(cursorValue, itemTotal);

        Value* nextState = iBuilder->CreateSelect(isAvailable, iBuilder->getSize(0), nextStateValue);
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

    Value* SequentialKernel::offsetToPackBaseOffset(const unique_ptr<KernelBuilder> &iBuilder, Value* offset) {
        return iBuilder->CreateShl(
                this->offsetToPackIndex(iBuilder, offset),
                iBuilder->getSize(std::log2(64))
        );
    }
    Value* SequentialKernel::offsetToPackIndex(const unique_ptr<KernelBuilder> &iBuilder, Value* offset) {
        return iBuilder->CreateLShr(offset, iBuilder->getSize(std::log2(64)));
    }

    Value* SequentialKernel::offsetToPackOffset(const unique_ptr<KernelBuilder> &iBuilder, Value* offset) {
        return iBuilder->CreateAnd(offset, iBuilder->getSize(64 - 1));
    }

    Value* SequentialKernel::offsetToActualBufferOffset(const unique_ptr<KernelBuilder> &iBuilder, string inputBufferName, Value* offset) {
        size_t bufferSize = this->getInputBufferSize(iBuilder, inputBufferName);
        Value* bufferOffsetMask = iBuilder->getSize(bufferSize - 1);
        return iBuilder->CreateAnd(bufferOffsetMask, offset);
    }

    Value* SequentialKernel::generateLoadCircularInputPack(const unique_ptr<KernelBuilder> &iBuilder, string inputBufferName, Value* offset) {
        Value* actualBufferOffset = this->offsetToActualBufferOffset(iBuilder, inputBufferName, offset);
        Value* packIndex = this->offsetToPackIndex(iBuilder, actualBufferOffset);
//        Value* countStartBitIndex = this->offsetToPackOffset(iBuilder, actualBufferOffset);


        Value* inputStreamPtr = iBuilder->getInputStreamBlockPtr(inputBufferName, iBuilder->getInt32(0));
        inputStreamPtr = iBuilder->CreatePointerCast(inputStreamPtr, iBuilder->getInt64Ty()->getPointerTo());
        return iBuilder->CreateLoad(iBuilder->CreateGEP(inputStreamPtr, packIndex));

//        packData = iBuilder->CreateLShr(packData, countStartBitIndex);

    }

    Value* SequentialKernel::generateLoadCircularInput(const unique_ptr<KernelBuilder> &iBuilder, string inputBufferName, Value* offset, Type* pointerType) {
        size_t inputSize = this->getInputBufferSize(iBuilder, inputBufferName);
        Value* offsetMask = iBuilder->getSize(inputSize - 1);
        Value* maskedOffset = iBuilder->CreateAnd(offsetMask, offset);

        Value* inputBufferPtr = iBuilder->getRawInputPointer(inputBufferName, iBuilder->getSize(0));

        inputBufferPtr = iBuilder->CreatePointerCast(inputBufferPtr, pointerType);
        return iBuilder->CreateLoad(iBuilder->CreateGEP(inputBufferPtr, maskedOffset));
    }
    Value* SequentialKernel::generateLoadCircularOutput(const unique_ptr<KernelBuilder> &iBuilder, string inputBufferName, Value* offset, Type* pointerType) {
        size_t inputSize = this->getOutputBufferSize(iBuilder, inputBufferName);
        Value* offsetMask = iBuilder->getSize(inputSize - 1);
        Value* maskedOffset = iBuilder->CreateAnd(offsetMask, offset);

        Value* inputBufferPtr = iBuilder->getRawOutputPointer(inputBufferName, iBuilder->getSize(0));

        inputBufferPtr = iBuilder->CreatePointerCast(inputBufferPtr, pointerType);
        return iBuilder->CreateLoad(iBuilder->CreateGEP(inputBufferPtr, maskedOffset));
    }

    Value* SequentialKernel::generateLoadSourceInputByte(const std::unique_ptr<KernelBuilder> &iBuilder, string sourceBufferName, Value* offset) {
        Value * blockStartPtr = iBuilder->CreatePointerCast(
                iBuilder->getInputStreamBlockPtr(sourceBufferName, iBuilder->getInt32(0)),
                iBuilder->getInt8PtrTy()
        );
        Value * ptr = iBuilder->CreateGEP(blockStartPtr, offset);


        return iBuilder->CreateLoad(ptr);
    }


    void SequentialKernel::generateStoreCircularOutput(const unique_ptr<KernelBuilder> &iBuilder, string outputBufferName, Type* pointerType, Value* value) {
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

    void SequentialKernel::increaseScalarField(const std::unique_ptr<KernelBuilder> &iBuilder, const std::string& fieldName, llvm::Value* value) {
        Value* fieldValue = iBuilder->getScalarField(fieldName);
        fieldValue = iBuilder->CreateAdd(fieldValue, value);
        iBuilder->setScalarField(fieldName, fieldValue);
    }


    void SequentialKernel::markCircularOutputBitstreamOnePack(const std::unique_ptr<KernelBuilder> &iBuilder, const std::string& bitstreamName, llvm::Value* start, llvm::Value* end, bool isOne) {
        Value* outputBasePtr = iBuilder->getRawOutputPointer(bitstreamName, iBuilder->getSize(0));

        outputBasePtr = iBuilder->CreatePointerCast(outputBasePtr, iBuilder->getInt64Ty()->getPointerTo());

        size_t outputBufferSize = this->getOutputBufferSize(iBuilder, bitstreamName);
        Value* outputMask = iBuilder->getSize(outputBufferSize / 64 - 1);


        Value* startOffset = iBuilder->CreateLShr(start, iBuilder->getSize(std::log2(64)), "startOffset");
        Value* curOffset = startOffset;


        Value* outputLowestBitValue = iBuilder->CreateSelect(
                iBuilder->CreateICmpULE(
                        iBuilder->CreateShl(curOffset, std::log2(64)),
                        start
                ),
                iBuilder->CreateShl(iBuilder->getSize(1), iBuilder->CreateAnd(start, iBuilder->getSize(64 - 1))),
                iBuilder->getSize(1)
        );

        Value* outputHighestBitValue = iBuilder->CreateShl(
                iBuilder->getSize(1),
                iBuilder->CreateAnd(end, iBuilder->getSize(64 - 1))
        );


        Value* bitMask = iBuilder->CreateSub(
                outputHighestBitValue,
                outputLowestBitValue
        );

        if (!isOne) {
            bitMask = iBuilder->CreateNot(bitMask);
        }
    }

    // Assume we have enough output buffer
    llvm::BasicBlock* SequentialKernel::markCircularOutputBitstream(const std::unique_ptr<KernelBuilder> &iBuilder, const std::string& bitstreamName, llvm::Value* start, llvm::Value* end, bool isOne, bool setProduced) {
        BasicBlock* entryBlock = iBuilder->GetInsertBlock();

        Value* outputBasePtr = iBuilder->getRawOutputPointer(bitstreamName, iBuilder->getSize(0));

        outputBasePtr = iBuilder->CreatePointerCast(outputBasePtr, iBuilder->getInt64Ty()->getPointerTo());

        size_t outputBufferSize = this->getOutputBufferSize(iBuilder, bitstreamName);
        Value* outputMask = iBuilder->getSize(outputBufferSize / 64 - 1);

        BasicBlock* conBlock = iBuilder->CreateBasicBlock("mark_bit_one_con");
        BasicBlock* bodyBlock =iBuilder->CreateBasicBlock("mark_bit_one_body");
        BasicBlock* exitBlock =iBuilder->CreateBasicBlock("mark_bit_one_exit");

        Value* startOffset = iBuilder->CreateLShr(start, iBuilder->getSize(std::log2(64)), "startOffset");

        iBuilder->CreateBr(conBlock);

        // Con
        iBuilder->SetInsertPoint(conBlock);


        PHINode* curOffset = iBuilder->CreatePHI(iBuilder->getSizeTy(), 2);
        curOffset->addIncoming(startOffset, entryBlock);

        iBuilder->CreateCondBr(
                iBuilder->CreateICmpULT(iBuilder->CreateShl(curOffset, std::log2(64)), end),
                bodyBlock,
                exitBlock
        );

        // Body
        iBuilder->SetInsertPoint(bodyBlock);
        Value* maskedOffset = iBuilder->CreateAnd(curOffset, outputMask);

        Value* outputLowestBitValue = iBuilder->CreateSelect(
                iBuilder->CreateICmpULE(
                        iBuilder->CreateShl(curOffset, std::log2(64)),
                        start
                ),
                iBuilder->CreateShl(iBuilder->getSize(1), iBuilder->CreateAnd(start, iBuilder->getSize(64 - 1))),
                iBuilder->getSize(1)
        );

        Value* hasNotReachEnd = iBuilder->CreateICmpULE(
                iBuilder->CreateShl(iBuilder->CreateAdd(curOffset, iBuilder->getSize(1)), std::log2(64)),
                end
        );
        Value* producedItemsCount = iBuilder->CreateSelect(
                hasNotReachEnd,
                iBuilder->CreateShl(iBuilder->CreateAdd(curOffset, iBuilder->getSize(1)), std::log2(64)),
                end
        );

        Value* outputHighestBitValue = iBuilder->CreateSelect(
                hasNotReachEnd,
                iBuilder->getSize(0),
                iBuilder->CreateShl(
                        iBuilder->getSize(1),
                        iBuilder->CreateAnd(end, iBuilder->getSize(64 - 1))
                )
        );


        Value* bitMask = iBuilder->CreateSub(
                outputHighestBitValue,
                outputLowestBitValue
        );

        if (!isOne) {
            bitMask = iBuilder->CreateNot(bitMask);
        }

        Value* targetPtr = iBuilder->CreateGEP(outputBasePtr, maskedOffset);
        Value* oldValue = iBuilder->CreateLoad(targetPtr);
        Value* newValue = NULL;
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