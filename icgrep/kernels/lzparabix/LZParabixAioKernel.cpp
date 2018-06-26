//
// Created by wxy325 on 2018/6/18.
//

#include "LZParabixAioKernel.h"

#include <kernels/kernel_builder.h>
#include <iostream>
#include <string>
#include <llvm/Support/raw_ostream.h>
#include <kernels/streamset.h>
#include <cstdint>


using namespace llvm;
using namespace kernel;
using namespace std;


namespace kernel{
    LZParabixAioKernel::LZParabixAioKernel(const std::unique_ptr<kernel::KernelBuilder> &b, std::vector<unsigned> numsOfBitStreams)
            :SegmentOrientedKernel("LZParabixAioKernel",
            // Inputs
                                   {
                    Binding{b->getStreamSetTy(1, 8), "byteStream", BoundedRate(0, 1)},

                    // block data
                    Binding{b->getStreamSetTy(1, 64), "blockStart", BoundedRate(0, 1), AlwaysConsume()},
                    Binding{b->getStreamSetTy(1, 64), "blockEnd", RateEqualTo("blockStart"), AlwaysConsume()}


            },
            //Outputs
                                   {

                                   },
            //Arguments
                                   {
                                           Binding{b->getSizeTy(), "fileSize"}
                                   },
                                   {},
            //Internal states:
                                   {
                                           Binding{b->getSizeTy(), "blockDataIndex"},
                                           Binding{b->getInt64Ty(), "outputPos"},


                                   }), mNumsOfBitStreams(numsOfBitStreams) {
        this->setStride(4 * 1024 * 1024);
        addAttribute(MustExplicitlyTerminate());

        mStreamSetInputs.push_back(Binding{b->getStreamSetTy(numsOfBitStreams[0], 1), "inputBitStream0", RateEqualTo("byteStream")});
        mStreamSetOutputs.push_back(Binding{b->getStreamSetTy(numsOfBitStreams[0], 1), "outputStream0", BoundedRate(0, 1)});

        for (unsigned i = 1; i < numsOfBitStreams.size(); i++) {
            mStreamSetInputs.push_back(Binding{b->getStreamSetTy(numsOfBitStreams[i], 1), "inputBitStream" + std::to_string(i), RateEqualTo("byteStream")});
            mStreamSetOutputs.push_back(Binding{b->getStreamSetTy(numsOfBitStreams[i], 1), "outputStream" + std::to_string(i), RateEqualTo("outputStream0")});
        }

        this->initPendingOutputScalar(b);

    }


    void LZParabixAioKernel::generateDoSegmentMethod(const std::unique_ptr<KernelBuilder> &b) {
        BasicBlock* exitBlock = b->CreateBasicBlock("exitBlock");
        BasicBlock* blockEndConBlock = b->CreateBasicBlock("blockEndConBlock");

        Value * blockDataIndex = b->getScalarField("blockDataIndex");
        Value * totalNumber = b->getAvailableItemCount("blockEnd");

        Value * blockEnd = this->generateLoadInt64NumberInput(b, "blockEnd", blockDataIndex);

        b->CreateCondBr(b->CreateICmpULT(blockDataIndex, totalNumber), blockEndConBlock, exitBlock);

        b->SetInsertPoint(blockEndConBlock);
        Value * blockStart = this->generateLoadInt64NumberInput(b, "blockStart", blockDataIndex);
        BasicBlock * processBlock = b->CreateBasicBlock("processBlock");
//        b->CreateCondBr(b->CreateICmpULE(blockEnd, totalExtender), processBlock, exitBlock);
        b->CreateBr(processBlock);

        b->SetInsertPoint(processBlock);

        this->generateProcessCompressedBlock(b, blockStart, blockEnd);


        Value * newBlockDataIndex = b->CreateAdd(blockDataIndex, b->getInt64(1));
        b->setScalarField("blockDataIndex", newBlockDataIndex);
        b->setProcessedItemCount("blockStart", newBlockDataIndex);
        b->setProcessedItemCount("byteStream", blockEnd);
        b->setProducedItemCount("outputStream0", b->getScalarField("outputPos"));
//        b->CallPrintInt("produced", b->getScalarField("outputPos"));
        b->CreateBr(exitBlock);

        b->SetInsertPoint(exitBlock);
    }


    llvm::Value *LZParabixAioKernel::generateLoadInt64NumberInput(const std::unique_ptr<KernelBuilder> &iBuilder,
                                                                      std::string inputBufferName, llvm::Value *globalOffset) {

        Value * capacity = iBuilder->getCapacity(inputBufferName);
        Value * processed = iBuilder->getProcessedItemCount(inputBufferName);
        processed = iBuilder->CreateAnd(processed, iBuilder->CreateNeg(capacity));
        Value * offset = iBuilder->CreateSub(globalOffset, processed);
        Value * valuePtr = iBuilder->getRawInputPointer(inputBufferName, offset);
        return iBuilder->CreateLoad(valuePtr);
    }

    void LZParabixAioKernel::generateProcessCompressedBlock(const std::unique_ptr<KernelBuilder> &b,
                                                            llvm::Value *lz4BlockStart, llvm::Value *lz4BlockEnd) {
        Value* isTerminal = b->CreateICmpEQ(lz4BlockEnd, b->getScalarField("fileSize"));
        b->setTerminationSignal(isTerminal);

        BasicBlock* exitBlock = b->CreateBasicBlock("processCompressedExitBlock");

        BasicBlock* processCon = b->CreateBasicBlock("processCompressedConBlock");
        BasicBlock* processBody = b->CreateBasicBlock("processCompressedBodyBlock");


        BasicBlock* beforeProcessConBlock = b->GetInsertBlock();
        b->CreateBr(processCon);
        b->SetInsertPoint(processCon);

        PHINode* phiCursorValue = b->CreatePHI(b->getInt64Ty(), 2, "phiCursorValue"); // phiCursorValue should always be the position of next token except for the final sequence
        phiCursorValue->addIncoming(lz4BlockStart, beforeProcessConBlock);

        b->CreateCondBr(b->CreateICmpULT(phiCursorValue, lz4BlockEnd), processBody, exitBlock);

        b->SetInsertPoint(processBody);

        Value* nextTokenGlobalPos = this->processSequence(b, phiCursorValue, lz4BlockEnd);
        phiCursorValue->addIncoming(nextTokenGlobalPos, b->GetInsertBlock());
        b->CreateBr(processCon);

        b->SetInsertPoint(exitBlock);
        this->storePendingOutput(b);

    }


    llvm::Value *LZParabixAioKernel::processSequence(const std::unique_ptr<KernelBuilder> &b, llvm::Value *cursorPos,
                                                              llvm::Value *lz4BlockEnd) {

        BasicBlock* exitBlock = b->CreateBasicBlock("exitBlock");
        Value* cursorNextPos = b->CreateAdd(cursorPos, b->getSize(1));

        Value* sequenceBasePtr = b->getRawInputPointer("byteStream", cursorPos);
        Value* sequenceToken = b->CreateLoad(sequenceBasePtr);

        Value* highestTokenBit = b->CreateAnd(sequenceToken, b->getInt8((uint8_t)1 << 7));
        Value* isLiteral = b->CreateICmpNE(highestTokenBit, b->getInt8(0));
        Value* tokenNumValue = b->CreateZExt(b->CreateSub(sequenceToken, highestTokenBit), b->getSizeTy());


        BasicBlock* literalProcessBlock = b->CreateBasicBlock("literalProcessBlock");
        BasicBlock* matchProcessBlock = b->CreateBasicBlock("matchProcessBlock");
        b->CreateCondBr(isLiteral, literalProcessBlock, matchProcessBlock);

        // ---- literalProcessBlock
        b->SetInsertPoint(literalProcessBlock);
        this->processLiteral(b, cursorNextPos, tokenNumValue);
        Value* newCursorPosAfterLiteral = b->CreateAdd(cursorNextPos, tokenNumValue);


        BasicBlock* literalProcessFinalBlock = b->GetInsertBlock();
        b->CreateBr(exitBlock);

        // ---- matchProcessBlock
        b->SetInsertPoint(matchProcessBlock);

        Value* matchIndexBytes = this->processMatch(b, cursorNextPos, tokenNumValue, sequenceBasePtr);
        Value* newCursorPosAfterMatch = b->CreateAdd(cursorNextPos, matchIndexBytes);

        BasicBlock* matchProcessFinalBlock = b->GetInsertBlock();
        b->CreateBr(exitBlock);

        // ---- exitBlock
        b->SetInsertPoint(exitBlock);
        PHINode* phiCursorValue = b->CreatePHI(b->getSizeTy(), 2);
        phiCursorValue->addIncoming(newCursorPosAfterLiteral, literalProcessFinalBlock);
        phiCursorValue->addIncoming(newCursorPosAfterMatch, matchProcessFinalBlock);


        return phiCursorValue;
    }

    llvm::Value *LZParabixAioKernel::processLiteral(const std::unique_ptr<KernelBuilder> &b, llvm::Value* cursorPos, llvm::Value* literalLength) {
        Value* remCursorPos = b->CreateURem(cursorPos, b->getCapacity("inputBitStream0"));

        Value* cursorBlockIndex = b->CreateUDiv(remCursorPos, b->getSize(b->getBitBlockWidth()));
        Value* cursorBlockRem = b->CreateURem(remCursorPos, b->getSize(b->getBitBlockWidth()));
        Value* cursorI64BlockIndex = b->CreateUDiv(cursorBlockRem, b->getSize(64));
        Value* cursorI64BlockRem = b->CreateURem(cursorBlockRem, b->getSize(64));
        Value* literalMask = b->CreateSub(b->CreateShl(b->getInt64(1), literalLength), b->getInt64(1));

        std::vector<llvm::Value*> extractValues;


        for (unsigned i = 0; i < mNumsOfBitStreams.size(); i++) {
            Value* bitStreamBasePtr = b->CreatePointerCast(b->getRawInputPointer("inputBitStream" + std::to_string(i), b->getSize(0)), b->getBitBlockType()->getPointerTo());
            Value* targetBlockBasePtr = b->CreatePointerCast(b->CreateGEP(bitStreamBasePtr, b->CreateMul(cursorBlockIndex, b->getSize(mNumsOfBitStreams[i]))), b->getInt64Ty()->getPointerTo());

            for (unsigned j = 0; j < mNumsOfBitStreams[i]; j++) {
                Value* ptr = b->CreateGEP(targetBlockBasePtr, b->CreateAdd(cursorI64BlockIndex, b->getSize(j * (b->getBitBlockWidth() / 64))));
                Value* extractV = b->CreateLShr(b->CreateLoad(ptr), cursorI64BlockRem);
                extractV = b->CreateAnd(extractV, literalMask);
                extractValues.push_back(extractV);
            }
        }

        this->appendBitStreamOutput(b, extractValues, literalLength);
    }


    llvm::Value *LZParabixAioKernel::processMatch(const std::unique_ptr<KernelBuilder> &b, llvm::Value* cursorPos, llvm::Value* matchOffset, llvm::Value* sequenceBasePtr) {
        Function* pdep = Intrinsic::getDeclaration(b->getModule(), Intrinsic::x86_bmi_pdep_64);
        Constant * PEXT_func = Intrinsic::getDeclaration(b->getModule(), Intrinsic::x86_bmi_pext_64);

        Value* pdepMask = b->getInt64(0x0101010101010101);

        Value* oneBitIndex = b->CreateZExt(b->CreateLoad(b->CreateGEP(sequenceBasePtr, b->getSize(1))), b->getInt64Ty());
        Value* zeroBitIndex = b->CreateZExt(b->CreateLoad(b->CreateGEP(sequenceBasePtr, b->getSize(2))), b->getInt64Ty());
        Value* matchMaskPtr = b->CreatePointerCast(b->CreateGEP(sequenceBasePtr, b->getSize(3)), b->getInt64Ty()->getPointerTo());
        Value* matchMask = b->CreateLoad(matchMaskPtr);

        Value* fullOneBitIndex = b->CreateCall(pdep, {oneBitIndex, pdepMask});
        Value* fullZeroBitIndex = b->CreateCall(pdep, {zeroBitIndex, pdepMask});


        Value* remainingMask = b->CreateNot(b->CreateMul(b->CreateOr(fullOneBitIndex, fullZeroBitIndex), b->getInt64(0xff)));
        Value* fullMatchMask = b->CreateOr(
                b->CreateMul(fullOneBitIndex, b->getInt64(0xff)),
                b->CreateCall(pdep, {matchMask, remainingMask})
        );

        Value* remCursorPos = b->CreateURem(cursorPos, b->getCapacity("outputStream0"));
        Value* cursorPosI64BlockIndex = b->CreateUDiv(remCursorPos, b->getSize(64));

        Value* remOutputPos = b->CreateURem(b->getScalarField("outputPos"), b->getCapacity("outputStream0"));
        Value* outputPosI64BlockIndex = b->CreateUDiv(remOutputPos, b->getSize(64));

        Value* matchCopyFromI64BlockIndex = b->CreateSub(outputPosI64BlockIndex, matchOffset);

        Value* matchCopyFromBitBlockIndex = b->CreateUDiv(matchCopyFromI64BlockIndex, b->getSize(4));
        matchCopyFromI64BlockIndex = b->CreateURem(matchCopyFromI64BlockIndex, b->getSize(4));

        std::vector<llvm::Value*> extractValues;
        unsigned iStreamIndex = 0;
        for (unsigned i = 0; i < mNumsOfBitStreams.size(); i++) {
            Value* outputBasePtr = b->CreatePointerCast(b->getRawOutputPointer("outputStream" + std::to_string(i), b->getSize(0)), b->getBitBlockType()->getPointerTo());
            Value* outputBlockBasePtr = b->CreateGEP(outputBasePtr, b->CreateMul(matchCopyFromBitBlockIndex, b->getSize(mNumsOfBitStreams[i])));
            outputBlockBasePtr = b->CreatePointerCast(outputBlockBasePtr, b->getInt64Ty()->getPointerTo());

            for (unsigned j = 0; j < mNumsOfBitStreams[i]; j++) {
                Value* ptr = b->CreateGEP(outputBlockBasePtr, b->CreateAdd(matchCopyFromI64BlockIndex, b->getSize(j * (b->getBitBlockWidth() / 64))));
                Value* value = b->CreateLoad(ptr);
                Value * extractV = b->CreateCall(PEXT_func, {value, fullMatchMask});
                extractValues.push_back(extractV);
                ++iStreamIndex;
            }
        }

        this->appendBitStreamOutput(b, extractValues, b->CreatePopcount(fullMatchMask));

        return b->CreateAdd(b->CreateSub(b->getSize(8), b->CreatePopcount(b->CreateOr(oneBitIndex, zeroBitIndex))), b->getSize(2));
    }


    // ---- Output
    void LZParabixAioKernel::initPendingOutputScalar(const std::unique_ptr<KernelBuilder> &b) {
        this->initPendingOutputScalar_BitStream(b);
//        this->initPendingOutputScalar_Swizzled(b);
    }

    void LZParabixAioKernel::appendBitStreamOutput(const std::unique_ptr<KernelBuilder> &b, std::vector<llvm::Value*>& extractedValues, llvm::Value* valueLength) {
        this->appendBitStreamOutput_BitStream(b, extractedValues, valueLength);
//        this->appendBitStreamOutput_Swizzled(b, extractedValues, valueLength);
    }

    void LZParabixAioKernel::storePendingOutput(const std::unique_ptr<KernelBuilder> &b) {
        this->storePendingOutput_BitStream(b);
//        this->storePendingOutput_Swizzled(b);
    }


    // ---- Output BitStream
    void LZParabixAioKernel::initPendingOutputScalar_BitStream(const std::unique_ptr<KernelBuilder> &b) {
        for (unsigned i = 0; i < mNumsOfBitStreams.size(); i++) {
            for (unsigned j = 0; j < mNumsOfBitStreams[i]; j++) {
                this->addScalar(b->getInt64Ty(), "pendingOutput" + std::to_string(i) + "_" + std::to_string(j));
            }
        }
    }

    void LZParabixAioKernel::appendBitStreamOutput_BitStream(const std::unique_ptr<KernelBuilder> &b, std::vector<llvm::Value*>& extractedValues, llvm::Value* valueLength) {
        BasicBlock* exitBlock = b->CreateBasicBlock("exitBlock");

        Value* oldOutputPos = b->getScalarField("outputPos");
        Value* oldOutputPosRem64 = b->CreateURem(oldOutputPos, b->getSize(64));

        std::vector<llvm::Value*> newOutputVec;

        unsigned iStreamIndex = 0;
        for (unsigned i = 0; i < mNumsOfBitStreams.size(); i++) {
            for (unsigned j = 0; j < mNumsOfBitStreams[i]; j++) {
                Value* newValue = b->CreateOr(b->getScalarField("pendingOutput" + std::to_string(i) + "_" + std::to_string(j)), b->CreateShl(extractedValues[iStreamIndex], oldOutputPosRem64));
                newOutputVec.push_back(newValue);
                ++iStreamIndex;
            }
        }

        BasicBlock* noStoreOutputBlock = b->CreateBasicBlock("noStoreOutputBlock");
        BasicBlock* storeOutputBlock =b->CreateBasicBlock("storeOutputBlock");

        b->CreateCondBr(b->CreateICmpULT(b->CreateAdd(oldOutputPosRem64, valueLength), b->getSize(64)), noStoreOutputBlock, storeOutputBlock);

        // ---- noStoreOutputBlock
        b->SetInsertPoint(noStoreOutputBlock);

        iStreamIndex = 0;
        for (unsigned i = 0; i < mNumsOfBitStreams.size(); i++) {
            for (unsigned j = 0; j < mNumsOfBitStreams[i]; j++) {
                b->setScalarField("pendingOutput" + std::to_string(i) + "_" + std::to_string(j), newOutputVec[iStreamIndex]);
                ++iStreamIndex;
            }
        }

        b->CreateBr(exitBlock);

        // ---- storeOutputBlock
        b->SetInsertPoint(storeOutputBlock);

        Value* oldOutputPosRem = b->CreateURem(oldOutputPos, b->getCapacity("outputStream0"));
        Value* oldOutputPosBitBlockIndex = b->CreateUDiv(oldOutputPosRem, b->getSize(b->getBitBlockWidth()));
        Value* oldOutputPosBitBlockRem = b->CreateURem(oldOutputPosRem, b->getSize(b->getBitBlockWidth()));

        iStreamIndex = 0;
        for (unsigned i = 0; i < mNumsOfBitStreams.size(); i++) {
            Value* outputBasePtr = b->CreatePointerCast(b->getRawOutputPointer("outputStream" + std::to_string(i), b->getSize(0)), b->getBitBlockType()->getPointerTo());
            Value* outputBitBlockBasePtr = b->CreateGEP(outputBasePtr, b->CreateMul(oldOutputPosBitBlockIndex, b->getSize(mNumsOfBitStreams[i])));
            outputBitBlockBasePtr = b->CreatePointerCast(outputBitBlockBasePtr, b->getInt64Ty()->getPointerTo());

            Value* oldOutputPosI64Index = b->CreateUDiv(oldOutputPosBitBlockRem, b->getSize(64));

            for (unsigned j = 0; j < mNumsOfBitStreams[i]; j++) {
                Value* targetPtr = b->CreateGEP(outputBitBlockBasePtr, b->CreateAdd(oldOutputPosI64Index, b->getSize(j * (b->getBitBlockWidth() / 64))));
                b->CreateStore(newOutputVec[iStreamIndex], targetPtr);
                ++iStreamIndex;
            }
        }

        Value* shiftAmount = b->CreateSub(b->getSize(0x40), oldOutputPosRem64);
        Value* fullyShift = b->CreateICmpEQ(shiftAmount, b->getSize(0x40));

        iStreamIndex = 0;
        for (unsigned i = 0; i < mNumsOfBitStreams.size(); i++) {
            for (unsigned j = 0; j < mNumsOfBitStreams[i]; j++) {
                b->setScalarField("pendingOutput" + std::to_string(i) + "_" + std::to_string(j), b->CreateSelect(fullyShift, b->getInt64(0), b->CreateLShr(extractedValues[iStreamIndex], shiftAmount)));
                ++iStreamIndex;
            }
        }

        b->CreateBr(exitBlock);

        b->SetInsertPoint(exitBlock);
        b->setScalarField("outputPos", b->CreateAdd(oldOutputPos, valueLength));
    }

    void LZParabixAioKernel::storePendingOutput_BitStream(const std::unique_ptr<KernelBuilder> &b) {
        Value* oldOutputPos = b->getScalarField("outputPos");
        Value* oldOutputPosRem = b->CreateURem(oldOutputPos, b->getCapacity("outputStream0"));
        Value* oldOutputPosBitBlockIndex = b->CreateUDiv(oldOutputPosRem, b->getSize(b->getBitBlockWidth()));
        Value* oldOutputPosBitBlockRem = b->CreateURem(oldOutputPosRem, b->getSize(b->getBitBlockWidth()));
        Value* oldOutputPosI64Index = b->CreateUDiv(oldOutputPosBitBlockRem, b->getSize(64));

        unsigned iStreamIndex = 0;
        for (unsigned i = 0; i < mNumsOfBitStreams.size(); i++) {
            Value* outputBasePtr = b->CreatePointerCast(b->getRawOutputPointer("outputStream" + std::to_string(i), b->getSize(0)), b->getBitBlockType()->getPointerTo());
            Value* outputBitBlockBasePtr = b->CreateGEP(outputBasePtr, b->CreateMul(oldOutputPosBitBlockIndex, b->getSize(mNumsOfBitStreams[i])));
            outputBitBlockBasePtr = b->CreatePointerCast(outputBitBlockBasePtr, b->getInt64Ty()->getPointerTo());
            for (unsigned j = 0; j < mNumsOfBitStreams[i]; j++) {
                Value* targetPtr = b->CreateGEP(outputBitBlockBasePtr, b->CreateAdd(oldOutputPosI64Index, b->getSize(j * (b->getBitBlockWidth() / 64))));
                b->CreateStore(b->getScalarField("pendingOutput" + std::to_string(i) + "_" + std::to_string(j)), targetPtr);
                ++iStreamIndex;
            }
        }
    }

    // ---- Output Swizzled
    void LZParabixAioKernel::initPendingOutputScalar_Swizzled(const std::unique_ptr<KernelBuilder> &b) {
        for (unsigned i = 0; i < (mNumsOfBitStreams[0] + 3) / 4; i++) {
            this->addScalar(b->getBitBlockType(), "pendingOutput" + std::to_string(0) + "_" + std::to_string(i));
        }
    }
    void LZParabixAioKernel::appendBitStreamOutput_Swizzled(const std::unique_ptr<KernelBuilder> &b, std::vector<llvm::Value*>& extractedValues, llvm::Value* valueLength) {

        std::vector<llvm::Value*> extractedValuesVec;
        for (unsigned i = 0; i < 2; i++) {
            Value* vec = ConstantVector::getNullValue(b->getBitBlockType());
            for (unsigned j = 0; j < 4; j++) {
                vec = b->CreateInsertElement(vec, extractedValues[i * 4 + j], j);
            }
            extractedValuesVec.push_back(vec);
        }

        BasicBlock* exitBlock = b->CreateBasicBlock("exitBlock");

        Value* oldOutputPos = b->getScalarField("outputPos");
        Value* oldOutputPosRem64 = b->CreateURem(oldOutputPos, b->getSize(64));

        std::vector<llvm::Value*> newOutputVec;
        for (unsigned i = 0; i < 2; i++) {
            Value* newValue = b->CreateOr(b->getScalarField("pendingOutput" + std::to_string(0) + "_" + std::to_string(i)), b->CreateShl(extractedValuesVec[i], b->simd_fill(64, oldOutputPosRem64)));
            newOutputVec.push_back(newValue);
        }


        BasicBlock* noStoreOutputBlock = b->CreateBasicBlock("noStoreOutputBlock");
        BasicBlock* storeOutputBlock =b->CreateBasicBlock("storeOutputBlock");

        b->CreateCondBr(b->CreateICmpULT(b->CreateAdd(oldOutputPosRem64, valueLength), b->getSize(64)), noStoreOutputBlock, storeOutputBlock);

        // ---- noStoreOutputBlock
        b->SetInsertPoint(noStoreOutputBlock);
        for (unsigned i = 0; i < 2; i++) {
            b->setScalarField("pendingOutput" + std::to_string(0) + "_" + std::to_string(i), newOutputVec[i]);
        }
        b->CreateBr(exitBlock);

        // ---- storeOutputBlock
        b->SetInsertPoint(storeOutputBlock);

        Value* oldOutputPosRem = b->CreateURem(oldOutputPos, b->getCapacity("outputStream0"));
        Value* oldOutputPosBitBlockIndex = b->CreateUDiv(oldOutputPosRem, b->getSize(b->getBitBlockWidth()));
        Value* oldOutputPosBitBlockRem = b->CreateURem(oldOutputPosRem, b->getSize(b->getBitBlockWidth()));

        Value* outputBasePtr = b->CreatePointerCast(b->getRawOutputPointer("outputStream0", b->getSize(0)), b->getBitBlockType()->getPointerTo());
        Value* outputBitBlockBasePtr = b->CreateGEP(outputBasePtr, b->CreateMul(oldOutputPosBitBlockIndex, b->getSize(8)));
        outputBitBlockBasePtr = b->CreatePointerCast(outputBitBlockBasePtr, b->getInt64Ty()->getPointerTo());

        Value* oldOutputPosI64Index = b->CreateUDiv(oldOutputPosBitBlockRem, b->getSize(64));

        for (unsigned i = 0; i < 2; i++) {
            for (unsigned j = 0; j < 4; j++) {
                Value* targetPtr = b->CreateGEP(outputBitBlockBasePtr, b->CreateAdd(oldOutputPosI64Index, b->getSize((i * 4 + j) * 4)));
                b->CreateStore(b->CreateExtractElement(newOutputVec[i], j), targetPtr);
            }

        }

        Value* shiftAmount = b->CreateSub(b->getSize(0x40), oldOutputPosRem64);
        Value* fullyShift = b->CreateICmpEQ(shiftAmount, b->getSize(0x40));

        for (unsigned i = 0; i < 2; i++) {

            b->setScalarField("pendingOutput" + std::to_string(0) + "_" + std::to_string(i), b->CreateSelect(fullyShift, ConstantVector::getNullValue(b->getBitBlockType()), b->CreateLShr(extractedValuesVec[i], b->simd_fill(64, shiftAmount))));
        }

        b->CreateBr(exitBlock);

        b->SetInsertPoint(exitBlock);
        b->setScalarField("outputPos", b->CreateAdd(oldOutputPos, valueLength));

    }

    void LZParabixAioKernel::storePendingOutput_Swizzled(const std::unique_ptr<KernelBuilder> &b) {
        Value* oldOutputPos = b->getScalarField("outputPos");
        Value* oldOutputPosRem = b->CreateURem(oldOutputPos, b->getCapacity("outputStream0"));
        Value* oldOutputPosBitBlockIndex = b->CreateUDiv(oldOutputPosRem, b->getSize(b->getBitBlockWidth()));
        Value* oldOutputPosBitBlockRem = b->CreateURem(oldOutputPosRem, b->getSize(b->getBitBlockWidth()));

        Value* oldOutputPosI64Index = b->CreateUDiv(oldOutputPosBitBlockRem, b->getSize(64));

        Value* outputBasePtr = b->CreatePointerCast(b->getRawOutputPointer("outputStream0", b->getSize(0)), b->getBitBlockType()->getPointerTo());
        Value* outputBitBlockBasePtr = b->CreateGEP(outputBasePtr, b->CreateMul(oldOutputPosBitBlockIndex, b->getSize(8)));
        outputBitBlockBasePtr = b->CreatePointerCast(outputBitBlockBasePtr, b->getInt64Ty()->getPointerTo());

        vector<Value*> pendingOutputVec;
        for (unsigned i = 0; i < 2; i++) {
            pendingOutputVec.push_back(b->getScalarField("pendingOutput" + std::to_string(0) + "_" + std::to_string(i)));
        }

        for (unsigned i = 0; i < 2; i++) {
            for (unsigned j = 0; j < 2; j++) {
                Value* targetPtr = b->CreateGEP(outputBitBlockBasePtr, b->CreateAdd(oldOutputPosI64Index, b->getSize((i * 4 + j) * 4)));
                b->CreateStore(b->CreateExtractElement(pendingOutputVec[i], j), targetPtr);
            }
        }
    }
}