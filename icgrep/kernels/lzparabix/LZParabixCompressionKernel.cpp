
#include "LZParabixCompressionKernel.h"

#include <kernels/kernel_builder.h>
#include <iostream>
#include <string>
#include <llvm/Support/raw_ostream.h>
#include <kernels/streamset.h>
#include <cstdint>


#define HASHLOG (12)
#define HASHNBCELLS4 (1 << HASHLOG)


using namespace llvm;
using namespace kernel;
using namespace std;

namespace kernel{




    LZParabixCompressionKernel::LZParabixCompressionKernel(const std::unique_ptr<kernel::KernelBuilder> &b)
    :SegmentOrientedKernel("LZParabixAioKernel",
            // Inputs
                           {
                                   Binding{b->getStreamSetTy(1, 8), "byteStream", FixedRate()},
                           },
            //Outputs
                           {
                                   Binding{b->getStreamSetTy(1, 8), "outputStream", BoundedRate(0, 1)}
                           },
            //Arguments
                           {
                                   Binding{b->getSizeTy(), "fileSize"}
                           },
                           {},
            //Internal states:
                           {
                                   // Two hashmap to optimize the performance of matching
                                   Binding(ArrayType::get(b->getInt64Ty(), HASHNBCELLS4), "strToBlockIndex"),
                                   Binding(ArrayType::get(b->getInt32Ty(), HASHNBCELLS4), "strToMatchPos")

                           }),
     mLzParabixBlockSize(4 * 1024 * 1024)
    {
        this->setStride(mLzParabixBlockSize);
        addAttribute(MustExplicitlyTerminate());
    }

    void LZParabixCompressionKernel::generateDoSegmentMethod(const std::unique_ptr<KernelBuilder> &b) {
        Value* inputCursor = b->getProcessedItemCount("byteStream");
        Value* inputEndPos = b->CreateAdd(inputCursor, b->getSize(mLzParabixBlockSize));
        Value* fileSize = b->getScalarField("fileSize");
        inputEndPos = b->CreateUMin(inputEndPos, fileSize);
        Value* outputCursor = b->getProducedItemCount("outputStream");
        this->encodeBlock(b, inputCursor, inputEndPos, outputCursor);

        b->setTerminationSignal(b->CreateICmpEQ(inputEndPos, fileSize));
        b->setProcessedItemCount("byteStream", inputEndPos);
    }

    void LZParabixCompressionKernel::encodeBlock(const std::unique_ptr<KernelBuilder> &b, llvm::Value *inputCursor,
                                                 llvm::Value *inputEndPos, llvm::Value *outputPos) {
        // Constant
        ConstantInt* SIZE_64 = b->getSize(64);

        // ---- EntryBlock
        BasicBlock* entryBlock = b->GetInsertBlock();

        BasicBlock* encodeBlockCon = b->CreateBasicBlock("encodeBlockCon");
        BasicBlock* encodeBlockBody = b->CreateBasicBlock("encodeBlockBody");
        BasicBlock* exitBlock = b->CreateBasicBlock("exitBlock");

        Value* blockSizePos = outputPos;

        Value* initOutputCursor = b->CreateAdd(outputPos, b->getSize(4)); // blockSize takes 4 bytes
        Value* initBlockIndex = b->CreateUDiv(inputCursor, b->getSize(64));


        b->CreateBr(encodeBlockCon);

        // ---- encodeBlockCon
        b->SetInsertPoint(encodeBlockCon);

        PHINode* phiInputCursorPos = b->CreatePHI(b->getSizeTy(), 3);
        phiInputCursorPos->addIncoming(inputCursor, entryBlock);
        PHINode* phiPreviousInputCursorPos = b->CreatePHI(b->getSizeTy(), 3);
        phiPreviousInputCursorPos->addIncoming(inputCursor, entryBlock);
        PHINode* phiPreviousBlockIndex = b->CreatePHI(b->getSizeTy(), 3);
        phiPreviousBlockIndex->addIncoming(initBlockIndex, entryBlock);
        PHINode* phiOutputCursorPos = b->CreatePHI(b->getSizeTy(), 3);
        phiOutputCursorPos->addIncoming(initOutputCursor, entryBlock);

        b->CreateCondBr(
                b->CreateICmpULT(phiInputCursorPos, inputEndPos),
                encodeBlockBody,
                exitBlock
        );

        // ---- encodeBlockBody
        b->SetInsertPoint(encodeBlockBody);

        BasicBlock* updateCacheBlock = b->CreateBasicBlock("updateCacheBlock");
        BasicBlock* extractMatchInfoBlock = b->CreateBasicBlock("extractMatchInfoBlock");

        Value* newBlockIndex = b->CreateUDiv(phiInputCursorPos, SIZE_64);
        b->CreateCondBr(
                b->CreateICmpNE(newBlockIndex, phiPreviousBlockIndex),
                updateCacheBlock,
                extractMatchInfoBlock
        );

        // ---- updateCacheBlock
        b->SetInsertPoint(updateCacheBlock);

        this->updateCache(b, phiPreviousBlockIndex, initBlockIndex);

        b->CreateBr(extractMatchInfoBlock);

        // ---- encodeProcessBlock
        b->SetInsertPoint(extractMatchInfoBlock);

        MatchInfo retMatchInfo = this->extractMatchInfo(b, phiInputCursorPos, initBlockIndex, inputEndPos);
//        b->CallPrintInt("matchLength", retMatchInfo.matchLength);
        BasicBlock* noAppendOutputBlock = b->CreateBasicBlock("noAppendOutputBlock");
        BasicBlock* appendOutputBlock = b->CreateBasicBlock("appendOutputBlock");

        b->CreateCondBr(
                b->CreateICmpUGE(
                    retMatchInfo.matchLength,
                    b->getSize(9)
                ),
                appendOutputBlock,
                noAppendOutputBlock
        );

        // ---- appendOutputBlock
        b->SetInsertPoint(appendOutputBlock);

        Value* outputCursorPosAfterLiteral = this->appendLiteralSequence(b, phiPreviousInputCursorPos, phiInputCursorPos, phiOutputCursorPos);
        Value* outputCursorPosAfterMatch = this->appendMatchSequence(b, retMatchInfo, outputCursorPosAfterLiteral);
        Value* nextCursorPos = b->CreateAdd(phiInputCursorPos, retMatchInfo.matchLength);

        phiInputCursorPos->addIncoming(nextCursorPos, b->GetInsertBlock());
        phiPreviousInputCursorPos->addIncoming(nextCursorPos, b->GetInsertBlock());
        phiPreviousBlockIndex->addIncoming(newBlockIndex, b->GetInsertBlock());
        phiOutputCursorPos->addIncoming(outputCursorPosAfterMatch, b->GetInsertBlock());
        b->CreateBr(encodeBlockCon);


        // ---- noAppendOutputBlock
        b->SetInsertPoint(noAppendOutputBlock);

        Value* newInputCursorPos = b->CreateSelect(
                b->CreateICmpEQ(phiPreviousInputCursorPos, phiInputCursorPos),
                b->CreateAdd(phiInputCursorPos, b->getSize(8)),
                b->CreateAdd(phiInputCursorPos, b->getSize(1))
        );
        newInputCursorPos = b->CreateUMin(newInputCursorPos, inputEndPos);

        phiInputCursorPos->addIncoming(newInputCursorPos, b->GetInsertBlock());
        phiPreviousInputCursorPos->addIncoming(phiPreviousInputCursorPos, b->GetInsertBlock());
        phiPreviousBlockIndex->addIncoming(newBlockIndex, b->GetInsertBlock());
        phiOutputCursorPos->addIncoming(phiOutputCursorPos, b->GetInsertBlock());
        b->CreateBr(encodeBlockCon);

        // ---- exitBlock
        b->SetInsertPoint(exitBlock);


        Value* finalOutputPos = this->appendLiteralSequence(b, phiPreviousInputCursorPos, inputEndPos, phiOutputCursorPos);
        Value* blockSize = b->CreateSub(finalOutputPos, initOutputCursor);
//        blockSizePos
        Value* outputBasePtr = b->CreatePointerCast(b->getRawOutputPointer("outputStream", b->getSize(0)), b->getInt8Ty()->getPointerTo());
        b->CreateStore(b->CreateTrunc(blockSize, b->getInt32Ty()), b->CreatePointerCast(b->CreateGEP(outputBasePtr, b->CreateURem(blockSizePos, b->getCapacity("outputStream"))), b->getInt32Ty()->getPointerTo()));

        b->setProducedItemCount("outputStream", finalOutputPos);

    }

    void LZParabixCompressionKernel::updateCache(const std::unique_ptr<KernelBuilder> &b, llvm::Value *i64BlockGlobalIndex, llvm::Value *initBlockGlobalIndex) {
        // Constant
        ConstantInt* SIZE_64 = b->getSize(64);
        ConstantInt* INT_64_16 = b->getInt64(16);
        ConstantInt* INT_32_8 = b->getInt32(8);

        // Type
        Type* i64PtrTy = b->getInt64Ty()->getPointerTo();
        Type* i32PtrTy = b->getInt32Ty()->getPointerTo();

        Value* localI64BlockIndex = b->CreateSub(i64BlockGlobalIndex, initBlockGlobalIndex);


        Value* inputBasePtr = b->CreatePointerCast(b->getRawInputPointer("byteStream", b->getSize(0)), b->getInt8PtrTy());

        Value* startPos = b->CreateMul(i64BlockGlobalIndex, SIZE_64);
        for (unsigned i = 0; i + 4 < 64; ++i) {
            Value* targetPos = b->CreateAdd(startPos, b->getSize(i));
            Value* valuePtr = b->CreatePointerCast(b->CreateGEP(inputBasePtr, targetPos), i32PtrTy);
            Value* inputValue = b->CreateLoad(valuePtr);

            Value* hashedKey = this->hashKey(b, inputValue);

            // Update mStrToBlockIndex
            Value* strToBlockIndexBasePtr = b->CreatePointerCast(b->getScalarFieldPtr("strToBlockIndex"), i64PtrTy);
            Value* strToBlockIndexTargetPtr = b->CreateGEP(strToBlockIndexBasePtr, hashedKey);
            Value* newBlockIndexCacheValue = b->CreateOr(b->CreateShl(b->CreateLoad(strToBlockIndexTargetPtr), INT_64_16), localI64BlockIndex);
            b->CreateStore(newBlockIndexCacheValue, strToBlockIndexTargetPtr);

            // Update mStrToMatchPos
            Value* strToMatchPosBasePtr = b->CreatePointerCast(b->getScalarFieldPtr("strToMatchPos"), i32PtrTy);
            Value* strToMatchPosTargetPtr = b->CreateGEP(strToMatchPosBasePtr, hashedKey);
            Value* newMatchPosCacheValue = b->CreateOr(b->CreateShl(b->CreateLoad(strToMatchPosTargetPtr), INT_32_8), b->getInt32(i));
            b->CreateStore(newMatchPosCacheValue, strToMatchPosTargetPtr);
        }
    }

    llvm::Value *LZParabixCompressionKernel::hashKey(const std::unique_ptr<KernelBuilder> &b, llvm::Value *v) {
        //  (((v) * 2654435761U) >> ((4 * 8) - HASHLOG))
        ConstantInt* magicNumber = b->getInt32(2654435761); // 2654435761 here is a Magic Number for hash
        return b->CreateLShr(
                b->CreateMul(v, magicNumber),
                b->getInt32(4 * 8 - HASHLOG)
        );
    }



    MatchInfo LZParabixCompressionKernel::extractMatchInfo(const std::unique_ptr<KernelBuilder> &b, llvm::Value *cursorPos, llvm::Value *initBlockGlobalIndex, llvm::Value* inputEndPos) {
        // ---- EntryBlock
        Value* SIZE_0 = b->getSize(0);
        MatchInfo retInfo = {SIZE_0, SIZE_0, SIZE_0, SIZE_0};

        BasicBlock* entryBlock = b->GetInsertBlock();
        BasicBlock* exitBlock = b->CreateBasicBlock("exitBlock");

        Value* inputBasePtr = b->CreatePointerCast(b->getRawInputPointer("byteStream", b->getSize(0)), b->getInt8PtrTy());

        auto retMatchInfo = this->getPossibleMatchInfo(b, cursorPos);

        Value* possibleBlockIndexes = retMatchInfo.first;
        Value* possibleMatchPos = retMatchInfo.second;

        Value* currentGlobalInputBlockIndex = b->CreateUDiv(cursorPos, b->getSize(64));



        for (unsigned i = 0; i < 4; i++) {
            BasicBlock* extractMatchInfoEntryBlock = b->CreateBasicBlock("extractMatchInfoEntryBlock");
            b->CreateBr(extractMatchInfoEntryBlock);

            // ---- extractMatchInfoEntryBlock
            b->SetInsertPoint(extractMatchInfoEntryBlock);


            Value* localMatchBlockIndex = b->CreateAnd(b->CreateLShr(possibleBlockIndexes, b->getInt64(i * 16)), b->getInt64(0xffff));
            Value* globalMatchBlockIndex = b->CreateAdd(localMatchBlockIndex, initBlockGlobalIndex);

            Value* matchPos = b->CreateAnd(b->CreateLShr(possibleMatchPos, b->getInt32(i * 8)), b->getInt32(0xff));
            matchPos = b->CreateZExt(matchPos, b->getInt64Ty());

            Value* blockOffset = b->CreateSub(currentGlobalInputBlockIndex, globalMatchBlockIndex);

            Value* isNoMatch = b->CreateOr(
                    b->CreateICmpUGE(globalMatchBlockIndex, currentGlobalInputBlockIndex),
                    b->CreateICmpUGE(blockOffset, b->getSize(128))
            );

//            b->CallPrintInt("isNoMatch", isNoMatch);

            BasicBlock* scanMatchConBlock = b->CreateBasicBlock("scanMatchConBlock");
            BasicBlock* scanMatchBodyBlock = b->CreateBasicBlock("scanMatchBodyBlock");
            BasicBlock* scanMatchExitBlock = b->CreateBasicBlock("scanMatchExitBlock");

//            b->CreateCondBr(isNoMatch, scanMatchExitBlock, scanMatchConBlock);
            b->CreateBr(scanMatchConBlock);

            // ---- scanMatchConBlock
            b->SetInsertPoint(scanMatchConBlock);

            PHINode* phiForwardMatchLength = b->CreatePHI(b->getSizeTy(), 2);
            phiForwardMatchLength->addIncoming(b->getSize(0), extractMatchInfoEntryBlock);
            PHINode* phiMatchPos = b->CreatePHI(b->getSizeTy(), 2);
            phiMatchPos->addIncoming(matchPos, extractMatchInfoEntryBlock);
            PHINode* phiMatchMask = b->CreatePHI(b->getInt64Ty(), 2);
            phiMatchMask->addIncoming(b->getInt64(0), extractMatchInfoEntryBlock);

            b->CreateCondBr(
                    b->CreateAnd(
                            b->CreateAnd(
                                    b->CreateICmpULT(phiMatchPos, b->getInt64(64)),
                                    b->CreateICmpULT(b->CreateAdd(cursorPos, phiForwardMatchLength), inputEndPos)
                            ),
                            b->CreateNot(isNoMatch)
                    ),
                    scanMatchBodyBlock,
                    scanMatchExitBlock
            );

            // ---- scanMatchBodyBlock
            b->SetInsertPoint(scanMatchBodyBlock);

            Value* isMatch = b->CreateICmpEQ(
                    b->CreateLoad(b->CreateGEP(inputBasePtr, b->CreateAdd(b->CreateMul(globalMatchBlockIndex, b->getInt64(64)), phiMatchPos))),
                    b->CreateLoad(b->CreateGEP(inputBasePtr, b->CreateAdd(cursorPos, phiForwardMatchLength)))
            );


            phiForwardMatchLength->addIncoming(
                    b->CreateSelect(isMatch, b->CreateAdd(phiForwardMatchLength, b->getSize(1)), phiForwardMatchLength),
                    b->GetInsertBlock()
            );
            phiMatchPos->addIncoming(
                    b->CreateAdd(phiMatchPos, b->getSize(1)),
                    b->GetInsertBlock()
            );

            phiMatchMask->addIncoming(
                    b->CreateSelect(
                            isMatch,
                            b->CreateOr(phiMatchMask, b->CreateShl(b->getInt64(1), phiMatchPos)),
                            phiMatchMask
                    ),
                    b->GetInsertBlock()
            );

            b->CreateBr(scanMatchConBlock);

            // ---- scanMatchExitBlock
            b->SetInsertPoint(scanMatchExitBlock);

            Value* isBetterMatch = b->CreateICmpUGT(phiForwardMatchLength, retInfo.matchLength);
            retInfo.matchLength = b->CreateSelect(isBetterMatch, phiForwardMatchLength, retInfo.matchLength);
            retInfo.matchMask = b->CreateSelect(isBetterMatch, phiMatchMask, retInfo.matchMask);
            retInfo.matchOffset = b->CreateSelect(isBetterMatch, blockOffset, retInfo.matchOffset);
            retInfo.matchStart = b->CreateSelect(isBetterMatch, cursorPos, retInfo.matchOffset);

            // TODO search back and adjust match start
        }
        b->CreateBr(exitBlock);
        b->SetInsertPoint(exitBlock);

        return retInfo;

    }

    std::pair<llvm::Value *, llvm::Value *>
    LZParabixCompressionKernel::getPossibleMatchInfo(const std::unique_ptr<KernelBuilder> &b, llvm::Value *cursorPos) {
        // Type
        Type* i64PtrTy = b->getInt64Ty()->getPointerTo();
        Type* i32PtrTy = b->getInt32Ty()->getPointerTo();

        Value* inputBasePtr = b->CreatePointerCast(b->getRawInputPointer("byteStream", b->getSize(0)), b->getInt8PtrTy());
        Value* targetPtr = b->CreateGEP(inputBasePtr, cursorPos);
        targetPtr = b->CreatePointerCast(targetPtr, b->getInt32Ty()->getPointerTo());
        Value* hashedKey = this->hashKey(b, b->CreateLoad(targetPtr));
//        b->CallPrintInt("hashedKey", hashedKey);
        Value* strToBlockIndexBasePtr = b->CreatePointerCast(b->getScalarFieldPtr("strToBlockIndex"), i64PtrTy);
        Value* strToBlockIndexTargetPtr = b->CreateGEP(strToBlockIndexBasePtr, hashedKey);

        Value* strToMatchPosBasePtr = b->CreatePointerCast(b->getScalarFieldPtr("strToMatchPos"), i32PtrTy);
        Value* strToMatchPosTargetPtr = b->CreateGEP(strToMatchPosBasePtr, hashedKey);

        return std::make_pair(
                b->CreateLoad(strToBlockIndexTargetPtr),
                b->CreateLoad(strToMatchPosTargetPtr)
        );
    }

    Value *LZParabixCompressionKernel::appendLiteralSequence(const unique_ptr<KernelBuilder> &b, Value *literalStart,
                                                             Value *literalEnd, Value *outputPos) {
        // Constant
        Value* SIZE_0 = b->getSize(0);
        Value* SIZE_1 = b->getSize(1);
        Value* SIZE_64 = b->getSize(64);
        Type* i8PtrTy = b->getInt8PtrTy();


        Value* inputBasePtr = b->CreatePointerCast(b->getRawInputPointer("byteStream", SIZE_0), i8PtrTy);
        Value* outputBasePtr = b->CreatePointerCast(b->getRawOutputPointer("outputStream", SIZE_0), i8PtrTy);
        Value* outputCapacity = b->getCapacity("outputStream");


        // ---- EntryBlock
        BasicBlock* entryBlock = b->GetInsertBlock();
        BasicBlock* appendLiteralConBlock = b->CreateBasicBlock("AppendLiteralConBlock");
        BasicBlock* appendLiteralBodyBlock = b->CreateBasicBlock("AppendLiteralBodyBlock");
        BasicBlock* appendLiteralExitBlock = b->CreateBasicBlock("AppendLiteralExitBlock");

        b->CreateBr(appendLiteralConBlock);

        // ---- AppendLiteralConBlock
        b->SetInsertPoint(appendLiteralConBlock);
        PHINode* phiOutputPos = b->CreatePHI(b->getSizeTy(), 2);
        phiOutputPos->addIncoming(outputPos, entryBlock);
        PHINode* phiLiteralStart = b->CreatePHI(b->getSizeTy(), 2);
        phiLiteralStart->addIncoming(literalStart, entryBlock);

        Value* remainingLiteralLength = b->CreateSub(literalEnd, phiLiteralStart);

        b->CreateCondBr(b->CreateICmpUGT(remainingLiteralLength, b->getSize(0)), appendLiteralBodyBlock, appendLiteralExitBlock);

        // ---- AppendLiteralBodyBlock
        b->SetInsertPoint(appendLiteralBodyBlock);

        Value* outputPosRem = b->CreateURem(phiOutputPos, outputCapacity);
        Value* outputNextPosRem = b->CreateAdd(outputPosRem, b->getSize(1));

        Value* maxLiteralLength = b->CreateSub(SIZE_64, b->CreateURem(outputNextPosRem, SIZE_64));
        Value* literalLength = b->CreateUMin(remainingLiteralLength, maxLiteralLength);

        Value* literalToken = b->CreateOr(b->CreateTrunc(literalLength, b->getInt8Ty()), b->getInt8(1 << 7));

        b->CreateStore(literalToken, b->CreateGEP(outputBasePtr, outputPosRem));
        b->CreateMemCpy(
                b->CreateGEP(outputBasePtr, outputNextPosRem),
                b->CreateGEP(inputBasePtr, phiLiteralStart),
                literalLength,
                1
        );

        phiLiteralStart->addIncoming(b->CreateAdd(phiLiteralStart, literalLength), b->GetInsertBlock());
        phiOutputPos->addIncoming(b->CreateAdd(phiOutputPos, b->CreateAdd(literalLength, SIZE_1)), b->GetInsertBlock());
        b->CreateBr(appendLiteralConBlock);

        // ---- AppendLiteralExitBlock
        b->SetInsertPoint(appendLiteralExitBlock);

        return phiOutputPos;
    }

    Value* LZParabixCompressionKernel::appendMatchSequence(const unique_ptr<KernelBuilder> &b, const MatchInfo& matchInfo, Value* outputPos) {
        // Constant
        Value* SIZE_0 = b->getSize(0);
        Value* SIZE_1 = b->getSize(1);
        Value* SIZE_64 = b->getSize(64);
        Type* i8PtrTy = b->getInt8PtrTy();

        Value* outputBasePtr = b->CreatePointerCast(b->getRawOutputPointer("outputStream", SIZE_0), i8PtrTy);
        Value* outputCapacity = b->getCapacity("outputStream");

        // MatchOffset
        b->CreateStore(b->CreateTrunc(matchInfo.matchOffset, b->getInt8Ty()), b->CreateGEP(outputBasePtr, b->CreateURem(outputPos, outputCapacity)));

        Value* outputOneBitIndexPos = b->CreateAdd(outputPos, SIZE_1);
        Value* outputZeroBitIndexPos = b->CreateAdd(outputOneBitIndexPos, SIZE_1);

        Value* outputBitMaskPos = b->CreateAdd(outputZeroBitIndexPos, SIZE_1);

        Value* oneBitIndex = b->getInt8(0);
        Value* zeroBitIndex = b->getInt8(0);

        for (unsigned i = 0; i < 8; i++) {
            Value* currentBitIndex = b->CreateTrunc(b->CreateLShr(matchInfo.matchMask, b->getInt64(i * 8)), b->getInt8Ty());
            Value* newOneBit = b->CreateICmpEQ(currentBitIndex, b->getInt8(0xff));
            Value* newZeroBit = b->CreateICmpEQ(currentBitIndex, b->getInt8(0));

            oneBitIndex = b->CreateSelect(
                    newOneBit,
                    b->CreateOr(oneBitIndex, b->getInt8(1 << i)),
                    oneBitIndex
            );

            zeroBitIndex = b->CreateSelect(
                    newZeroBit,
                    b->CreateOr(zeroBitIndex, b->getInt8(1 << i)),
                    zeroBitIndex
            );
            b->CreateStore(currentBitIndex, b->CreateGEP(outputBasePtr, b->CreateURem(outputBitMaskPos, outputCapacity)));
            outputBitMaskPos = b->CreateSelect(
                    b->CreateOr(newOneBit, newZeroBit),
                    outputBitMaskPos,
                    b->CreateAdd(outputBitMaskPos, SIZE_1)
            );
        }
        b->CreateStore(oneBitIndex, b->CreateGEP(outputBasePtr, b->CreateURem(outputOneBitIndexPos, outputCapacity)));
        b->CreateStore(zeroBitIndex, b->CreateGEP(outputBasePtr, b->CreateURem(outputZeroBitIndexPos, outputCapacity)));

        return outputBitMaskPos;
    }

}