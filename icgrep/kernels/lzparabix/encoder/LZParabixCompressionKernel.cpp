
#include "LZParabixCompressionKernel.h"

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
    LZParabixCompressionKernel::LZParabixCompressionKernel(const std::unique_ptr<kernel::KernelBuilder> &b)
    :SegmentOrientedKernel("LZParabixAioKernel",
            // Inputs
                           {
                                   Binding{b->getStreamSetTy(1, 8), "byteStream", FixedRate()},
                                   Binding{b->getStreamSetTy(8, 1), "basisBits", RateEqualTo("byteStream")}
                           },
            //Outputs
                           {
                                   Binding{b->getStreamSetTy(1, 8), "outputStream", BoundedRate(0, 1)},


                                   // TODO: workaround here, it seems that allocating large memory buffer in internal scalar fields will largely slow down
                                   // the performance of JIT compiling, we temporarily use 4 output buffer as internal scalar field

                                   // Two hashmap buffer to optimize the performance of matching
                                   Binding{b->getStreamSetTy(1, 64), "strToBlockIndex", BoundedRate(0, 1)},
                                   Binding{b->getStreamSetTy(1, 32), "strToMatchPos", BoundedRate(0, 1)},
                                   // Two mLzParabixBlockSize helper buffer
                                   Binding{b->getStreamSetTy(1, 8), "literalByteBuffer", BoundedRate(0, 1)},
                                   Binding{b->getStreamSetTy(1, 8), "tokenByteBuffer", BoundedRate(0, 1)}

                           },
            //Arguments
                           {
                                   Binding{b->getSizeTy(), "fileSize"}
                           },
                           {},
            //Internal states:
                           {
                                   Binding{ArrayType::get(b->getInt64Ty(), 8), "pendingLiteralOutput"},
                                   Binding{b->getSizeTy(), "literalOutputPos"},

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
        Value* literalSizePos = b->CreateAdd(outputPos, b->getSize(4));

        Value* initOutputCursor = b->CreateAdd(outputPos, b->getSize(8)); // blockSize takes 4 bytes, and literalSize takes 4 bytes
        Value* initBlockIndex = b->CreateUDiv(inputCursor, b->getSize(64));
        b->setScalarField("literalOutputPos", b->getSize(0));

        b->CreateMemSet(b->getScalarFieldPtr("pendingLiteralOutput"), b->getInt8(0), b->getSize(64), 1);

        b->CreateBr(encodeBlockCon);

        // ---- encodeBlockCon
        b->SetInsertPoint(encodeBlockCon);

        PHINode* phiInputCursorPos = b->CreatePHI(b->getSizeTy(), 3);
        phiInputCursorPos->addIncoming(inputCursor, entryBlock);
        PHINode* phiPreviousInputCursorPos = b->CreatePHI(b->getSizeTy(), 3);
        phiPreviousInputCursorPos->addIncoming(inputCursor, entryBlock);
        PHINode* phiPreviousBlockIndex = b->CreatePHI(b->getSizeTy(), 3);
        phiPreviousBlockIndex->addIncoming(initBlockIndex, entryBlock);

        PHINode* phiTokenOutputCursorPos = b->CreatePHI(b->getSizeTy(), 3);
        phiTokenOutputCursorPos->addIncoming(b->getSize(0), entryBlock);

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

        MatchInfo retMatchInfo = this->extractMatchInfo(b, phiInputCursorPos, initBlockIndex, inputEndPos, phiPreviousInputCursorPos);

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
        Value* tokenOutputPosAfterLiteralCopy = this->appendLiteralSequence(b, phiPreviousInputCursorPos, retMatchInfo.matchStart, phiTokenOutputCursorPos);

        Value* tokenOutputCursorPosAfterMatchCopy = this->appendMatchSequence(b, retMatchInfo, tokenOutputPosAfterLiteralCopy);
        Value* nextCursorPos = b->CreateAdd(retMatchInfo.matchStart, retMatchInfo.matchLength);

        phiInputCursorPos->addIncoming(nextCursorPos, b->GetInsertBlock());
        phiPreviousInputCursorPos->addIncoming(nextCursorPos, b->GetInsertBlock());
        phiPreviousBlockIndex->addIncoming(newBlockIndex, b->GetInsertBlock());
        phiTokenOutputCursorPos->addIncoming(tokenOutputCursorPosAfterMatchCopy, b->GetInsertBlock());
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
        phiTokenOutputCursorPos->addIncoming(phiTokenOutputCursorPos, b->GetInsertBlock());

        b->CreateBr(encodeBlockCon);

        // ---- exitBlock
        b->SetInsertPoint(exitBlock);

        Value* totalTokenLength = this->appendLiteralSequence(b, phiPreviousInputCursorPos, inputEndPos, phiTokenOutputCursorPos);
        Value* totalLiteralLength = b->getScalarField("literalOutputPos");
        this->storePendingLiteralData(b);
        // total literal length will be wrap to bit block width (we use 256 temporarily, will be 512 in the future)
        Value* SIZE_BIT_BLOCK_WIDTH = b->getSize(b->getBitBlockWidth());
        totalLiteralLength = b->CreateMul(
                b->CreateUDiv(
                        b->CreateAdd(totalLiteralLength, b->CreateSub(SIZE_BIT_BLOCK_WIDTH, b->getSize(1))),
                        SIZE_BIT_BLOCK_WIDTH
                ),
                SIZE_BIT_BLOCK_WIDTH
        );

//        b->CallPrintInt("totalLiteralLength", totalLiteralLength);
        // Store Output

        Value* blockSize = b->CreateAdd(b->CreateAdd(totalLiteralLength, totalTokenLength), b->getSize(4));

        Value* outputBasePtr = b->CreatePointerCast(b->getRawOutputPointer("outputStream", b->getSize(0)), b->getInt8Ty()->getPointerTo());
        Value* outputCapacity = b->getCapacity("outputStream");

        // TODO handle edge case when part of the block size or literal size is at the edge of output buffr
        b->CreateStore(b->CreateTrunc(blockSize, b->getInt32Ty()), b->CreatePointerCast(b->CreateGEP(outputBasePtr, b->CreateURem(blockSizePos, outputCapacity)), b->getInt32Ty()->getPointerTo()));
        b->CreateStore(b->CreateTrunc(totalLiteralLength, b->getInt32Ty()), b->CreatePointerCast(b->CreateGEP(outputBasePtr, b->CreateURem(literalSizePos, outputCapacity)), b->getInt32Ty()->getPointerTo()));


        Value* SIZE_0 = b->getSize(0);
        Type* i8PtrTy = b->getInt8PtrTy();

        // Literal Part will be BitStream
        // Copy Literal Part
        // copy literal buffer to [initOutputCursor, initOutputCursor + literalLength)
        Value* literalPartBeginPos = initOutputCursor;
        Value* literalPartBeginPosRem = b->CreateURem(literalPartBeginPos, outputCapacity);
        Value* literalOutputBasePtr = b->CreatePointerCast(b->getRawOutputPointer("literalByteBuffer", SIZE_0), i8PtrTy);
        Value* remainingBufferSize = b->CreateSub(outputCapacity, literalPartBeginPosRem);
        Value* literalCopyLength1 = b->CreateUMin(remainingBufferSize, totalLiteralLength);
        Value* literalCopyLength2 = b->CreateSub(totalLiteralLength, literalCopyLength1);

        b->CreateMemCpy(
                b->CreateGEP(outputBasePtr, literalPartBeginPosRem),
                literalOutputBasePtr,
                literalCopyLength1,
                1
        );

        b->CreateMemCpy(
                outputBasePtr,
                b->CreateGEP(literalOutputBasePtr, literalCopyLength1),
                literalCopyLength2,
                1
        );


        // copy Token Part
        // copy token buffer to [initOutputCursor + literalLength, initOutputCursor + literalLength + tokenLength)
        Value* tokenPartBeginPos = b->CreateAdd(literalPartBeginPos, totalLiteralLength);
        Value* tokenPartBeginPosRem = b->CreateURem(tokenPartBeginPos, outputCapacity);
        Value* tokenOutputBasePtr = b->CreatePointerCast(b->getRawOutputPointer("tokenByteBuffer", SIZE_0), i8PtrTy);
        remainingBufferSize = b->CreateSub(outputCapacity, tokenPartBeginPosRem);
        Value* tokenCopyLength1 = b->CreateUMin(remainingBufferSize, totalTokenLength);
        Value* tokenCopyLength2 = b->CreateSub(totalTokenLength, tokenCopyLength1);

        b->CreateMemCpy(
                b->CreateGEP(outputBasePtr, tokenPartBeginPosRem),
                tokenOutputBasePtr,
                tokenCopyLength1,
                1
        );

        b->CreateMemCpy(
                outputBasePtr,
                b->CreateGEP(tokenOutputBasePtr, tokenCopyLength1),
                tokenCopyLength2,
                1
        );
        b->setProducedItemCount("outputStream", b->CreateAdd(blockSizePos, b->CreateAdd(blockSize, b->getSize(4))));
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
            Value* strToBlockIndexBasePtr = b->CreatePointerCast(b->getRawOutputPointer("strToBlockIndex", b->getSize(0)), i64PtrTy);
            Value* strToBlockIndexTargetPtr = b->CreateGEP(strToBlockIndexBasePtr, hashedKey);
            Value* newBlockIndexCacheValue = b->CreateOr(b->CreateShl(b->CreateLoad(strToBlockIndexTargetPtr), INT_64_16), localI64BlockIndex);
            b->CreateStore(newBlockIndexCacheValue, strToBlockIndexTargetPtr);

            // Update mStrToMatchPos
            Value* strToMatchPosBasePtr = b->CreatePointerCast(b->getRawOutputPointer("strToMatchPos", b->getSize(0)), i32PtrTy);
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


    MatchInfo
    LZParabixCompressionKernel::extractMatchInfo(const std::unique_ptr<KernelBuilder> &b, llvm::Value *cursorPos,
                                                 llvm::Value *initBlockGlobalIndex, llvm::Value *inputEndPos,
                                                 llvm::Value *previousCursorPos) {
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

            BasicBlock* scanForwardMatchConBlock = b->CreateBasicBlock("scanForwardMatchConBlock");
            BasicBlock* scanForwardMatchBodyBlock = b->CreateBasicBlock("scanForwardMatchBodyBlock");
            BasicBlock* scanForwardMatchExitBlock = b->CreateBasicBlock("scanForwardMatchExitBlock");

//            b->CreateCondBr(isNoMatch, scanForwardMatchExitBlock, scanForwardMatchConBlock);
            b->CreateBr(scanForwardMatchConBlock);

            // ---- scanForwardMatchConBlock
            b->SetInsertPoint(scanForwardMatchConBlock);

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
                    scanForwardMatchBodyBlock,
                    scanForwardMatchExitBlock
            );

            // ---- scanForwardMatchBodyBlock
            b->SetInsertPoint(scanForwardMatchBodyBlock);

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

            b->CreateBr(scanForwardMatchConBlock);

            // ---- scanForwardMatchExitBlock
            b->SetInsertPoint(scanForwardMatchExitBlock);


            BasicBlock* scanBackwardMatchConBlock = b->CreateBasicBlock("scanBackwardMatchConBlock");
            BasicBlock* scanBackwardMatchBodyBlock = b->CreateBasicBlock("scanBackwardMatchBodyBlock");
            BasicBlock* scanBackwardMatchExitBlock = b->CreateBasicBlock("scanBackwardMatchExitBlock");

            b->CreateBr(scanBackwardMatchConBlock);

            // ---- scanBackwardMatchConBlock
            b->SetInsertPoint(scanBackwardMatchConBlock);
            PHINode* phiBackMatchLength = b->CreatePHI(b->getSizeTy(), 2);
            phiBackMatchLength->addIncoming(b->getSize(0), scanForwardMatchExitBlock);
            PHINode* phiBackMatchMask = b->CreatePHI(b->getInt64Ty(), 2);
            phiBackMatchMask->addIncoming(b->getInt64(0), scanForwardMatchExitBlock);
            PHINode* phiBackMatchPos = b->CreatePHI(b->getSizeTy(), 2);
            phiBackMatchPos->addIncoming(matchPos, scanForwardMatchExitBlock);


            Value* backMatchMinPos = b->CreateUMax(previousCursorPos, b->CreateMul(currentGlobalInputBlockIndex, b->getSize(64)));
            b->CreateCondBr(
                    b->CreateAnd(
                            b->CreateAnd(
                                    b->CreateICmpUGT(phiBackMatchPos, b->getSize(0)),
                                    b->CreateICmpUGT(b->CreateSub(cursorPos, phiBackMatchLength), backMatchMinPos)
                            ),
                            b->CreateNot(isNoMatch)
                    ),
                    scanBackwardMatchBodyBlock,
                    scanBackwardMatchExitBlock
            );

            // ---- scanBackwardMatchBodyBlock
            b->SetInsertPoint(scanBackwardMatchBodyBlock);
            Value* targetBackMatchPos = b->CreateSub(phiBackMatchPos, b->getSize(1));

            Value* isBackMatch = b->CreateICmpEQ(
                    b->CreateLoad(b->CreateGEP(inputBasePtr, b->CreateAdd(b->CreateMul(globalMatchBlockIndex, b->getInt64(64)), targetBackMatchPos))),
                    b->CreateLoad(b->CreateGEP(inputBasePtr, b->CreateSub(cursorPos, b->CreateAdd(phiBackMatchLength, b->getSize(1)))))
            );

            phiBackMatchLength->addIncoming(
                    b->CreateSelect(isBackMatch, b->CreateAdd(phiBackMatchLength, b->getSize(1)), phiBackMatchLength),
                    b->GetInsertBlock()
            );

            phiBackMatchPos->addIncoming(
                    b->CreateSub(phiBackMatchPos, b->getSize(1)),
                    b->GetInsertBlock()
            );

            phiBackMatchMask->addIncoming(
                    b->CreateSelect(
                            isBackMatch,
                            b->CreateOr(phiBackMatchMask, b->CreateShl(b->getInt64(1), targetBackMatchPos)),
                            phiBackMatchMask
                    ),
                    b->GetInsertBlock()
            );
            b->CreateBr(scanBackwardMatchConBlock);

            // ---- scanBackwardMatchExitBlock
            b->SetInsertPoint(scanBackwardMatchExitBlock);

            Value* newMatchLength = b->CreateAdd(phiForwardMatchLength, phiBackMatchLength);

            Value* isBetterMatch = b->CreateICmpUGT(newMatchLength, retInfo.matchLength);
            retInfo.matchLength = b->CreateSelect(isBetterMatch, newMatchLength, retInfo.matchLength);
            retInfo.matchMask = b->CreateSelect(isBetterMatch, b->CreateOr(phiMatchMask, phiBackMatchMask), retInfo.matchMask);
            retInfo.matchOffset = b->CreateSelect(isBetterMatch, blockOffset, retInfo.matchOffset);
            retInfo.matchStart = b->CreateSelect(isBetterMatch, b->CreateSub(cursorPos, phiBackMatchLength), retInfo.matchStart);
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
        Value* strToBlockIndexBasePtr = b->CreatePointerCast(b->getRawOutputPointer("strToBlockIndex", b->getSize(0)), i64PtrTy);
        Value* strToBlockIndexTargetPtr = b->CreateGEP(strToBlockIndexBasePtr, hashedKey);

        Value* strToMatchPosBasePtr = b->CreatePointerCast(b->getRawOutputPointer("strToMatchPos", b->getSize(0)), i32PtrTy);
        Value* strToMatchPosTargetPtr = b->CreateGEP(strToMatchPosBasePtr, hashedKey);

        return std::make_pair(
                b->CreateLoad(strToBlockIndexTargetPtr),
                b->CreateLoad(strToMatchPosTargetPtr)
        );
    }

    llvm::Value* LZParabixCompressionKernel::appendLiteralSequence(const unique_ptr<KernelBuilder> &b, Value *literalStart,
                                                             Value *literalEnd, llvm::Value *tokenOutputPos) {
        // Constant
        Value* SIZE_0 = b->getSize(0);
        Value* SIZE_1 = b->getSize(1);
        Value* SIZE_64 = b->getSize(64);
        Type* i8PtrTy = b->getInt8PtrTy();

        Value* inputBasePtr = b->CreatePointerCast(b->getRawInputPointer("byteStream", SIZE_0), i8PtrTy);
        Value* tokenOutputBasePtr = b->CreatePointerCast(b->getRawOutputPointer("tokenByteBuffer", SIZE_0), i8PtrTy);
        Value* outputCapacity = b->getCapacity("outputStream");

        // ---- EntryBlock
        BasicBlock* entryBlock = b->GetInsertBlock();
        BasicBlock* appendLiteralConBlock = b->CreateBasicBlock("AppendLiteralConBlock");
        BasicBlock* appendLiteralBodyBlock = b->CreateBasicBlock("AppendLiteralBodyBlock");
        BasicBlock* appendLiteralExitBlock = b->CreateBasicBlock("AppendLiteralExitBlock");

        b->CreateBr(appendLiteralConBlock);

        // ---- AppendLiteralConBlock
        b->SetInsertPoint(appendLiteralConBlock);
        PHINode* phiTokenOutputPos = b->CreatePHI(b->getSizeTy(), 2);
        phiTokenOutputPos->addIncoming(tokenOutputPos, entryBlock);
        PHINode* phiLiteralStart = b->CreatePHI(b->getSizeTy(), 2);
        phiLiteralStart->addIncoming(literalStart, entryBlock);

        Value* remainingLiteralLength = b->CreateSub(literalEnd, phiLiteralStart);
        b->CreateCondBr(b->CreateICmpUGT(remainingLiteralLength, b->getSize(0)), appendLiteralBodyBlock, appendLiteralExitBlock);

        // ---- AppendLiteralBodyBlock
        b->SetInsertPoint(appendLiteralBodyBlock);

        Value* maxLiteralLength = b->getSize(127);
        Value* literalLength = b->CreateUMin(remainingLiteralLength, maxLiteralLength);

        Value* literalToken = b->CreateOr(b->CreateTrunc(literalLength, b->getInt8Ty()), b->getInt8(1 << 7));

        b->CreateStore(literalToken, b->CreateGEP(tokenOutputBasePtr, phiTokenOutputPos));

        this->appendLiteralData(b, phiLiteralStart, literalLength);

        phiLiteralStart->addIncoming(b->CreateAdd(phiLiteralStart, literalLength), b->GetInsertBlock());
        phiTokenOutputPos->addIncoming(b->CreateAdd(phiTokenOutputPos, SIZE_1), b->GetInsertBlock());
        b->CreateBr(appendLiteralConBlock);

        // ---- AppendLiteralExitBlock
        b->SetInsertPoint(appendLiteralExitBlock);
        return phiTokenOutputPos;
    }

    Value* LZParabixCompressionKernel::appendMatchSequence(const unique_ptr<KernelBuilder> &b, const MatchInfo& matchInfo, Value* tokenOutputPos) {
        // Constant
        Value* SIZE_0 = b->getSize(0);
        Value* SIZE_1 = b->getSize(1);
        Value* SIZE_64 = b->getSize(64);
        Type* i8PtrTy = b->getInt8PtrTy();

        Value* tokenOutputBasePtr = b->CreatePointerCast(b->getRawOutputPointer("tokenByteBuffer", SIZE_0), i8PtrTy);

        // MatchOffset
        b->CreateStore(b->CreateTrunc(matchInfo.matchOffset, b->getInt8Ty()), b->CreateGEP(tokenOutputBasePtr, tokenOutputPos));

        Value* outputOneBitIndexPos = b->CreateAdd(tokenOutputPos, SIZE_1);
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
            b->CreateStore(currentBitIndex, b->CreateGEP(tokenOutputBasePtr, outputBitMaskPos));
            outputBitMaskPos = b->CreateSelect(
                    b->CreateOr(newOneBit, newZeroBit),
                    outputBitMaskPos,
                    b->CreateAdd(outputBitMaskPos, SIZE_1)
            );
        }
        b->CreateStore(oneBitIndex, b->CreateGEP(tokenOutputBasePtr, outputOneBitIndexPos));
        b->CreateStore(zeroBitIndex, b->CreateGEP(tokenOutputBasePtr,outputZeroBitIndexPos));

        return outputBitMaskPos;
    }


    void LZParabixCompressionKernel::appendLiteralData(const std::unique_ptr<KernelBuilder> &b, llvm::Value* literalStart, llvm::Value* literalLength) {
        Value* SIZE_0 = b->getSize(0);
        Value* SIZE_64 = b->getSize(64);
        Value* shouldPrint = b->CreateICmpEQ(b->getScalarField("literalOutputPos"), b->getSize(0));

        Value* literalStartRem = b->CreateURem(literalStart, b->getCapacity("basisBits"));
        Value* literalEndRem = b->CreateAdd(literalStartRem, literalLength);


        // ---- EntryBlock
        BasicBlock* entryBlock = b->GetInsertBlock();
        BasicBlock* literalCopyConBlock = b->CreateBasicBlock("literalCopyConBlock");
        BasicBlock* literalCopyBodyBlock = b->CreateBasicBlock("literalCopyBodyBlock");
        BasicBlock* literalCopyExitBlock = b->CreateBasicBlock("literalCopyExitBlock");
        b->CreateBr(literalCopyConBlock);

        // ---- literalCopyConBlock
        b->SetInsertPoint(literalCopyConBlock);
        PHINode* phiLiteralPos = b->CreatePHI(b->getSizeTy(), 2);
        phiLiteralPos->addIncoming(literalStartRem, entryBlock);
        b->CreateCondBr(b->CreateICmpULT(phiLiteralPos, literalEndRem), literalCopyBodyBlock, literalCopyExitBlock);

        // ---- literalCopyBodyBlock
        b->SetInsertPoint(literalCopyBodyBlock);


        Value* remainingLiteralLength = b->CreateSub(literalEndRem, phiLiteralPos);
        Value* copyLength = b->CreateSub(SIZE_64, b->CreateURem(phiLiteralPos, SIZE_64));
        copyLength = b->CreateUMin(copyLength, remainingLiteralLength);

        Value* cursorBlockIndex = b->CreateUDiv(phiLiteralPos, b->getSize(b->getBitBlockWidth()));
        Value* cursorBlockRem = b->CreateURem(phiLiteralPos, b->getSize(b->getBitBlockWidth()));
        Value* cursorI64BlockIndex = b->CreateUDiv(cursorBlockRem, b->getSize(64));
        Value* cursorI64BlockRem = b->CreateURem(cursorBlockRem, b->getSize(64));
        Value* literalMask = b->CreateSub(
                b->CreateSelect(b->CreateICmpEQ(copyLength, b->getInt64(0x40)), b->getInt64(0), b->CreateShl(b->getInt64(1), copyLength)),
                b->getInt64(1)
        );

        std::vector<llvm::Value*> extractValues;

        Value* bitStreamBasePtr = b->CreatePointerCast(b->getRawInputPointer("basisBits", SIZE_0), b->getBitBlockType()->getPointerTo());
        Value* targetBlockBasePtr = b->CreatePointerCast(b->CreateGEP(bitStreamBasePtr, b->CreateMul(cursorBlockIndex, b->getSize(8))), b->getInt64Ty()->getPointerTo());

        for (unsigned j = 0; j < 8; j++) {
            Value* ptr = b->CreateGEP(targetBlockBasePtr, b->CreateAdd(cursorI64BlockIndex, b->getSize(j * (b->getBitBlockWidth() / 64))));
            Value* extractV = b->CreateLShr(b->CreateLoad(ptr), cursorI64BlockRem);
            extractV = b->CreateAnd(extractV, literalMask);
            extractValues.push_back(extractV);
        }
        this->appendLiteralData(b, extractValues, copyLength);

        phiLiteralPos->addIncoming(b->CreateAdd(phiLiteralPos, copyLength), b->GetInsertBlock());
        b->CreateBr(literalCopyConBlock);

        // ---- literalCopyExitBlock
        b->SetInsertPoint(literalCopyExitBlock);
    }

    void LZParabixCompressionKernel::appendLiteralData(const std::unique_ptr<KernelBuilder> &b, std::vector<llvm::Value*> literalData, llvm::Value* valueLength) {
        BasicBlock* exitBlock = b->CreateBasicBlock("exitBlock");

        Value* oldOutputPos = b->getScalarField("literalOutputPos");
        Value* oldOutputPosRem64 = b->CreateURem(oldOutputPos, b->getSize(64));
        Value* pendingOutputBasePtr = b->CreatePointerCast(b->getScalarFieldPtr("pendingLiteralOutput"), b->getInt64Ty()->getPointerTo());


        std::vector<llvm::Value*> newOutputVec;

        for (unsigned j = 0; j < 8; j++) {
            Value* newValue = b->CreateOr(b->CreateLoad(b->CreateGEP(pendingOutputBasePtr, b->getSize(j))), b->CreateShl(literalData[j], oldOutputPosRem64));
            newOutputVec.push_back(newValue);
        }

        BasicBlock* noStoreOutputBlock = b->CreateBasicBlock("noStoreOutputBlock");
        BasicBlock* storeOutputBlock =b->CreateBasicBlock("storeOutputBlock");

        b->CreateCondBr(b->CreateICmpULT(b->CreateAdd(oldOutputPosRem64, valueLength), b->getSize(64)), noStoreOutputBlock, storeOutputBlock);

        // ---- noStoreOutputBlock
        b->SetInsertPoint(noStoreOutputBlock);

        for (unsigned j = 0; j < 8; j++) {
            b->CreateStore(newOutputVec[j], b->CreateGEP(pendingOutputBasePtr, b->getSize(j)));
        }

        b->CreateBr(exitBlock);

        // ---- storeOutputBlock
        b->SetInsertPoint(storeOutputBlock);

        Value* oldOutputPosRem = oldOutputPos; // we guarantee oldOutputPos will always inside buffer
        Value* oldOutputPosBitBlockIndex = b->CreateUDiv(oldOutputPosRem, b->getSize(b->getBitBlockWidth()));
        Value* oldOutputPosBitBlockRem = b->CreateURem(oldOutputPosRem, b->getSize(b->getBitBlockWidth()));

        Value* outputBasePtr = b->CreatePointerCast(b->getRawOutputPointer("literalByteBuffer", b->getSize(0)), b->getBitBlockType()->getPointerTo());
        Value* outputBitBlockBasePtr = b->CreateGEP(outputBasePtr, b->CreateMul(oldOutputPosBitBlockIndex, b->getSize(8)));
        outputBitBlockBasePtr = b->CreatePointerCast(outputBitBlockBasePtr, b->getInt64Ty()->getPointerTo());

        Value* oldOutputPosI64Index = b->CreateUDiv(oldOutputPosBitBlockRem, b->getSize(64));

        for (unsigned j = 0; j < 8; j++) {
            Value* targetPtr = b->CreateGEP(outputBitBlockBasePtr, b->CreateAdd(oldOutputPosI64Index, b->getSize(j * (b->getBitBlockWidth() / 64))));
            b->CreateStore(newOutputVec[j], targetPtr);
        }

        Value* shiftAmount = b->CreateSub(b->getSize(0x40), oldOutputPosRem64);
        Value* fullyShift = b->CreateICmpEQ(shiftAmount, b->getSize(0x40));

        for (unsigned j = 0; j < 8; j++) {
            b->CreateStore(b->CreateSelect(fullyShift, b->getInt64(0), b->CreateLShr(literalData[j], shiftAmount)), b->CreateGEP(pendingOutputBasePtr, b->getSize(j)));
        }

        b->CreateBr(exitBlock);

        b->SetInsertPoint(exitBlock);
        b->setScalarField("literalOutputPos", b->CreateAdd(oldOutputPos, valueLength));
    }

    void LZParabixCompressionKernel::storePendingLiteralData(const std::unique_ptr<KernelBuilder> &b) {
        Value* oldOutputPos = b->getScalarField("literalOutputPos");
        Value* oldOutputPosRem = oldOutputPos; // We guarantee output buffer will always be enough
        Value* oldOutputPosBitBlockIndex = b->CreateUDiv(oldOutputPosRem, b->getSize(b->getBitBlockWidth()));
        Value* oldOutputPosBitBlockRem = b->CreateURem(oldOutputPosRem, b->getSize(b->getBitBlockWidth()));
        Value* oldOutputPosI64Index = b->CreateUDiv(oldOutputPosBitBlockRem, b->getSize(64));

        Value* pendingOutputBasePtr = b->CreatePointerCast(b->getScalarFieldPtr("pendingLiteralOutput"), b->getInt64Ty()->getPointerTo());
        Value* outputBasePtr = b->CreatePointerCast(b->getRawOutputPointer("literalByteBuffer", b->getSize(0)), b->getBitBlockType()->getPointerTo());
        Value* outputBitBlockBasePtr = b->CreateGEP(outputBasePtr, b->CreateMul(oldOutputPosBitBlockIndex, b->getSize(8)));
        outputBitBlockBasePtr = b->CreatePointerCast(outputBitBlockBasePtr, b->getInt64Ty()->getPointerTo());
        for (unsigned j = 0; j < 8; j++) {
            Value* targetPtr = b->CreateGEP(outputBitBlockBasePtr, b->CreateAdd(oldOutputPosI64Index, b->getSize(j * (b->getBitBlockWidth() / 64))));
            b->CreateStore(b->CreateLoad(b->CreateGEP(pendingOutputBasePtr, b->getSize(j))), targetPtr);
        }
    }
}