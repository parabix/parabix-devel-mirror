
#include "lz4_bitstream_decompression.h"

#include <kernels/kernel_builder.h>
#include <iostream>
#include <string>
#include <llvm/Support/raw_ostream.h>
#include <kernels/core/streamset.h>

using namespace llvm;
using namespace kernel;
using namespace std;

namespace kernel{

    std::vector<unsigned> makeNumOfBitstreams(const StreamSets & streams) {
        std::vector<unsigned> numOfBitstreams;
        numOfBitstreams.reserve(streams.size());
        for (const auto & stream : streams) {
            numOfBitstreams.push_back(stream->getNumElements());
        }
        return numOfBitstreams;
    }

    LZ4BitStreamDecompressionKernel::LZ4BitStreamDecompressionKernel(const std::unique_ptr<kernel::KernelBuilder> &b,
                                                                     Scalar * fileSize,
                                                                     StreamSet * inputStream,
                                                                     const LZ4BlockInfo & blockInfo,
                                                                     StreamSets inputStreams,
                                                                     StreamSets outputStreams)
    : LZ4SequentialDecompressionKernel(b, "LZ4BitStreamDecompression", fileSize, inputStream, blockInfo),
      mNumsOfBitStreams(makeNumOfBitstreams(inputStreams)) {
        mInputStreamSets.push_back(Binding{"inputBitStream0", inputStreams[0], RateEqualTo("byteStream")});
        for (unsigned i = 1; i < inputStreams.size(); i++) {
            mInputStreamSets.push_back(Binding{"inputBitStream" + std::to_string(i), inputStreams[i], RateEqualTo("byteStream")});
        }
        mOutputStreamSets.push_back(Binding{"outputStream0", outputStreams[0], BoundedRate(0, 1)});
        for (unsigned i = 1; i < outputStreams.size(); i++) {
            mOutputStreamSets.push_back(Binding{"outputStream" + std::to_string(i), outputStreams[i], RateEqualTo("outputStream0")});
        }
        initPendingOutputScalar(b);
    }

    void LZ4BitStreamDecompressionKernel::initPendingOutputScalar(const std::unique_ptr<kernel::KernelBuilder> &b) {
        for (unsigned i = 0; i < mNumsOfBitStreams.size(); i++) {
            for (unsigned j = 0; j < mNumsOfBitStreams[i]; j++) {
                addInternalScalar(b->getInt64Ty(), "pendingOutput" + std::to_string(i) + "_" + std::to_string(j));
            }
        }
    }

    void LZ4BitStreamDecompressionKernel::doLiteralCopy(const std::unique_ptr<KernelBuilder> &b, Value *literalStart,
                                              Value *literalLength, Value* blockStart) {
        // Constant
        ConstantInt* INT_64_0 = b->getInt64(0);
        ConstantInt* INT_64_1 = b->getInt64(1);

        // ---- EntryBlock
        BasicBlock* entryBlock = b->GetInsertBlock();
        Value* remLiteralStart = b->CreateURem(literalStart, b->getCapacity("inputBitStream0"));
        Value* remLiteralEnd = b->CreateAdd(remLiteralStart, literalLength);


        BasicBlock* literalCopyCon = b->CreateBasicBlock("literalCopyCon");
        BasicBlock* literalCopyBody = b->CreateBasicBlock("literalCopyBody");
        BasicBlock* literalCopyExit = b->CreateBasicBlock("literalCopyExit");

        b->CreateBr(literalCopyCon);

        // ---- literalCopyCon
        b->SetInsertPoint(literalCopyCon);
        PHINode* phiLiteralPos = b->CreatePHI(b->getSizeTy(), 2);
        phiLiteralPos->addIncoming(remLiteralStart, entryBlock);
        b->CreateCondBr(b->CreateICmpULT(phiLiteralPos, remLiteralEnd), literalCopyBody, literalCopyExit);

        // ---- literalCopyBody
        b->SetInsertPoint(literalCopyBody);

        Value* cursorBlockIndex = b->CreateUDiv(phiLiteralPos, b->getSize(b->getBitBlockWidth()));
        Value* cursorBlockRem = b->CreateURem(phiLiteralPos, b->getSize(b->getBitBlockWidth()));
        Value* cursorI64BlockIndex = b->CreateUDiv(cursorBlockRem, b->getSize(64));
        Value* cursorI64BlockRem = b->CreateURem(cursorBlockRem, b->getSize(64));

        Value* remainingLiteralLength = b->CreateSub(remLiteralEnd, phiLiteralPos);
        Value* copyLength = b->CreateSub(b->getSize(64), cursorI64BlockRem);
        copyLength = b->CreateUMin(copyLength, remainingLiteralLength);
        Value* copyMask = b->CreateSub(
                b->CreateSelect(b->CreateICmpEQ(copyLength, b->getInt64(0x40)), INT_64_0, b->CreateShl(INT_64_1, copyLength)),
                INT_64_1
        );

        std::vector<Value*> extractValues;

        for (unsigned i = 0; i < mNumsOfBitStreams.size(); i++) {
            Value* bitStreamBasePtr = b->CreatePointerCast(b->getRawInputPointer("inputBitStream" + std::to_string(i), b->getSize(0)), b->getBitBlockType()->getPointerTo());
            Value* targetBlockBasePtr = b->CreatePointerCast(b->CreateGEP(bitStreamBasePtr, b->CreateMul(cursorBlockIndex, b->getSize(mNumsOfBitStreams[i]))), b->getInt64Ty()->getPointerTo());

            for (unsigned j = 0; j < mNumsOfBitStreams[i]; j++) {
                Value* ptr = b->CreateGEP(targetBlockBasePtr, b->CreateAdd(cursorI64BlockIndex, b->getSize(j * (b->getBitBlockWidth() / 64))));
                Value* extractV = b->CreateLShr(b->CreateLoad(ptr), cursorI64BlockRem);
                extractV = b->CreateAnd(extractV, copyMask);
                extractValues.push_back(extractV);
            }
        }
        appendBitStreamOutput(b, extractValues, copyLength);

        phiLiteralPos->addIncoming(b->CreateAdd(phiLiteralPos, copyLength), b->GetInsertBlock());
        b->CreateBr(literalCopyCon);

        // ---- literalCopyExit
        b->SetInsertPoint(literalCopyExit);
    }

    void LZ4BitStreamDecompressionKernel::doMatchCopy(const std::unique_ptr<KernelBuilder> &b, Value *matchOffset,
                                            Value *matchLength) {
        // Constant
        ConstantInt* INT_64_0 = b->getInt64(0);
        ConstantInt* INT_64_1 = b->getInt64(1);

        // ---- EntryBlock
        BasicBlock* entryBlock = b->GetInsertBlock();
        Value* outputPos = b->getScalarField("outputPos");
        Value* outputBufferSize = b->getCapacity("outputStream0");
        Value* matchPos = b->CreateSub(outputPos, matchOffset);

        Value* remMatchPos = b->CreateURem(matchPos, outputBufferSize);
        Value* remMatchEndPos = b->CreateAdd(remMatchPos, matchLength);

        BasicBlock* matchCopyCon = b->CreateBasicBlock("matchCopyCon");
        BasicBlock* matchCopyBody = b->CreateBasicBlock("matchCopyBody");
        BasicBlock* matchCopyExit = b->CreateBasicBlock("matchCopyExit");

        b->CreateBr(matchCopyCon);

        // ---- matchCopyCon
        b->SetInsertPoint(matchCopyCon);
        PHINode* phiMatchPos = b->CreatePHI(b->getSizeTy(), 2);
        phiMatchPos->addIncoming(remMatchPos, entryBlock);
        b->CreateCondBr(b->CreateICmpULT(phiMatchPos, remMatchEndPos), matchCopyBody, matchCopyExit);

        // ---- matchCopyBody
        b->SetInsertPoint(matchCopyBody);

        Value* matchPosBlockIndex = b->CreateUDiv(phiMatchPos, b->getSize(b->getBitBlockWidth()));
        Value* matchPosBlockRem = b->CreateURem(phiMatchPos, b->getSize(b->getBitBlockWidth()));
        Value* matchPosI64BlockIndex = b->CreateUDiv(matchPosBlockRem, b->getSize(64));
        Value* matchPosI64BlockRem = b->CreateURem(matchPosBlockRem, b->getSize(64));

        Value* remainingMatchLength = b->CreateSub(remMatchEndPos, phiMatchPos);
        Value* copyLength = b->CreateSub(b->getSize(64), matchPosI64BlockRem);
        copyLength = b->CreateUMin(copyLength, remainingMatchLength);
        copyLength = b->CreateUMin(copyLength, matchOffset);

        Value* copyMask = b->CreateSub(
                b->CreateSelect(b->CreateICmpEQ(copyLength, b->getInt64(0x40)), INT_64_0, b->CreateShl(INT_64_1, copyLength)),
                INT_64_1
        );


        Value* sameBlock = b->CreateICmpEQ(
                b->CreateUDiv(b->CreateAdd(phiMatchPos, matchOffset), b->getSize(64)),
                b->CreateUDiv(phiMatchPos, b->getSize(64))
        );
//        b->CallPrintInt("sameBlock", sameBlock);

        std::vector<Value*> extractValues;
//        b->CallPrintInt("matchPosI64BlockRem", matchPosI64BlockRem);
        unsigned iStreamIndex = 0;
        for (unsigned i = 0; i < mNumsOfBitStreams.size(); i++) {
            Value* bitStreamBasePtr = b->CreatePointerCast(b->getRawOutputPointer("outputStream" + std::to_string(i), b->getSize(0)), b->getBitBlockType()->getPointerTo());
            Value* targetBlockBasePtr = b->CreatePointerCast(b->CreateGEP(bitStreamBasePtr, b->CreateMul(matchPosBlockIndex, b->getSize(mNumsOfBitStreams[i]))), b->getInt64Ty()->getPointerTo());

            for (unsigned j = 0; j < mNumsOfBitStreams[i]; j++) {
                Value* ptr = b->CreateGEP(targetBlockBasePtr, b->CreateAdd(matchPosI64BlockIndex, b->getSize(j * (b->getBitBlockWidth() / 64))));
                Value* extractV = b->CreateLoad(ptr);

                extractV = b->CreateSelect(sameBlock, b->getScalarField("pendingOutput" + std::to_string(i) + "_" + std::to_string(j)), extractV);
                extractV = b->CreateLShr(extractV, matchPosI64BlockRem);
                extractV = b->CreateAnd(extractV, copyMask);

                extractValues.push_back(extractV);

                ++iStreamIndex;
            }
            for (unsigned j = 0; j < mNumsOfBitStreams[i]; j++) {
//                b->CallPrintRegister("e" + std::to_string(j), b->CreateZExt(extractValues[mNumsOfBitStreams[i] - j], b->getIntNTy(256)));
            }

        }

        appendBitStreamOutput(b, extractValues, copyLength);

        phiMatchPos->addIncoming(b->CreateAdd(phiMatchPos, copyLength), b->GetInsertBlock());
        b->CreateBr(matchCopyCon);

        // ---- matchCopyExit
        b->SetInsertPoint(matchCopyExit);
    }

    void LZ4BitStreamDecompressionKernel::setProducedOutputItemCount(const std::unique_ptr<KernelBuilder> &b, Value *produced) {
        b->setProducedItemCount("outputStream0", produced);
    }

    void LZ4BitStreamDecompressionKernel::appendBitStreamOutput(const std::unique_ptr<KernelBuilder> &b, std::vector<Value*>& extractedValues, Value* valueLength) {
        BasicBlock* exitBlock = b->CreateBasicBlock("exitBlock");

        Value* oldOutputPos = b->getScalarField("outputPos");
        Value* oldOutputPosRem64 = b->CreateURem(oldOutputPos, b->getSize(64));

        std::vector<Value*> newOutputVec;

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

    void LZ4BitStreamDecompressionKernel::storePendingOutput(const std::unique_ptr<KernelBuilder> &b) {
        BasicBlock* storePendingOutputBlock = b->CreateBasicBlock("storePendingOutputBlock");
        BasicBlock* storePendingOutputExitBlock = b->CreateBasicBlock("storePendingOutputExitBlock");

        Value* oldOutputPos = b->getScalarField("outputPos");
        b->CreateCondBr(
                b->CreateICmpNE(b->CreateURem(oldOutputPos, b->getSize(64)), b->getSize(0)),
                storePendingOutputBlock,
                storePendingOutputExitBlock
        );

        b->SetInsertPoint(storePendingOutputBlock);
        storePendingOutput_BitStream(b);
//        storePendingOutput_Swizzled(b);
        b->CreateBr(storePendingOutputExitBlock);

        b->SetInsertPoint(storePendingOutputExitBlock);
    }

    void LZ4BitStreamDecompressionKernel::storePendingOutput_BitStream(const std::unique_ptr<KernelBuilder> &b) {

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

}
