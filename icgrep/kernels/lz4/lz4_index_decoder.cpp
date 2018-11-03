/*
 *  Copyright (c) 2017 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#include "lz4_index_decoder.h"
#include <kernels/kernel_builder.h>
 
using namespace llvm;
using namespace kernel;

#ifndef NDEBUG
#define DEBUG_RT_PRINT 0
#else
#define DEBUG_RT_PRINT 0
#endif

#define printRTDebugMsg(MSG)
//    if (DEBUG_RT_PRINT) b->CallPrintMsgToStderr(MSG)

#define printRTDebugInt(NAME, X)
//    if (DEBUG_RT_PRINT) b->CallPrintIntToStderr(NAME, X)

#define printGlobalPos()
//    printRTDebugInt("GlobalPos", b->CreateAdd(blockStartPos, b->CreateLoad(sOffset)))

namespace {

Value * generateBitswap(const std::unique_ptr<KernelBuilder> & b, Value * v) {
    Value * bswapFunc = Intrinsic::getDeclaration(b->getModule(),
            Intrinsic::bswap, v->getType());
    return b->CreateCall(bswapFunc, {v});
}

Value * createStackVar(const std::unique_ptr<KernelBuilder> & b, Type * type, StringRef name, Value * initializer = nullptr) {
    Value * var = b->CreateAlloca(type, nullptr, name);
    if (initializer) {
        b->CreateStore(initializer, var);
    } else {
        b->CreateStore(ConstantInt::get(type, 0), var);
    }
    return var;
}

void incStackVar(const std::unique_ptr<KernelBuilder> & b, Value * svar, Value * increment = nullptr) {
    Value * value = b->CreateLoad(svar);
    if (increment) {
        value = b->CreateAdd(value, increment);
    } else {
        value = b->CreateAdd(value, ConstantInt::get(value->getType(), 1));
    }
    b->CreateStore(value, svar);
}

Value * getOutputPtr(const std::unique_ptr<KernelBuilder> & b, Value * blockStartPtr, Value * offset) {
    return b->CreateGEP(
            b->CreatePointerCast(blockStartPtr, b->getInt32Ty()->getPointerTo()),
            offset
            );
}

}       // anonymouse namespace

/**
 * Get the offset within the current word.
 */
Value * LZ4IndexDecoderKernel::getWordOffset(const std::unique_ptr<kernel::KernelBuilder> & b) {
    Value * offset = b->CreateLoad(sOffset);
    IntegerType * type = cast<IntegerType>(offset->getType());
    Constant * mask = ConstantInt::get(type, wordWidth - 1);
    return b->CreateAnd(offset, mask);
}

/**
 * Get the offset of the start of the current word.
 */
Value * LZ4IndexDecoderKernel::getWordStartOffset(const std::unique_ptr<KernelBuilder> & b) {
    Value * offset = b->CreateLoad(sOffset);
    IntegerType * type = cast<IntegerType>(offset->getType());
    Constant * mask = ConstantExpr::getNeg(ConstantInt::get(type, wordWidth));
    return b->CreateAnd(offset, mask);
}

/**
 * Load a raw byte from byteStream.
 * If offset is not provided, load the current byte by default.
 */
Value * LZ4IndexDecoderKernel::loadRawByte(const std::unique_ptr<KernelBuilder> & b, Value * offset) {
    Value * blockStartPtr = b->CreatePointerCast(
            b->getInputStreamBlockPtr("byteStream", b->getInt32(0)),
            b->getInt8PtrTy()
            );
    if (offset == nullptr) {
        offset = b->CreateLoad(sOffset);
    }
    Value * ptr = b->CreateGEP(blockStartPtr, offset);
    return b->CreateLoad(ptr);
}


/**
 * Set the current extender word up until before the offset position.
 * extender = .......  (little-endian, LSB on the right)
 * offset   =    ^
 * cleared  = ....111
 */
void LZ4IndexDecoderKernel::setExtenderUntilOffset(const std::unique_ptr<KernelBuilder> & b) {
    // Little-endian, offset counts from LSB
    // extender = extender ^ ~((1 << offset) -1)
    Value * extender = b->CreateLoad(sExtender);
    Value * wordOffset = b->CreateZExt(
            getWordOffset(b),
            b->getSizeTy()
            );
    Value * one = b->getSize(1);
    Value * mask = b->CreateSub(
            b->CreateShl(one, wordOffset),
            one);
    extender = b->CreateOr(extender, mask);
    b->CreateStore(extender, sExtender);
}


/**
 * Load the extender word at the current offset.
 * Called when we potentially reach a new word.  Usually followed by setExtenderUntilOffset.
 */
void LZ4IndexDecoderKernel::loadCurrentExtender(const std::unique_ptr<KernelBuilder> & b) {
    Value * offset = b->CreateLoad(sOffset);
    IntegerType * type = cast<IntegerType>(offset->getType());
    ConstantInt * shift = ConstantInt::get(type, std::log2(wordWidth));
    Value * shiftedOffset = b->CreateLShr(offset, shift);
    Value * extender = b->CreateExtractElement(extenders, shiftedOffset);
    b->CreateStore(extender, sExtender);
}


void LZ4IndexDecoderKernel::generateProduceOutput(const std::unique_ptr<KernelBuilder> &b) {
    Value * producedItem = b->getProducedItemCount("literalIndexes");

    // producedItem % blockWidth (as blockWidth is always a power of 2)
    Value * outputOffset = b->CreateAnd(b->CreateTrunc(producedItem, b->getInt32Ty()), b->getInt32(b->getBitBlockWidth() - 1));
    Value * baseLiteralStartPtr = b->getOutputStreamBlockPtr("literalIndexes", b->getInt32(0));

    Value * literalStartPtr = getOutputPtr(b, baseLiteralStartPtr, outputOffset);
    Value * literalLengthPtr = getOutputPtr(b,
            b->getOutputStreamBlockPtr("literalIndexes", b->getInt32(1)), outputOffset);
    Value * matchOffsetPtr = getOutputPtr(b,
            b->getOutputStreamBlockPtr("matchIndexes", b->getInt32(0)), outputOffset);
    Value * matchLengthPtr = getOutputPtr(b,
            b->getOutputStreamBlockPtr("matchIndexes", b->getInt32(1)), outputOffset);

    b->CreateStore(b->getScalarField("LiteralStart"), literalStartPtr);
    b->CreateStore(b->getScalarField("LiteralLength"), literalLengthPtr);
    b->CreateStore(b->getScalarField("MatchOffset"), matchOffsetPtr);
    b->CreateStore(b->getScalarField("MatchLength"), matchLengthPtr);
    b->setProducedItemCount("literalIndexes", b->CreateAdd(producedItem, b->getSize(1)));
    // matchIndexes has a fixed ratio of 1:1 w.r.t. literalIndexes.
}


void LZ4IndexDecoderKernel::generateDoBlockMethod(const std::unique_ptr<KernelBuilder> & b) {
    BasicBlock * entry_block = b->GetInsertBlock();
    BasicBlock * exit_block = b->CreateBasicBlock("exit");

    // %entry
    b->SetInsertPoint(entry_block);
    printRTDebugMsg("entry");
    // Global positions in the byte stream.
    Value * blockNo = b->getScalarField("BlockNo");
    blockStartPos = b->CreateMul(blockNo, b->getInt32(b->getBitBlockWidth()), "blockStartPos");
    extenders = b->CreateBitCast(
            b->loadInputStreamBlock("extenders", b->getInt32(0)),
            VectorType::get(b->getSizeTy(), b->getBitBlockWidth() / b->getSizeTy()->getBitWidth()),
            "extenders");
    // Create a series of stack variables which will be promoted by mem2reg.
    sOffset = createStackVar(b, b->getInt32Ty(), "offset");
    // tempLength has different meanings in different states.
    sTempLength = createStackVar(b, b->getInt32Ty(), "tempLength", b->getScalarField("TempLength"));
    sTempCount = createStackVar(b, b->getInt32Ty(), "tempCount", b->getScalarField("TempCount"));
    sState = createStackVar(b, b->getInt8Ty(), "state", b->getScalarField("State"));
    sExtender = createStackVar(b, b->getSizeTy(), "extender",
            b->CreateExtractElement(extenders, b->getInt32(0)));

    BasicBlock * skippingBytes = b->CreateBasicBlock("skipping_bytes");
    BasicBlock * dispatch = b->CreateBasicBlock("dispatch");

    b->CreateCondBr(
            b->CreateICmpUGT(b->getScalarField("BytesToSkip"), b->getInt32(0)),
            skippingBytes, dispatch
            );

    // %skipping_bytes
    generateSkippingBytes(b, skippingBytes, exit_block);
    // Insert point is at the end of skippingBytes.
    b->CreateBr(dispatch);

    // %dispatch
    // Indirect branching will be added to %dispatch at last.

    // %at_block_checksum
    BasicBlock * atBlockChecksum = b->CreateBasicBlock("at_block_checksum");
    generateAtBlockChecksum(b, atBlockChecksum, skippingBytes);
  
    // %at_block_size
    BasicBlock * atBlockSize = b->CreateBasicBlock("at_block_size");
    generateAtBlockSize(b, atBlockSize, skippingBytes, exit_block);

    // %at_token
    BasicBlock * atToken = b->CreateBasicBlock("at_token");
    generateAtToken(b, atToken, exit_block);

    // %extending_literal_length
    BasicBlock * extendingLiteralLen = b->CreateBasicBlock("extending_literal_length");
    generateExtendingLiteralLen(b, extendingLiteralLen, exit_block);

    // %at_literals
    BasicBlock * atLiterals = b->CreateBasicBlock("at_literals");
    generateAtLiterals(b, atLiterals);
    b->CreateBr(skippingBytes);

    // %at_first_offset
    // Note that the last sequence is incomplete and ends with literals.
    // If the whole LZ4 block is done, process the (optional) checksum.
    // Otherwise, go around to process the next sequence.
    BasicBlock * atOffset1 = b->CreateBasicBlock("at_first_offset");
    b->SetInsertPoint(atOffset1);
    Value * nowGlobalPos = b->CreateAdd(blockStartPos, b->CreateLoad(sOffset));
    BasicBlock * blockEnd_else = b->CreateBasicBlock("block_end_else");
    // Conditional branch inserted at the end of the last block.
    b->CreateUnlikelyCondBr(
            b->CreateICmpEQ(nowGlobalPos, b->getScalarField("LZ4BlockEnd")),
            atBlockChecksum, blockEnd_else
            );
    generateAtFirstOffset(b, blockEnd_else, exit_block);

    // %at_second_offset
    BasicBlock * atOffset2 = b->CreateBasicBlock("at_second_offset");
    generateAtSecondOffset(b, atOffset2, exit_block);

    // %extending_match_length
    BasicBlock * extendingMatchLen = b->CreateBasicBlock("extending_match_length");
    generateExtendingMatchLen(b, extendingMatchLen, exit_block);
    b->CreateBr(atToken);

    // Indirect branching.
    b->SetInsertPoint(dispatch);
    printRTDebugMsg("dispatch");
    // The order must comply with enum State.
    Constant * labels = ConstantVector::get(
            {BlockAddress::get(atBlockSize), BlockAddress::get(atToken), BlockAddress::get(extendingLiteralLen), BlockAddress::get(atLiterals),
             BlockAddress::get(atOffset1), BlockAddress::get(atOffset2), BlockAddress::get(extendingMatchLen), BlockAddress::get(atBlockChecksum)}
            );
    Value * target = b->CreateExtractElement(labels, b->CreateLoad(sState));
    IndirectBrInst * indirectBr = b->CreateIndirectBr(target);
    indirectBr->addDestination(atBlockSize);
    indirectBr->addDestination(atToken);
    indirectBr->addDestination(extendingLiteralLen);
    indirectBr->addDestination(atLiterals);
    indirectBr->addDestination(atOffset1);
    indirectBr->addDestination(atOffset2);
    indirectBr->addDestination(extendingMatchLen);
    indirectBr->addDestination(atBlockChecksum);

    // %exit
    b->SetInsertPoint(exit_block);
    printRTDebugMsg("exit");
    b->setScalarField("State", b->CreateLoad(sState));
    b->setScalarField("TempLength", b->CreateLoad(sTempLength));
    b->setScalarField("TempCount", b->CreateLoad(sTempCount));
    b->setScalarField("BlockNo", b->CreateAdd(blockNo, b->getInt32(1)));
    // When the kernel builder uses indirectbr, doBlock is not a separate function.
    // Hence, we branch to a new basic block and fall through instead of returning.
    BasicBlock * end_block = b->CreateBasicBlock("end_of_block");
    b->CreateBr(end_block);
    b->SetInsertPoint(end_block);
}


void LZ4IndexDecoderKernel::generateBoundaryDetection(const std::unique_ptr<KernelBuilder> & b, State state, BasicBlock * exit_block, bool updateExtenderWord) {
    if (updateExtenderWord) {
        BasicBlock * wordBoundary_then = b->CreateBasicBlock("word_boundary_then-" + StateLabels.at(state));
        BasicBlock * blockBoundary_else = b->CreateBasicBlock("block_boundary_else-" + StateLabels.at(state));
        BasicBlock * wordBoundary_cont = b->CreateBasicBlock("word_boundary_cont-" + StateLabels.at(state));
        b->CreateUnlikelyCondBr(
                b->CreateICmpEQ(getWordOffset(b), b->getInt32(0)),
                wordBoundary_then, wordBoundary_cont
                );

        b->SetInsertPoint(wordBoundary_then);
        b->CreateUnlikelyCondBr(
                b->CreateICmpEQ(b->CreateLoad(sOffset), b->getInt32(b->getBitBlockWidth())),
                exit_block, blockBoundary_else
                );

        // Reaching word boundary but not block boundary.  Update the extender word as requested.
        b->SetInsertPoint(blockBoundary_else);
        loadCurrentExtender(b);
        b->CreateBr(wordBoundary_cont);

        // Leave the insert point at the end and return.
        b->SetInsertPoint(wordBoundary_cont);
    } else {
        BasicBlock * blockBoundary_cont = b->CreateBasicBlock("block_boundary_cont-" + StateLabels.at(state));
        b->CreateUnlikelyCondBr(
                b->CreateICmpEQ(b->CreateLoad(sOffset), b->getInt32(b->getBitBlockWidth())),
                exit_block, blockBoundary_cont
                );
        // Leave the insert point at the end and return.
        b->SetInsertPoint(blockBoundary_cont);
    }
}


void LZ4IndexDecoderKernel::generateSkippingBytes(const std::unique_ptr<kernel::KernelBuilder> & b, BasicBlock * bb, BasicBlock * exit_block) {
    b->SetInsertPoint(bb);
    printRTDebugMsg("skipping bytes");

    Value * remainingBytesInBlock = b->CreateSub(
            b->getInt32(b->getBitBlockWidth()), b->CreateLoad(sOffset)
            );
    Value * remainingBytesToSkip = b->getScalarField("BytesToSkip");
    Value * advanceDist = b->CreateUMin(remainingBytesInBlock, remainingBytesToSkip);
    remainingBytesToSkip = b->CreateSub(remainingBytesToSkip, advanceDist);
    incStackVar(b, sOffset, advanceDist);
    b->setScalarField("BytesToSkip", remainingBytesToSkip);

    generateBoundaryDetection(b, State::SKIPPING_BYTES, exit_block);
    // Falls through.
}


void LZ4IndexDecoderKernel::generateAtBlockSize(const std::unique_ptr<KernelBuilder> &b, BasicBlock * bb, BasicBlock * skippingBytes, BasicBlock * exit_block) {
    b->CreateBr(bb);
    b->SetInsertPoint(bb);
    printRTDebugMsg("scanning block size");
    printGlobalPos();

    // Use tempLength to hold the block size temporarily.
    // Note that it is initially stored as big-endian (for the ease of reading) and will be "swapped" later.
    // Use tempCount as the loop counter (0..3).
    // Both variables are initialized from kernel states at %entry.

    // A do-while loop.
    BasicBlock * loopBody = b->CreateBasicBlock("blocksize_loop_body");
    BasicBlock * loopExit = b->CreateBasicBlock("blocksize_loop_exit");
    b->CreateBr(loopBody);

    b->SetInsertPoint(loopBody);
    Value * byte = loadRawByte(b);
    Value * newTempLength = b->CreateAdd(
            b->CreateShl(b->CreateLoad(sTempLength), b->getInt32(8)),
            b->CreateZExt(byte, b->getInt32Ty())
            );
    b->CreateStore(newTempLength, sTempLength);
    incStackVar(b, sTempCount);
    incStackVar(b, sOffset);
    // Stop when we read all four bytes or reach the end of the block.
    b->CreateCondBr(
            b->CreateOr(
                b->CreateICmpEQ(b->CreateLoad(sTempCount), b->getInt32(4)),
                b->CreateICmpEQ(b->CreateLoad(sOffset), b->getInt32(b->getBitBlockWidth()))
                ),
            loopExit, loopBody
            );

    b->SetInsertPoint(loopExit);
    BasicBlock * blockSizeCompleted_then = b->CreateBasicBlock("blocksize_completed_then");
    BasicBlock * blockSizeCompleted_cont = b->CreateBasicBlock("blocksize_completed_cont");
    b->CreateLikelyCondBr(
            b->CreateICmpEQ(b->CreateLoad(sTempCount), b->getInt32(4)),
            blockSizeCompleted_then, blockSizeCompleted_cont
            );

    // All four bytes of the block size are read in.
    b->SetInsertPoint(blockSizeCompleted_then);
    // Remember to swap the block size back to little-endian.
    Value * blockSize = generateBitswap(b, b->CreateLoad(sTempLength));
    Value * currentPos = b->CreateAdd(blockStartPos, b->CreateLoad(sOffset));
    b->setScalarField("LZ4BlockStart", currentPos);
    b->setScalarField("LZ4BlockEnd", b->CreateAdd(currentPos, blockSize));
    printRTDebugInt("blockSize", blockSize);

    BasicBlock * uncompressedBlock_then = b->CreateBasicBlock("uncompressed_block_then");
    BasicBlock * uncompressedBlock_else = b->CreateBasicBlock("uncompressed_block_cont");
    b->CreateUnlikelyCondBr(
            b->CreateTrunc(
                b->CreateLShr(blockSize, b->getInt32(31)),
                b->getInt1Ty()
                ),
            uncompressedBlock_then,
            uncompressedBlock_else
            );

    b->SetInsertPoint(uncompressedBlock_then);
    Value * realBlockSize = b->CreateXor(blockSize, b->getInt32(1L << 31));
    b->setScalarField("LZ4BlockEnd", b->CreateAdd(currentPos, realBlockSize));
    b->setScalarField("BytesToSkip", realBlockSize);
    b->setScalarField("LiteralStart", currentPos);
    b->setScalarField("LiteralLength", realBlockSize);
    // No need to set MatchLength/MatchOffset to 0, nor to produce output,
    // because %atBlockChecksum will do so as the last sequence.
    b->CreateStore(b->getInt8(State::AT_BLOCK_CHECKSUM), sState);
    b->CreateBr(skippingBytes);

    b->SetInsertPoint(uncompressedBlock_else);
    // Reset these temporary values for later use.
    b->CreateStore(b->getInt32(0), sTempLength);
    b->CreateStore(b->getInt32(0), sTempCount);
    b->CreateStore(b->getInt8(State::AT_TOKEN), sState);
    // A block size of 0 is the end mark of the frame. Exit.
    b->CreateUnlikelyCondBr(
            b->CreateICmpEQ(blockSize, ConstantInt::getNullValue(blockSize->getType())),
            exit_block,
            blockSizeCompleted_cont
            );

    // We could be at the boundary no matter the block size is completed or not.
    b->SetInsertPoint(blockSizeCompleted_cont);
    generateBoundaryDetection(b, State::AT_BLOCK_SIZE, exit_block);
    // Falls through to %at_token.
}


void LZ4IndexDecoderKernel::generateAtToken(const std::unique_ptr<kernel::KernelBuilder> & b, BasicBlock * bb, BasicBlock * exit_block) {
    b->CreateBr(bb);
    b->SetInsertPoint(bb);
    printRTDebugMsg("reading token");

    Value * token = loadRawByte(b);
    Value * literalLen = b->CreateZExt(
        b->CreateLShr(token, b->getInt8(4)),
        b->getInt32Ty()
        );
    Value * matchLen = b->CreateZExt(
        b->CreateAnd(token, b->getInt8(0xf)),
        b->getInt32Ty()
        );
    incStackVar(b, sOffset);
    // Prepare extender word for scanning.
    loadCurrentExtender(b);
    setExtenderUntilOffset(b);
    // Store the (partial) match length to be extended later.
    b->setScalarField("MatchLength", matchLen);
    // Use tempLength to accumulate extended lengths (until at_literals).
    b->CreateStore(literalLen, sTempLength);
    b->CreateStore(b->getInt8(State::EXTENDING_LITERAL_LENGTH), sState);

    generateBoundaryDetection(b, State::AT_TOKEN, exit_block);
    // Falls through to %extending_literal_length.
}


void LZ4IndexDecoderKernel::generateExtendingLiteralLen(const std::unique_ptr<KernelBuilder> & b, BasicBlock * bb, BasicBlock * exit_block) {
    b->CreateBr(bb);
    b->SetInsertPoint(bb);
    printRTDebugMsg("extending literal len");

    Value * wordOffset = getWordOffset(b);
    Value * blockOffset = getWordStartOffset(b);
    Value * literalLen = b->CreateLoad(sTempLength);
    Value * literalExtEnd = b->CreateTrunc(
                b->CreateCountForwardZeroes(b->CreateNot(b->CreateLoad(sExtender))),
                b->getInt32Ty());
    printRTDebugInt("wordOffset", wordOffset);
    printRTDebugInt("literalExtEnd", literalExtEnd);
    // number of extender = literalExtEnd - wordOffset
    Value * numExtenders = b->CreateSub(literalExtEnd, wordOffset);
    Value * literalExtReachBoundary =
            b->CreateICmpEQ(literalExtEnd, b->getInt32(wordWidth));
    // There are literalExtEnd forward zeroes, we load bytes[literalExtEnd]
    // which is the first non-extender.  If literalExtEnd == 64, we force the
    // load index to be 0 to avoid out-of-bound access, and lastByte will be 0.
    Value * loadOffset = b->CreateSelect(literalExtReachBoundary,
            ConstantInt::getNullValue(literalExtEnd->getType()),
            literalExtEnd);
    Value * lastByte = b->CreateSelect(literalExtReachBoundary,
            b->getInt8(0),
            loadRawByte(b, b->CreateAdd(blockOffset, loadOffset)));
    Value * literalLenExted = b->CreateICmpUGE(literalLen, b->getInt32(0xf));
    literalLen = b->CreateSelect(literalLenExted,
            b->CreateAdd(
                literalLen,
                b->CreateAdd(
                    b->CreateMul(numExtenders, b->getInt32(0xff)),
                    b->CreateZExt(lastByte, b->getInt32Ty())
                    )
                ),      // literalLen + numExtenders * 255
            literalLen);
    wordOffset = b->CreateSelect(literalLenExted,
            literalExtEnd,
            wordOffset);
    // If lastByte is truly the last length byte, we need to advance the cursor by 1.
    wordOffset = b->CreateSelect(
            b->CreateAnd(literalLenExted, b->CreateNot(literalExtReachBoundary)),
            b->CreateAdd(wordOffset, b->getInt32(1)),
            wordOffset
            );
    b->CreateStore(literalLen, sTempLength);
    b->CreateStore(b->CreateAdd(blockOffset, wordOffset), sOffset);
    Value * unfinished = b->CreateAnd(literalExtReachBoundary, literalLenExted);
    Value * newState = b->CreateSelect(unfinished,
            b->getInt8(State::EXTENDING_LITERAL_LENGTH),
            b->getInt8(State::AT_LITERALS));
    b->CreateStore(newState, sState);

    generateBoundaryDetection(b, State::EXTENDING_LITERAL_LENGTH, exit_block, true);
    BasicBlock * cont_block = b->CreateBasicBlock("finished_" + StateLabels.at(State::EXTENDING_LITERAL_LENGTH));
    // Insert point is still in wordBoundary block now.
    // See if there are still more extenders.
    b->CreateUnlikelyCondBr(unfinished, bb, cont_block);

    b->SetInsertPoint(cont_block);
    // Falls through to %at_literals.
}


void LZ4IndexDecoderKernel::generateAtLiterals(const std::unique_ptr<KernelBuilder> & b, BasicBlock * bb) {
    b->CreateBr(bb);
    b->SetInsertPoint(bb);
    b->setScalarField("LiteralStart", b->CreateAdd(blockStartPos, b->CreateLoad(sOffset)));
    b->setScalarField("LiteralLength", b->CreateLoad(sTempLength));
    b->setScalarField("BytesToSkip", b->CreateLoad(sTempLength));
    b->CreateStore(b->getInt8(State::AT_FIRST_OFFSET), sState);

    // No boundary detection here as we do not advance the cursor.
    // Control flow will be redirected to %skipping_bytes later.
}


void LZ4IndexDecoderKernel::generateAtFirstOffset(const std::unique_ptr<KernelBuilder> &b, BasicBlock * bb, BasicBlock * exit_block) {
    b->SetInsertPoint(bb);
    printRTDebugMsg("reading first offset");

    Value * byte = b->CreateZExt(loadRawByte(b), b->getInt32Ty());
    // Use tempLength to store partial offset.
    b->CreateStore(byte, sTempLength);
    incStackVar(b, sOffset);
    b->CreateStore(b->getInt8(State::AT_SECOND_OFFSET), sState);

    generateBoundaryDetection(b, State::AT_FIRST_OFFSET, exit_block);
    // Falls through to %at_second_offset.
}


void LZ4IndexDecoderKernel::generateAtSecondOffset(const std::unique_ptr<KernelBuilder> & b, BasicBlock * bb, BasicBlock * exit_block) {
    b->CreateBr(bb);
    b->SetInsertPoint(bb);
    printRTDebugMsg("reading second offset");

    Value * byte1 = b->CreateLoad(sTempLength);
    Value * byte2 = b->CreateZExt(loadRawByte(b), b->getInt32Ty());
    Value * offset = b->CreateAdd(
            b->CreateShl(byte2, b->getInt32(8)),
            byte1
            );
    b->setScalarField("MatchOffset", offset);
    incStackVar(b, sOffset);
    // Prepare extender word and tempLength for extending.
    loadCurrentExtender(b);
    setExtenderUntilOffset(b);
    b->CreateStore(b->getScalarField("MatchLength"), sTempLength);
    b->CreateStore(b->getInt8(State::EXTENDING_MATCH_LENGTH), sState);

    generateBoundaryDetection(b, State::AT_SECOND_OFFSET, exit_block);
    // Falls through to %extending_match_length.
}


void LZ4IndexDecoderKernel::generateExtendingMatchLen(const std::unique_ptr<KernelBuilder> & b, BasicBlock * bb, BasicBlock * exit_block) {
    b->CreateBr(bb);
    b->SetInsertPoint(bb);
    printRTDebugMsg("extending match length");
    printGlobalPos();
    printRTDebugInt("rawbyte", loadRawByte(b));
    printRTDebugInt("extword", b->CreateLoad(sExtender));

    Value * wordOffset = getWordOffset(b);
    Value * blockOffset = getWordStartOffset(b);
    Value * matchLen = b->CreateLoad(sTempLength);
    Value * matchExtEnd = b->CreateTrunc(
        b->CreateCountForwardZeroes(b->CreateNot(b->CreateLoad(sExtender))),
        b->getInt32Ty()
        );
    printRTDebugInt("wordoffset", wordOffset);
    printRTDebugInt("matchExtEnd", matchExtEnd);
    // number of extender = matchExtEnd - wordOffset
    Value * numExtenders = b->CreateSub(matchExtEnd, wordOffset);
    Value * matchExtReachBoundary = 
            b->CreateICmpEQ(matchExtEnd, b->getInt32(wordWidth));
    // There are matchExtEnd forward zeroes, we load bytes[matchExtEnd]
    // which is the first non-extender.  If matchExtEnd == 64, we force the
    // load index to be 0 to avoid out-of-bound access, and lastByte will be 0.
    Value * loadOffset = b->CreateSelect(matchExtReachBoundary,
            ConstantInt::getNullValue(matchExtEnd->getType()),
            matchExtEnd);
    Value * lastByte = b->CreateSelect(matchExtReachBoundary,
            b->getInt8(0),
            loadRawByte(b, b->CreateAdd(blockOffset, loadOffset)));
    Value * matchLenExted = b->CreateICmpUGE(matchLen, b->getInt32(0xf));
    matchLen = b->CreateSelect(matchLenExted,
            b->CreateAdd(
                matchLen,
                b->CreateAdd(
                    b->CreateMul(numExtenders, b->getInt32(0xff)),
                    b->CreateZExt(lastByte, b->getInt32Ty())
                    )
                ),      // matchLen + numExtenders * 255
            matchLen);
    wordOffset = b->CreateSelect(matchLenExted,
            matchExtEnd,
            wordOffset);
    // If lastByte is truly the last length byte, we need to advance the cursor by 1.
    wordOffset = b->CreateSelect(
            b->CreateAnd(matchLenExted, b->CreateNot(matchExtReachBoundary)),
            b->CreateAdd(wordOffset, b->getInt32(1)),
            wordOffset
            );
    b->CreateStore(matchLen, sTempLength);
    b->CreateStore(b->CreateAdd(blockOffset, wordOffset), sOffset);

    Value * unfinished = b->CreateAnd(matchExtReachBoundary, matchLenExted);
    BasicBlock * output_then = b->CreateBasicBlock("output_then");
    BasicBlock * output_cont = b->CreateBasicBlock("output_cont");
    b->CreateLikelyCondBr(
            b->CreateNot(unfinished),
            output_then, output_cont
            );
    b->SetInsertPoint(output_then);
    b->CreateStore(b->getInt8(State::AT_TOKEN), sState);
    matchLen = b->CreateAdd(matchLen, b->getInt32(4));    // Add the constant at the end.
    b->setScalarField("MatchLength", matchLen);
    generateProduceOutput(b);
    b->CreateBr(output_cont);

    b->SetInsertPoint(output_cont);
    generateBoundaryDetection(b, State::EXTENDING_MATCH_LENGTH, exit_block, true);
    BasicBlock * cont_block = b->CreateBasicBlock("finished_" + StateLabels.at(State::EXTENDING_MATCH_LENGTH));
    // Insert point is still in wordBoundary block now.
    // See if there are still more extenders.
    b->CreateUnlikelyCondBr(unfinished, bb, cont_block);

    b->SetInsertPoint(cont_block);
}


void LZ4IndexDecoderKernel::generateAtBlockChecksum(const std::unique_ptr<KernelBuilder> & b, BasicBlock * bb, BasicBlock * skippingBytes) {
    // No branch here as we have made a conditional branch outside.
    b->SetInsertPoint(bb);
    printRTDebugMsg("processing block checksum");

    // Produce the partial output (fill matchIndexes with 0).
    b->setScalarField("MatchOffset", b->getInt32(0));
    b->setScalarField("MatchLength", b->getInt32(0));
    generateProduceOutput(b);

    BasicBlock * hasChecksum_then = b->CreateBasicBlock("has_checksum_then");
    BasicBlock * hasChecksum_cont = b->CreateBasicBlock("has_checksum_cont");

    b->CreateStore(b->getInt8(State::AT_BLOCK_SIZE), sState);
    b->CreateCondBr(b->getScalarField("hasBlockChecksum"), hasChecksum_then, hasChecksum_cont);

    b->SetInsertPoint(hasChecksum_then);
    b->setScalarField("BytesToSkip", b->getInt32(4));
    b->CreateBr(skippingBytes);
    // Boundary detection will be done in skipping_bytes.

    b->SetInsertPoint(hasChecksum_cont);
    // No checksum, offset not advanced.  Falls through to the next block (block_size).
}

LZ4IndexDecoderKernel::LZ4IndexDecoderKernel(const std::unique_ptr<kernel::KernelBuilder> & b,
                                             // arguments
                                             Scalar * hasBlockChecksum,
                                             // inputs
                                             StreamSet * byteStream,
                                             StreamSet * extenders,
                                             // outputs
                                             StreamSet * literalIndexes,
                                             StreamSet * matchIndexes)
: BlockOrientedKernel("lz4IndexDecoder",
// Inputs
{Binding{"byteStream", byteStream, FixedRate(), Misaligned()},
 Binding{"extenders", extenders}},
// Outputs: literal start, literal length, match offset, match length
{Binding{"literalIndexes", literalIndexes, UnknownRate()},
 Binding{"matchIndexes", matchIndexes, RateEqualTo("literalIndexes")}},
// Arguments
{Binding{"hasBlockChecksum", hasBlockChecksum}},
{},
// Internal states:
{Binding{b->getInt32Ty(), "BlockNo"},
 Binding{b->getInt8Ty(), "State"},
 Binding{b->getInt32Ty(), "LZ4BlockStart"},
 Binding{b->getInt32Ty(), "LZ4BlockEnd"},
 Binding{b->getInt32Ty(), "BytesToSkip"},
 Binding{b->getInt32Ty(), "TempLength"},
 Binding{b->getInt32Ty(), "TempCount"},
 Binding{b->getInt32Ty(), "LiteralStart"},
 Binding{b->getInt32Ty(), "LiteralLength"},
 Binding{b->getInt32Ty(), "MatchOffset"},
 Binding{b->getInt32Ty(), "MatchLength"}})
, wordWidth{b->getSizeTy()->getBitWidth()} {

}
