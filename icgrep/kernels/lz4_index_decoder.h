/*
 *  Copyright (c) 2017 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#ifndef LZ4_INDEX_DECODER_H
#define LZ4_INDEX_DECODER_H

#include <map>

#include "kernel.h"

namespace llvm {
    class Module;
    class Function;
    class BasicBlock;
    class Value;
}
namespace IDISA { class IDISA_Builder; }

namespace kernel {

class LZ4IndexDecoderKernel : public BlockOrientedKernel {
public:
    LZ4IndexDecoderKernel(IDISA::IDISA_Builder * iBuilder);
protected:
    void generateDoBlockMethod() override;
private:
    bool useIndirectBr() const override;

    enum State : unsigned char {
        AT_BLOCK_SIZE = 0,
        AT_TOKEN = 1,
        EXTENDING_LITERAL_LENGTH = 2,
        AT_LITERALS = 3,
        AT_FIRST_OFFSET = 4,
        AT_SECOND_OFFSET = 5,
        EXTENDING_MATCH_LENGTH = 6,
        AT_BLOCK_CHECKSUM = 7,
        SKIPPING_BYTES = 255,   // not included in the indirectbr table
    };

    const std::map<enum State, std::string> StateLabels = {
        {State::AT_BLOCK_SIZE, "at_block_size"},
        {State::AT_TOKEN, "at_token"},
        {State::EXTENDING_LITERAL_LENGTH, "extending_literal_length"},
        {State::AT_LITERALS, "at_literals"},
        {State::AT_FIRST_OFFSET, "at_first_offset"},
        {State::AT_SECOND_OFFSET, "at_second_offset"},
        {State::EXTENDING_MATCH_LENGTH, "extending_match_length"},
        {State::AT_BLOCK_CHECKSUM, "at_block_checksum"},
        {State::SKIPPING_BYTES, "skipping_bytes"},
    };

    const size_t wordWidth;

    // Some variables that are constant (at least in each bit block).
    llvm::Value * blockStartPos;        // The global position of the start of the block.
    llvm::Value * extenders;    // Bitcasted extender stream.
 
    // Stack variables (will be promoted by mem2reg).
    llvm::Value * sOffset;      // offset within the current bit block
    llvm::Value * sTempLength;  // tempLength have different purposes in different states.
    llvm::Value * sTempCount;   // tempCount is used as a loop counter.
    llvm::Value * sExtender;    // Current extender word.
    llvm::Value * sState;

    // Helper methods.
    llvm::Value * getWordOffset();
    llvm::Value * getWordStartOffset();
    llvm::Value * loadRawByte(llvm::Value * offset);
    void setExtenderUntilOffset();
    void loadCurrentExtender();

    void generateProduceOutput();
    void generateBoundaryDetection(State state, llvm::BasicBlock * exit_block, bool updateExtenderWord);
    // Generate basic blocks for each state.
    void generateSkippingBytes(llvm::BasicBlock * bb, llvm::BasicBlock * exit_block);
    void generateAtBlockSize(llvm::BasicBlock * bb, llvm::BasicBlock * skippingBytes, llvm::BasicBlock * exit_block);
    void generateAtToken(llvm::BasicBlock * bb, llvm::BasicBlock * exit_block);
    void generateExtendingLiteralLen(llvm::BasicBlock * bb, llvm::BasicBlock * exit_block);
    void generateAtLiterals(llvm::BasicBlock * bb);
    void generateAtFirstOffset(llvm::BasicBlock * bb, llvm::BasicBlock * exit_block);
    void generateAtSecondOffset(llvm::BasicBlock * bb, llvm::BasicBlock * exit_block);
    void generateExtendingMatchLen(llvm::BasicBlock * bb, llvm::BasicBlock * exit_block);
    void generateAtBlockChecksum(llvm::BasicBlock * bb, llvm::BasicBlock * skippingBytes);
};

}

#endif // LZ4_INDEX_DECODER_H
