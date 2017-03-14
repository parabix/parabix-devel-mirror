/*
 *  Copyright (c) 2017 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 */

#include "alignedprint.h"
#include <IR_Gen/idisa_builder.h>  // for IDISA_Builder
#include <llvm/IR/Module.h>

using namespace llvm;

namespace kernel {

    void ap_p2s_step(IDISA::IDISA_Builder * iBuilder, Value * p0, Value * p1, Value * hi_mask, unsigned shift, Value * &s1, Value * &s0) {
    Value * t0 = iBuilder->simd_if(1, hi_mask, p0, iBuilder->simd_srli(16, p1, shift));
    Value * t1 = iBuilder->simd_if(1, hi_mask, iBuilder->simd_slli(16, p0, shift), p1);
    s1 = iBuilder->esimd_mergeh(8, t1, t0);
    s0 = iBuilder->esimd_mergel(8, t1, t0);
}

void p2s(IDISA::IDISA_Builder * iBuilder, Value * p[], Value * s[]) {
    Value * bit00004444[2];
    Value * bit22226666[2];
    Value * bit11115555[2];
    Value * bit33337777[2];
    ap_p2s_step(iBuilder, p[0], p[4], iBuilder->simd_himask(8), 4, bit00004444[1], bit00004444[0]);
    ap_p2s_step(iBuilder, p[1], p[5], iBuilder->simd_himask(8), 4, bit11115555[1], bit11115555[0]);
    ap_p2s_step(iBuilder, p[2], p[6], iBuilder->simd_himask(8), 4, bit22226666[1], bit22226666[0]);
    ap_p2s_step(iBuilder, p[3], p[7], iBuilder->simd_himask(8), 4, bit33337777[1], bit33337777[0]);
    Value * bit00224466[4];
    Value * bit11335577[4];
    for (unsigned j = 0; j<2; j++) {
        ap_p2s_step(iBuilder, bit00004444[j], bit22226666[j],iBuilder->simd_himask(4), 2, bit00224466[2*j+1], bit00224466[2*j]);
        ap_p2s_step(iBuilder, bit11115555[j], bit33337777[j],iBuilder->simd_himask(4), 2, bit11335577[2*j+1], bit11335577[2*j]);
    }
    for (unsigned j = 0; j<4; j++) {
        ap_p2s_step(iBuilder, bit00224466[j], bit11335577[j], iBuilder->simd_himask(2), 1, s[2*j+1], s[2*j]);
    }
}

void PrintableBits::generateDoBlockMethod() {
    // Load current block
    Value * bitStrmVal = loadInputStreamBlock("bitStream", iBuilder->getInt32(0));

    Value * bits[8];

    /*
    00110001 is the Unicode codepoint for '1' and 00101110 is the codepoint for '.'.
    We want to output a byte stream that is aligned with the input bitstream such that it contains 00110001 in each 1 position and 00101110 in each 0 position.
    
    For example, consider input bitstream 101. Our desired output is:
    00110001 00101110 00110001

    We can do the bitstream to bytestream conversion in parallel by viewing the output stream in terms of parallel bit streams.

    0   0   0 -> First bit position of every byte is all zeros
    0   0   0 -> Same for second bit
    1   1   1 -> Third bit is all ones
    1   0   1 -> 4th bit is 1 for a '1' byte and '0' for a zero byte. Matches input bit stream
    0   1   0 -> opposite
    0   1   0 -> opposite
    0   1   0 -> opposite
    1   0   1 -> same as 4th bit position. 
    
    Armed with the above we can do the bit->byte conversion all at once
    rather than byte at a time! That's what we do below.
    */

    bits[0] = ConstantInt::getNullValue(iBuilder->getBitBlockType());
    bits[1] = ConstantInt::getNullValue(iBuilder->getBitBlockType());
    bits[2] = ConstantInt::getAllOnesValue(iBuilder->getBitBlockType());
    bits[3] = bitStrmVal;
    Value * negBitStrmVal = iBuilder->simd_not(bitStrmVal);
    bits[4] = negBitStrmVal;
    bits[5] = negBitStrmVal;
    bits[6] = negBitStrmVal;
    bits[7] = bitStrmVal;
    
    // Reassemble the paralell bit streams into a byte stream
    Value * printableBytes[8];
    p2s(iBuilder, bits, printableBytes);
    
    for (unsigned j = 0; j < 8; ++j) {
        storeOutputStreamPack("byteStream", iBuilder->getInt32(0), iBuilder->getInt32(j), iBuilder->bitCast(printableBytes[j]));
    }
}

void SelectStream::generateDoBlockMethod() {
    if (mStreamIndex >= mSizeInputStreamSet)
        llvm::report_fatal_error("Stream index out of bounds.\n");
    
    Value * bitStrmVal = loadInputStreamBlock("bitStreams", iBuilder->getInt32(mStreamIndex));

    storeOutputStreamBlock("bitStream", iBuilder->getInt32(0), bitStrmVal);
}

void PrintStreamSet::generateDoBlockMethod() {

    /*
    00110001 is the Unicode codepoint for '1' and 00101110 is the codepoint for '.'.
    We want to output a byte stream that is aligned with the input bitstream such that it contains 00110001 in each 1 position and 00101110 in each 0 position.

    For example, consider input bitstream 101. Our desired output is:
    00110001 00101110 00110001

    We can do the bitstream to bytestream conversion in parallel by viewing the output stream in terms of parallel bit streams.

    0   0   0 -> First bit position of every byte is all zeros
    0   0   0 -> Same for second bit
    1   1   1 -> Third bit is all ones
    1   0   1 -> 4th bit is 1 for a '1' byte and '0' for a zero byte. Matches input bit stream
    0   1   0 -> opposite
    0   1   0 -> opposite
    0   1   0 -> opposite
    1   0   1 -> same as 4th bit position.

    Armed with the above we can do the bit->byte conversion all at once
    rather than byte at a time! That's what we do below.
    */

    for (const std::string & name : mNames) {

        BasicBlock * entry = iBuilder->GetInsertBlock();

        Value * count = getInputStreamSetCount(name);
        ConstantInt * const streamLength = iBuilder->getSize(iBuilder->getBitBlockWidth() + mNameWidth + 1);
        Value * output = iBuilder->CreateAlloca(iBuilder->getInt8Ty(), streamLength);

        Value * outputName = iBuilder->CreateGlobalStringPtr(name.c_str());
        ConstantInt * const length = iBuilder->getInt32(name.length());
        iBuilder->CreateMemCpy(output, outputName, length, 1);
        iBuilder->CreateMemSet(iBuilder->CreateGEP(output, iBuilder->getInt32(name.length())), iBuilder->getInt8(' '), iBuilder->getInt32(mNameWidth - name.length()), 1);
        iBuilder->CreateStore(iBuilder->getInt8(10), iBuilder->CreateGEP(output, iBuilder->getInt32(iBuilder->getBitBlockWidth() + mNameWidth)));

        if (isa<ConstantInt>(count) && cast<ConstantInt>(count)->isOne()) {

            // Load current block
            Value * const input = loadInputStreamBlock(name, iBuilder->getInt32(0));

            Value * bits[8];
            bits[0] = ConstantInt::getNullValue(iBuilder->getBitBlockType());
            bits[1] = ConstantInt::getNullValue(iBuilder->getBitBlockType());
            bits[2] = ConstantInt::getAllOnesValue(iBuilder->getBitBlockType());
            bits[3] = input;
            Value * const negated = iBuilder->simd_not(input);
            bits[4] = negated;
            bits[5] = negated;
            bits[6] = negated;
            bits[7] = input;

            // Reassemble the paralell bit streams into a byte stream
            Value * printableBytes[8];
            p2s(iBuilder, bits, printableBytes);
            for (unsigned k = 0; k < 8; ++k) {
                const auto offset = mNameWidth + (k * (iBuilder->getBitBlockWidth() / 8));
                for (unsigned t = 0; t < (iBuilder->getBitBlockWidth() / 8); ++t) {
                    iBuilder->CreateStore(iBuilder->CreateExtractElement(printableBytes[k], iBuilder->getInt32(t)), iBuilder->CreateGEP(output, iBuilder->getInt32(offset + t)));
                }
            }

            iBuilder->CreateWriteCall(iBuilder->getInt32(1), output, streamLength);

        } else {

            iBuilder->CreateStore(iBuilder->getInt8('['), iBuilder->CreateGEP(output, length));

            BasicBlock * cond = CreateBasicBlock("cond");

            BasicBlock * getIntLength = CreateBasicBlock("getIntLength");

            BasicBlock * writeInt = CreateBasicBlock("writeInt");
            BasicBlock * writeVector = CreateBasicBlock("writeVector");

            BasicBlock * exit = CreateBasicBlock("exit");

            ConstantInt * TEN = iBuilder->getSize(10);
            ConstantInt * ONE = iBuilder->getSize(1);

            iBuilder->CreateBr(cond);
            iBuilder->SetInsertPoint(cond);
            PHINode * i = iBuilder->CreatePHI(iBuilder->getSizeTy(), 2, "i");
            i->addIncoming(iBuilder->getSize(0), entry);

            iBuilder->CreateCondBr(iBuilder->CreateICmpNE(i, count), getIntLength, exit);
            // -------------------------------------------------------------------------
            iBuilder->SetInsertPoint(getIntLength);

            PHINode * l = iBuilder->CreatePHI(iBuilder->getSizeTy(), 2, "l");
            l->addIncoming(iBuilder->getSize(name.length() + 1), cond);
            PHINode * temp = iBuilder->CreatePHI(iBuilder->getSizeTy(), 2, "temp");
            temp->addIncoming(i, cond);

            l->addIncoming(iBuilder->CreateAdd(l, ONE), getIntLength);

            temp->addIncoming(iBuilder->CreateUDiv(temp, TEN), getIntLength);

            iBuilder->CreateCondBr(iBuilder->CreateICmpUGE(temp, TEN), getIntLength, writeInt);
            // -------------------------------------------------------------------------
            iBuilder->SetInsertPoint(writeInt);
            PHINode * value = iBuilder->CreatePHI(iBuilder->getSizeTy(), 2);
            value->addIncoming(i, getIntLength);

            PHINode * j = iBuilder->CreatePHI(iBuilder->getSizeTy(), 2, "j");
            j->addIncoming(l, getIntLength);
            Value * ch = iBuilder->CreateURem(value, TEN);
            ch = iBuilder->CreateTrunc(ch, iBuilder->getInt8Ty());
            ch = iBuilder->CreateAdd(ch, iBuilder->getInt8('0'));

            value->addIncoming(iBuilder->CreateUDiv(value, TEN), writeInt);
            iBuilder->CreateStore(ch, iBuilder->CreateGEP(output, j));
            j->addIncoming(iBuilder->CreateSub(j, ONE), writeInt);

            iBuilder->CreateCondBr(iBuilder->CreateICmpUGE(value, TEN), writeInt, writeVector);
            // -------------------------------------------------------------------------
            iBuilder->SetInsertPoint(writeVector);

            iBuilder->CreateStore(iBuilder->getInt8(']'), iBuilder->CreateGEP(output, iBuilder->CreateAdd(l, iBuilder->getSize(1))));

            // Load current block
            Value * const input = loadInputStreamBlock(name, i);

            Value * bits[8];
            bits[0] = ConstantInt::getNullValue(iBuilder->getBitBlockType());
            bits[1] = ConstantInt::getNullValue(iBuilder->getBitBlockType());
            bits[2] = ConstantInt::getAllOnesValue(iBuilder->getBitBlockType());
            bits[3] = input;
            Value * const negated = iBuilder->simd_not(input);
            bits[4] = negated;
            bits[5] = negated;
            bits[6] = negated;
            bits[7] = input;

            // Reassemble the paralell bit streams into a byte stream
            Value * printableBytes[8];
            p2s(iBuilder, bits, printableBytes);
            for (unsigned k = 0; k < 8; ++k) {
                const auto offset = mNameWidth + (k * (iBuilder->getBitBlockWidth() / 8));
                for (unsigned t = 0; t < (iBuilder->getBitBlockWidth() / 8); ++t) {
                    iBuilder->CreateStore(iBuilder->CreateExtractElement(printableBytes[k], iBuilder->getInt32(t)), iBuilder->CreateGEP(output, iBuilder->getInt32(offset + t)));
                }
            }

            iBuilder->CreateWriteCall(iBuilder->getInt32(1), output, streamLength);

            i->addIncoming(iBuilder->CreateAdd(i, ONE), iBuilder->GetInsertBlock());
            iBuilder->CreateBr(cond);

            // -------------------------------------------------------------------------
            iBuilder->SetInsertPoint(exit);

        }
    }



}

PrintableBits::PrintableBits(IDISA::IDISA_Builder * builder)
: BlockOrientedKernel(builder, "PrintableBits", {Binding{builder->getStreamSetTy(1), "bitStream"}}, {Binding{builder->getStreamSetTy(1, 8), "byteStream"}}, {}, {}, {}) {
    setNoTerminateAttribute(true);
}

SelectStream::SelectStream(IDISA::IDISA_Builder * builder, unsigned sizeInputStreamSet, unsigned streamIndex)
: BlockOrientedKernel(builder, "SelectStream", {Binding{builder->getStreamSetTy(sizeInputStreamSet), "bitStreams"}}, {Binding{builder->getStreamSetTy(1, 1), "bitStream"}}, {}, {}, {}), mSizeInputStreamSet(sizeInputStreamSet), mStreamIndex(streamIndex) {
    setNoTerminateAttribute(true);

}

PrintStreamSet::PrintStreamSet(IDISA::IDISA_Builder * builder, std::vector<std::string> && names, const unsigned minWidth)
: BlockOrientedKernel(builder, "PrintableStreamSet", {}, {}, {}, {}, {})
, mNames(names)
, mNameWidth(0) {
    auto width = minWidth;
    for (const std::string & name : mNames) {
        mStreamSetInputs.emplace_back(builder->getStreamSetTy(0), name);
        width = std::max<unsigned>(name.length() + 5, width);
    }
    mNameWidth = width;
}

}
