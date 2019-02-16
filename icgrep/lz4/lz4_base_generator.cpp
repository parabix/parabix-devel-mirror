

#include "lz4_base_generator.h"
#include <kernels/lz4/lz4_block_decoder.h>
#include <kernels/kernel_builder.h>
#include <kernels/source_kernel.h>
#include <kernels/s2p_kernel.h>
#include <kernels/p2s_kernel.h>
#include <kernels/swizzle.h>
#include <kernels/lz4/twist_kernel.h>
#include <kernels/lz4/untwist_kernel.h>
#include <kernels/lz4/decompression/lz4_bytestream_decompression.h>
#include <kernels/lz4/decompression/lz4_swizzled_decompression.h>
#include <kernels/lz4/decompression/lz4_twist_decompression.h>
#include <kernels/lz4/decompression/lz4_bitstream_decompression.h>
#include <kernels/pipeline_builder.h>

using namespace llvm;
using namespace kernel;

LZ4BaseGenerator::LZ4BaseGenerator()
: mPxDriver("lz4")
, mLz4BlockSize(4 * 1024 * 1024) {

}

StreamSet* LZ4BaseGenerator::loadByteStream() {
    StreamSet * const byteStream = mPipeline->CreateStreamSet(1, 8);
    mPipeline->CreateKernelCall<MemorySourceKernel>(mInputStream, mFileSize, byteStream);
    return byteStream;
}

StreamSet* LZ4BaseGenerator::s2p(StreamSet* byteStream) {
    StreamSet * const basisBits = mPipeline->CreateStreamSet(8, 1);
    mPipeline->CreateKernelCall<S2PKernel>(byteStream, basisBits);
    return basisBits;
}

StreamSet* LZ4BaseGenerator::p2s(StreamSet* bitStream) {
    StreamSet * const byteStream = mPipeline->CreateStreamSet(1, 8);
    mPipeline->CreateKernelCall<P2SKernel>(bitStream, byteStream);
    return byteStream;
}

std::pair<StreamSet*, StreamSet*>  LZ4BaseGenerator::loadByteStreamAndBitStream() {
    StreamSet * const byteStream = loadByteStream();
    StreamSet * const basisBits = s2p(byteStream);
    return std::make_pair(byteStream, basisBits);
}

LZ4BlockInfo LZ4BaseGenerator::getBlockInfo(StreamSet* compressedByteStream) {
    if (mBlockInfo.isCompress) {
        return mBlockInfo;
    }

    mBlockInfo.isCompress =  mPipeline->CreateStreamSet(1, 8);
    mBlockInfo.blockStart = mPipeline->CreateStreamSet(1, 64);
    mBlockInfo.blockEnd = mPipeline->CreateStreamSet(1, 64);

    mPipeline->CreateKernelCall<LZ4BlockDecoderKernel>(
                // arguments
                mHasBlockChecksum, mHeaderSize, mFileSize,
                // inputs
                compressedByteStream,
                // outputs
                mBlockInfo.isCompress,
                mBlockInfo.blockStart,
                mBlockInfo.blockEnd);

    return mBlockInfo;
}

StreamSet * LZ4BaseGenerator::byteStreamDecompression(StreamSet* compressedByteStream) {
    LZ4BlockInfo blockInfo = getBlockInfo(compressedByteStream);
    StreamSet * const decompressionByteStream = mPipeline->CreateStreamSet(1, 8);
    mPipeline->CreateKernelCall<LZ4ByteStreamDecompressionKernel>(mFileSize, compressedByteStream, blockInfo, nullptr, decompressionByteStream );
    return decompressionByteStream;
}

StreamSet * LZ4BaseGenerator::swizzledDecompression(StreamSet* compressedByteStream, StreamSet* compressedBasisBits) {
    LZ4BlockInfo blockInfo = getBlockInfo(compressedByteStream);

    // Produce unswizzled bit streams
    StreamSet * const u16Swizzle0 = mPipeline->CreateStreamSet(4);
    StreamSet * const u16Swizzle1 = mPipeline->CreateStreamSet(4);
    mPipeline->CreateKernelCall<SwizzleGenerator>(StreamSets{compressedBasisBits}, StreamSets{u16Swizzle0, u16Swizzle1});

    StreamSet * const uncompressedSwizzled0 = mPipeline->CreateStreamSet(4);
    StreamSet * const uncompressedSwizzled1 = mPipeline->CreateStreamSet(4);

    mPipeline->CreateKernelCall<LZ4SwizzledDecompressionKernel>(
        mFileSize,
        // inputs
        compressedByteStream, blockInfo,
        StreamSets{ u16Swizzle0, u16Swizzle1 },
        // outputs
        StreamSets{ uncompressedSwizzled0, uncompressedSwizzled1 } );

    StreamSet * const decompressionBitStream = mPipeline->CreateStreamSet(8);

    mPipeline->CreateKernelCall<SwizzleGenerator>(StreamSets{uncompressedSwizzled0, uncompressedSwizzled1}, StreamSets{decompressionBitStream});

    return decompressionBitStream;
}

StreamSet * LZ4BaseGenerator::bitStreamDecompression(StreamSet* compressedByteStream, StreamSet * compressedBasisBits) {
    return convertCompressedBitsStreamWithBitStreamAioApproach(compressedByteStream, {compressedBasisBits})[0];
}

StreamSets LZ4BaseGenerator::convertCompressedBitsStreamWithBitStreamAioApproach(StreamSet * compressedByteStream, StreamSets compressedBitStreams) {
    LZ4BlockInfo blockInfo = getBlockInfo(compressedByteStream);
    StreamSets outputStreams;
    outputStreams.reserve(compressedBitStreams.size());
    for (const auto & bitStream : compressedBitStreams) {
        outputStreams.push_back(mPipeline->CreateStreamSet(bitStream->getNumElements(), bitStream->getFieldWidth()));
    }
    mPipeline->CreateKernelCall<LZ4BitStreamDecompressionKernel>(mFileSize, compressedByteStream, blockInfo, compressedBitStreams, outputStreams);
    return outputStreams;
}

unsigned LZ4BaseGenerator::getBlockSizeBufferBlocks() {
    return mLz4BlockSize / codegen::BlockSize;
}

unsigned LZ4BaseGenerator::getDefaultBufferBlocks() {
    return getBlockSizeBufferBlocks() * 2; // buffer 2 LZ4 Block By Default
}

LZ4BaseGenerator::~LZ4BaseGenerator() {

}
