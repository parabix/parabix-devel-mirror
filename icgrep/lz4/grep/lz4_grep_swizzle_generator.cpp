
#include "lz4_grep_swizzle_generator.h"
#include <kernels/swizzle.h>
#include <kernels/lz4/decompression/lz4_swizzled_decompression.h>
#include <kernels/kernel_builder.h>
#include <kernels/pipeline_builder.h>

using namespace kernel;

StreamSet *LZ4GrepSwizzleGenerator::generateUncompressedBitStreams() {
    StreamSet *compressedByteStream = nullptr, *compressedBasisBits = nullptr;
    std::tie(compressedByteStream, compressedBasisBits) = loadByteStreamAndBitStream();
    return swizzledDecompression(compressedByteStream, compressedBasisBits);
}

StreamSet *LZ4GrepSwizzleGenerator::decompressBitStream(StreamSet * compressedByteStream, StreamSet *compressedBitStream) {
    return convertCompressedBitsStreamWithSwizzledAioApproach(compressedByteStream, compressedBitStream);
}


StreamSet * LZ4GrepSwizzleGenerator::convertCompressedBitsStreamWithSwizzledAioApproach(StreamSet *compressedByteStream, StreamSet *compressedBitStream
) {
    LZ4BlockInfo blockInfo = getBlockInfo(compressedByteStream);

    // Produce unswizzled bit streams
    StreamSet * u16Swizzle0 = mPipeline->CreateStreamSet(4);

    mPipeline->CreateKernelCall<SwizzleGenerator>(StreamSets{compressedBitStream}, StreamSets{u16Swizzle0});

    StreamSet * uncompressedSwizzled0 = mPipeline->CreateStreamSet(4);

    mPipeline->CreateKernelCall<LZ4SwizzledDecompressionKernel>(
        mFileSize,
        // inputs
        compressedByteStream, blockInfo,
        StreamSets{ u16Swizzle0 },
        // outputs
        StreamSets{ uncompressedSwizzled0 } );

    StreamSet * decompressionBitStream = mPipeline->CreateStreamSet(8 , 1);
    mPipeline->CreateKernelCall<SwizzleGenerator>(StreamSets{uncompressedSwizzled0}, StreamSets{decompressionBitStream});
    return decompressionBitStream;

}
