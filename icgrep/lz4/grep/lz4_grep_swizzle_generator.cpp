
#include "lz4_grep_swizzle_generator.h"
#include <kernels/swizzle.h>
#include <kernels/lz4/decompression/lz4_swizzled_decompression.h>
#include <kernels/kernel_builder.h>

using namespace parabix;
using namespace kernel;

StreamSetBuffer *LZ4GrepSwizzleGenerator::generateUncompressedBitStreams() {
    StreamSetBuffer *compressedByteStream = nullptr, *compressedBasisBits = nullptr;
    std::tie(compressedByteStream, compressedBasisBits) = this->loadByteStreamAndBitStream();
    return this->swizzledDecompression(compressedByteStream, compressedBasisBits);
}

StreamSetBuffer *LZ4GrepSwizzleGenerator::decompressBitStream(parabix::StreamSetBuffer *compressedByteStream,
                                                                       parabix::StreamSetBuffer *compressedBitStream) {
    return this->convertCompressedBitsStreamWithSwizzledAioApproach(compressedByteStream, compressedBitStream, "combined");
}


StreamSetBuffer * LZ4GrepSwizzleGenerator::convertCompressedBitsStreamWithSwizzledAioApproach(
        StreamSetBuffer *compressedByteStream,
        StreamSetBuffer *compressedBitStream,
        std::string prefix
) {
    auto mGrepDriver = &mPxDriver;
    auto & b = mGrepDriver->getBuilder();

    LZ4BlockInfo blockInfo = this->getBlockInfo(compressedByteStream);

    // Produce unswizzled bit streams
    StreamSetBuffer * u16Swizzle0 = mPxDriver.addBuffer<StaticBuffer>(b, b->getStreamSetTy(4),
                                                                      this->getDefaultBufferBlocks(), 1);
    Kernel * unSwizzleK = mPxDriver.addKernelInstance<SwizzleGenerator>(b, 4, 1, 1, 64, "source");
    mPxDriver.makeKernelCall(unSwizzleK, {compressedBitStream}, {u16Swizzle0});

    StreamSetBuffer * uncompressedSwizzled0 = mPxDriver.addBuffer<StaticBuffer>(b, b->getStreamSetTy(4),
                                                                                this->getDefaultBufferBlocks(), 1);


    Kernel* lz4AioK = mPxDriver.addKernelInstance<LZ4SwizzledDecompressionKernel>(b, 4, 1, 4);
    lz4AioK->setInitialArguments({mFileSize});
    mPxDriver.makeKernelCall(
            lz4AioK,
            {
                    compressedByteStream,

                    // Block Data
                    blockInfo.isCompress,
                    blockInfo.blockStart,
                    blockInfo.blockEnd,

                    u16Swizzle0,
            }, {
                    uncompressedSwizzled0,
            });



    StreamSetBuffer * const decompressionBitStream = mPxDriver.addBuffer<StaticBuffer>(b, b->getStreamSetTy(8, 1),
                                                                                       this->getDefaultBufferBlocks(), 1);
    Kernel * unSwizzleK2 = mPxDriver.addKernelInstance<SwizzleGenerator>(b, 4, 1, 1, 64, "dst");
    mPxDriver.makeKernelCall(unSwizzleK2, {uncompressedSwizzled0}, {decompressionBitStream});

    return decompressionBitStream;

}