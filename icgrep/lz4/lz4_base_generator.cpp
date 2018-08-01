

#include "lz4_base_generator.h"
#include <kernels/lz4/lz4_block_decoder.h>
#include <kernels/kernel_builder.h>
#include <kernels/source_kernel.h>
#include <kernels/s2p_kernel.h>
#include <kernels/swizzle.h>
#include <kernels/lz4/twist_kernel.h>
#include <kernels/lz4/untwist_kernel.h>
#include <kernels/lz4/decompression/lz4_bytestream_decompression.h>
#include <kernels/lz4/decompression/lz4_swizzled_decompression.h>
#include <kernels/lz4/decompression/lz4_twist_decompression.h>
#include <kernels/lz4/decompression/lz4_bitstream_decompression.h>
#include <kernels/lz4/decompression/lz4_parallel_bytestream_decompression.h>


using namespace llvm;
using namespace parabix;
using namespace kernel;

LZ4BaseGenerator::LZ4BaseGenerator():mPxDriver("lz4"), mLz4BlockSize(4 * 1024 * 1024) {

}

StreamSetBuffer* LZ4BaseGenerator::loadByteStream() {
    auto & b = mPxDriver.getBuilder();
    StreamSetBuffer* byteStream = mPxDriver.addBuffer<ExternalBuffer>(b, b->getStreamSetTy(1, 8));
    kernel::Kernel * sourceK = mPxDriver.addKernelInstance<MemorySourceKernel>(b);
    sourceK->setInitialArguments({mInputStream, mFileSize});
    mPxDriver.makeKernelCall(sourceK, {}, {byteStream});
    return byteStream;
}

StreamSetBuffer* LZ4BaseGenerator::s2p(parabix::StreamSetBuffer* byteStream) {
    auto & b = mPxDriver.getBuilder();
    StreamSetBuffer* basisBits = mPxDriver.addBuffer<StaticBuffer>(b, b->getStreamSetTy(8, 1),
                                                             this->getDefaultBufferBlocks());
    Kernel * s2pk = mPxDriver.addKernelInstance<S2PKernel>(b, cc::BitNumbering::BigEndian);
    mPxDriver.makeKernelCall(s2pk, {byteStream}, {basisBits});
    return basisBits;
}

std::pair<parabix::StreamSetBuffer*, parabix::StreamSetBuffer*>  LZ4BaseGenerator::loadByteStreamAndBitStream() {
    StreamSetBuffer* byteStream = this->loadByteStream();
    StreamSetBuffer*  basisBits = s2p(byteStream);
    return std::make_pair(byteStream, basisBits);
}


LZ4BlockInfo LZ4BaseGenerator::getBlockInfo(StreamSetBuffer* compressedByteStream) {
    auto & b = mPxDriver.getBuilder();
    LZ4BlockInfo blockInfo;
    blockInfo.isCompress = mPxDriver.addBuffer<StaticBuffer>(b, b->getStreamSetTy(1, 8), this->getDefaultBufferBlocks(), 1);
    blockInfo.blockStart = mPxDriver.addBuffer<StaticBuffer>(b, b->getStreamSetTy(1, 64),
                                                             this->getDefaultBufferBlocks(), 1);
    blockInfo.blockEnd = mPxDriver.addBuffer<StaticBuffer>(b, b->getStreamSetTy(1, 64), this->getDefaultBufferBlocks(), 1);

    Kernel * blockDecoderK = mPxDriver.addKernelInstance<LZ4BlockDecoderKernel>(b);
    blockDecoderK->setInitialArguments({b->CreateTrunc(mHasBlockChecksum, b->getInt1Ty()), mHeaderSize, mFileSize});
    mPxDriver.makeKernelCall(blockDecoderK, {compressedByteStream}, {blockInfo.isCompress, blockInfo.blockStart, blockInfo.blockEnd});

    return blockInfo;
}

StreamSetBuffer * LZ4BaseGenerator::byteStreamDecompression(StreamSetBuffer* compressedByteStream) {
    auto & b = mPxDriver.getBuilder();
    LZ4BlockInfo blockInfo = this->getBlockInfo(compressedByteStream);

    StreamSetBuffer *const decompressionByteStream =
            mPxDriver.addBuffer<StaticBuffer>(b, b->getStreamSetTy(1, 8),
                                              this->getDefaultBufferBlocks(), 1);
    Kernel* lz4AioK = mPxDriver.addKernelInstance<LZ4ByteStreamDecompressionKernel>(b);
    lz4AioK->setInitialArguments({mFileSize});
    mPxDriver.makeKernelCall(
            lz4AioK,
            {
                    compressedByteStream,

                    // Block Data
                    blockInfo.isCompress,
                    blockInfo.blockStart,
                    blockInfo.blockEnd
            }, {
                    decompressionByteStream
            });

    return decompressionByteStream;
}

StreamSetBuffer * LZ4BaseGenerator::swizzledDecompression(
        StreamSetBuffer* compressedByteStream,
        StreamSetBuffer* compressedBasisBits
) {
    auto & b = mPxDriver.getBuilder();
    LZ4BlockInfo blockInfo = this->getBlockInfo(compressedByteStream);

    // Produce unswizzled bit streams
    StreamSetBuffer * u16Swizzle0 = mPxDriver.addBuffer<StaticBuffer>(b, b->getStreamSetTy(4),
                                                                      this->getDefaultBufferBlocks(), 1);
    StreamSetBuffer * u16Swizzle1 = mPxDriver.addBuffer<StaticBuffer>(b, b->getStreamSetTy(4),
                                                                      this->getDefaultBufferBlocks(), 1);
    Kernel * unSwizzleK = mPxDriver.addKernelInstance<SwizzleGenerator>(b, 8, 2, 1, 64, "source");
    mPxDriver.makeKernelCall(unSwizzleK, {compressedBasisBits}, {u16Swizzle0, u16Swizzle1});



    StreamSetBuffer * uncompressedSwizzled0 = mPxDriver.addBuffer<StaticBuffer>(b, b->getStreamSetTy(4),
                                                                                this->getDefaultBufferBlocks(), 1);
    StreamSetBuffer * uncompressedSwizzled1 = mPxDriver.addBuffer<StaticBuffer>(b, b->getStreamSetTy(4),
                                                                                this->getDefaultBufferBlocks(), 1);


    Kernel* lz4AioK = mPxDriver.addKernelInstance<LZ4SwizzledDecompressionKernel>(b, 4, 2, 4);
    lz4AioK->setInitialArguments({mFileSize});
    mPxDriver.makeKernelCall(
            lz4AioK,
            {
                    compressedByteStream,

                    blockInfo.isCompress,
                    blockInfo.blockStart,
                    blockInfo.blockEnd,

                    u16Swizzle0,
                    u16Swizzle1
            }, {
                    uncompressedSwizzled0,
                    uncompressedSwizzled1
            });


    StreamSetBuffer * const decompressionBitStream = mPxDriver.addBuffer<StaticBuffer>(b, b->getStreamSetTy(8, 1),
                                                                                       this->getDefaultBufferBlocks());

    Kernel * unSwizzleK2 = mPxDriver.addKernelInstance<SwizzleGenerator>(b, 8, 1, 2);
    mPxDriver.makeKernelCall(unSwizzleK2, {uncompressedSwizzled0, uncompressedSwizzled1}, {decompressionBitStream});

    return decompressionBitStream;
}

StreamSetBuffer * LZ4BaseGenerator::bitStreamDecompression(
        parabix::StreamSetBuffer* compressedByteStream,
        parabix::StreamSetBuffer* compressedBasisBits
) {
    return this->convertCompressedBitsStreamWithBitStreamAioApproach(compressedByteStream, {compressedBasisBits})[0];
}

parabix::StreamSetBuffer *LZ4BaseGenerator::parallelByteStreamDecompression(
        parabix::StreamSetBuffer *compressedByteStream,
        bool enableGather, bool enableScatter,
        int minParallelLevel) {
    auto & iBuilder = mPxDriver.getBuilder();
    LZ4BlockInfo blockInfo = this->getBlockInfo(compressedByteStream);
    StreamSetBuffer *const decompressionByteStream
            = mPxDriver.addBuffer<StaticBuffer>(iBuilder,
                                                iBuilder->getStreamSetTy(1, 8),
                                                this->getDefaultBufferBlocks(), 1);

    Kernel *lz4AioK
            = mPxDriver.addKernelInstance<LZ4ParallelByteStreamDecompressionKernel>(iBuilder, mLz4BlockSize,
                                                                                    enableGather, enableScatter,
                                                                                    minParallelLevel);
    lz4AioK->setInitialArguments({mFileSize});
    mPxDriver.makeKernelCall(
            lz4AioK,
            {
                    compressedByteStream,

                    blockInfo.isCompress,
                    blockInfo.blockStart,
                    blockInfo.blockEnd
            }, {
                    decompressionByteStream
            });

    return decompressionByteStream;
}

std::vector<StreamSetBuffer*> LZ4BaseGenerator::convertCompressedBitsStreamWithBitStreamAioApproach(
        parabix::StreamSetBuffer* compressedByteStream,
        std::vector<StreamSetBuffer*> compressedBitStreams
) {
    auto & b = mPxDriver.getBuilder();

    LZ4BlockInfo blockInfo = this->getBlockInfo(compressedByteStream);

    unsigned numOfStreams = compressedBitStreams[0]->getNumOfStreams();

    // 1, 2, 4, 8

    if (numOfStreams <= 2) {
        StreamSetBuffer* twistedCharClasses = mPxDriver.addBuffer<StaticBuffer>(b, b->getStreamSetTy(1, 2),
                                                                                   this->getDefaultBufferBlocks());
        kernel::Kernel* twistK = mPxDriver.addKernelInstance<kernel::TwistByPDEPKernel>(b, numOfStreams, 2);
        mPxDriver.makeKernelCall(twistK, {compressedBitStreams[0]}, {twistedCharClasses});

        StreamSetBuffer* uncompressedTwistedCharClasses = mPxDriver.addBuffer<StaticBuffer>(b, b->getStreamSetTy(1, 2),
                                                                                               this->getDefaultBufferBlocks());
        Kernel* lz4I4AioK = mPxDriver.addKernelInstance<LZ4TwistDecompressionKernel>(b, 2);
        lz4I4AioK->setInitialArguments({mFileSize});
        mPxDriver.makeKernelCall(lz4I4AioK, {
                compressedByteStream,

                blockInfo.isCompress,
                blockInfo.blockStart,
                blockInfo.blockEnd,

                twistedCharClasses
        }, {
                                            uncompressedTwistedCharClasses
                                    });

        StreamSetBuffer* untwistedCharClasses = mPxDriver.addBuffer<StaticBuffer>(b, b->getStreamSetTy(numOfStreams),
                                                                                     this->getDefaultBufferBlocks());
        kernel::Kernel* untwistK = mPxDriver.addKernelInstance<kernel::UntwistByPEXTKernel>(b, numOfStreams, 2);
        mPxDriver.makeKernelCall(untwistK, {uncompressedTwistedCharClasses}, {untwistedCharClasses});
        return {untwistedCharClasses};
    }

    if (numOfStreams <= 4) {
        StreamSetBuffer* twistedCharClasses = mPxDriver.addBuffer<StaticBuffer>(b, b->getStreamSetTy(1, 4),
                                                                                   this->getDefaultBufferBlocks());
        kernel::Kernel* twistK = mPxDriver.addKernelInstance<kernel::TwistByPDEPKernel>(b, numOfStreams, 4);
        mPxDriver.makeKernelCall(twistK, {compressedBitStreams[0]}, {twistedCharClasses});


        StreamSetBuffer* uncompressedTwistedCharClasses = mPxDriver.addBuffer<StaticBuffer>(b, b->getStreamSetTy(1, 4),
                                                                                               this->getDefaultBufferBlocks());

        Kernel* lz4I4AioK = mPxDriver.addKernelInstance<LZ4TwistDecompressionKernel>(b, 4);
        lz4I4AioK->setInitialArguments({mFileSize});
        mPxDriver.makeKernelCall(lz4I4AioK, {
                compressedByteStream,

                blockInfo.isCompress,
                blockInfo.blockStart,
                blockInfo.blockEnd,

                twistedCharClasses
        }, {
                                            uncompressedTwistedCharClasses
                                    });

        StreamSetBuffer* untwistedCharClasses = mPxDriver.addBuffer<StaticBuffer>(b, b->getStreamSetTy(numOfStreams),
                                                                                     this->getDefaultBufferBlocks());
        kernel::Kernel* untwistK = mPxDriver.addKernelInstance<kernel::UntwistByPEXTKernel>(b, numOfStreams, 4);
        mPxDriver.makeKernelCall(untwistK, {uncompressedTwistedCharClasses}, {untwistedCharClasses});
        return {untwistedCharClasses};
    }

    std::vector<StreamSetBuffer *> inputStreams = {
            compressedByteStream,

            blockInfo.isCompress,
            blockInfo.blockStart,
            blockInfo.blockEnd,
    };

    std::vector<StreamSetBuffer *> outputStream;
    std::vector<unsigned> numbersOfStreams;

    for (unsigned i = 0; i < compressedBitStreams.size(); i++) {
        unsigned numOfStreams = compressedBitStreams[i]->getNumOfStreams();
        numbersOfStreams.push_back(numOfStreams);
        inputStreams.push_back(compressedBitStreams[i]);
        outputStream.push_back(mPxDriver.addBuffer<StaticBuffer>(b, b->getStreamSetTy(numOfStreams, 1),
                                                                 this->getDefaultBufferBlocks()));
    }

    Kernel* lz4AioK = mPxDriver.addKernelInstance<LZ4BitStreamDecompressionKernel>(b, numbersOfStreams);
    lz4AioK->setInitialArguments({mFileSize});
    mPxDriver.makeKernelCall(lz4AioK, inputStreams, outputStream);

    return outputStream;
}

unsigned LZ4BaseGenerator::getBlockSizeBufferBlocks() {
    return mLz4BlockSize / codegen::BlockSize;
}

unsigned LZ4BaseGenerator::getDefaultBufferBlocks() {
    return this->getBlockSizeBufferBlocks() * 2; // buffer 2 LZ4 Block By Default
}