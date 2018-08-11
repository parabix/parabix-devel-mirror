
#include "lz4_grep_bytestream_generator.h"
#include <kernels/lz4/twist_kernel.h>
#include <kernels/lz4/decompression/lz4_twist_decompression.h>
#include <kernels/lz4/untwist_kernel.h>
#include <kernels/s2p_kernel.h>
#include <kernels/p2s_kernel.h>
#include <kernels/lz4/decompression/lz4_bytestream_decompression.h>
#include <kernels/kernel_builder.h>


using namespace kernel;
using namespace parabix;

StreamSetBuffer *LZ4GrepByteStreamGenerator::generateUncompressedBitStreams() {
    StreamSetBuffer* compressedByteStream = this->loadByteStream();
    parabix::StreamSetBuffer * uncompressedByteStream = this->byteStreamDecompression(compressedByteStream);
    return this->s2p(uncompressedByteStream);
}

parabix::StreamSetBuffer *
LZ4GrepByteStreamGenerator::decompressBitStream(parabix::StreamSetBuffer *compressedByteStream,
                                                parabix::StreamSetBuffer *compressedBitStream) {
    return this->convertCompressedBitsStreamWithTwistApproach(compressedByteStream, compressedBitStream, "combined");
}


parabix::StreamSetBuffer * LZ4GrepByteStreamGenerator::convertCompressedBitsStreamWithTwistApproach(
        parabix::StreamSetBuffer *compressedByteStream,
        parabix::StreamSetBuffer *compressedBitStream,
        std::string prefix
) {
    auto & b = mPxDriver.getBuilder();

    LZ4BlockInfo blockInfo = this->getBlockInfo(compressedByteStream);

    unsigned numOfStreams = compressedBitStream->getNumOfStreams();

    if (numOfStreams == 1) {

        StreamSetBuffer* uncompressedBitStream = mPxDriver.addBuffer<StaticBuffer>(b, b->getStreamSetTy(1, 1),
                                                                                   this->getDefaultBufferBlocks(), 1);
        Kernel* lz4I4AioK = mPxDriver.addKernelInstance<LZ4TwistDecompressionKernel>(b, 1);
        lz4I4AioK->setInitialArguments({mFileSize});
        mPxDriver.makeKernelCall(lz4I4AioK, {
                compressedByteStream,

                blockInfo.isCompress,
                blockInfo.blockStart,
                blockInfo.blockEnd,

                compressedBitStream
//                , uncompressedBitStream2
        }, {
                                         uncompressedBitStream
                                 });


//        Kernel* streamCmp = mPxDriver.addKernelInstance<StreamCompareKernel>(b, 1);
//        mPxDriver.makeKernelCall(streamCmp, {
//                uncompressedBitStream,
//                uncompressedBitStream2
//        }, {});

        return uncompressedBitStream;
    }
    if (numOfStreams <= 2) {
        StreamSetBuffer* twistedCharClasses = mPxDriver.addBuffer<StaticBuffer>(b, b->getStreamSetTy(1, 2),
                                                                                this->getDefaultBufferBlocks(), 1);
        kernel::Kernel* twistK = mPxDriver.addKernelInstance<kernel::TwistMultipleByPDEPKernel>(b, std::vector<unsigned>{numOfStreams}, 2);
        mPxDriver.makeKernelCall(twistK, {compressedBitStream}, {twistedCharClasses});


        StreamSetBuffer* uncompressedTwistedCharClasses = mPxDriver.addBuffer<StaticBuffer>(b, b->getStreamSetTy(1, 2),
                                                                                            this->getDefaultBufferBlocks(), 1);
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
                                                                                  this->getDefaultBufferBlocks(), 1);
        kernel::Kernel* untwistK = mPxDriver.addKernelInstance<kernel::UntwistMultipleByPEXTKernel>(b, std::vector<unsigned>{numOfStreams}, 2);
        mPxDriver.makeKernelCall(untwistK, {uncompressedTwistedCharClasses}, {untwistedCharClasses});
        return untwistedCharClasses;
    }
    if (numOfStreams <= 4) {
        StreamSetBuffer* twistedCharClasses = mPxDriver.addBuffer<StaticBuffer>(b, b->getStreamSetTy(1, 4),
                                                                                this->getDefaultBufferBlocks(), 1);
        kernel::Kernel* twistK = mPxDriver.addKernelInstance<kernel::TwistMultipleByPDEPKernel>(b, std::vector<unsigned>{numOfStreams}, 4);
        mPxDriver.makeKernelCall(twistK, {compressedBitStream}, {twistedCharClasses});


        StreamSetBuffer* uncompressedTwistedCharClasses = mPxDriver.addBuffer<StaticBuffer>(b, b->getStreamSetTy(1, 4),
                                                                                            this->getDefaultBufferBlocks(), 1);

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
                                                                                  this->getDefaultBufferBlocks(), 1);
        kernel::Kernel* untwistK = mPxDriver.addKernelInstance<kernel::UntwistMultipleByPEXTKernel>(b, std::vector<unsigned>{numOfStreams}, 4);
        mPxDriver.makeKernelCall(untwistK, {uncompressedTwistedCharClasses}, {untwistedCharClasses});
        return untwistedCharClasses;
    }

    // <= 8
    StreamSetBuffer * const mtxByteStream = mPxDriver.addBuffer<StaticBuffer>(b, b->getStreamSetTy(1, 8),
                                                                              this->getDefaultBufferBlocks());
    Kernel * p2sK = mPxDriver.addKernelInstance<P2SKernel>(b, cc::BitNumbering::BigEndian, prefix, numOfStreams);
    mPxDriver.makeKernelCall(p2sK, {compressedBitStream}, {mtxByteStream});


    StreamSetBuffer * const decompressionMtxByteStream = mPxDriver.addBuffer<StaticBuffer>(b, b->getStreamSetTy(1, 8),
                                                                                           this->getDefaultBufferBlocks(), 1);
    Kernel* lz4AioK = mPxDriver.addKernelInstance<LZ4ByteStreamDecompressionKernel>(b, true);
    lz4AioK->setInitialArguments({mFileSize});
    mPxDriver.makeKernelCall(
            lz4AioK,
            {
                    compressedByteStream,
                    // Block Data
                    blockInfo.isCompress,
                    blockInfo.blockStart,
                    blockInfo.blockEnd,
                    mtxByteStream
            }, {
                    decompressionMtxByteStream
            });

    StreamSetBuffer * const uncompressedMtxBitStream = mPxDriver.addBuffer<StaticBuffer>(b, b->getStreamSetTy(numOfStreams),
                                                                                         this->getDefaultBufferBlocks());

    Kernel * s2pk = mPxDriver.addKernelInstance<S2PKernel>(b, cc::BitNumbering::BigEndian, true, prefix, numOfStreams);
    mPxDriver.makeKernelCall(s2pk, {decompressionMtxByteStream}, {uncompressedMtxBitStream});
    return uncompressedMtxBitStream;
}

std::vector<parabix::StreamSetBuffer *>
LZ4GrepByteStreamGenerator::decompressBitStreams(parabix::StreamSetBuffer *compressedByteStream,
                                                 std::vector<parabix::StreamSetBuffer *> compressedBitStreams) {


    auto & b = mPxDriver.getBuilder();
    LZ4BlockInfo blockInfo = this->getBlockInfo(compressedByteStream);

    unsigned totalStreamNum = 0;
    std::vector<unsigned> numOfStreams;
    std::vector<StreamSetBuffer*> retStreams;
    for (unsigned i = 0; i < compressedBitStreams.size(); i++) {
        unsigned n = compressedBitStreams[i]->getNumOfStreams();

        numOfStreams.push_back(n);
        totalStreamNum += n;
        retStreams.push_back(mPxDriver.addBuffer<StaticBuffer>(b, b->getStreamSetTy(n), this->getDefaultBufferBlocks(), 1));
    }

    if (totalStreamNum == 1) {
        // TODO
    } else if (totalStreamNum <= 4) {
        unsigned twistWidth = totalStreamNum == 2 ? 2 : 4;

        StreamSetBuffer* twistedStream = mPxDriver.addBuffer<StaticBuffer>(b, b->getStreamSetTy(1, twistWidth),
                                                                                this->getDefaultBufferBlocks(), 1);
        kernel::Kernel* twistK = mPxDriver.addKernelInstance<kernel::TwistMultipleByPDEPKernel>(b, numOfStreams, twistWidth);
        mPxDriver.makeKernelCall(twistK, compressedBitStreams, {twistedStream});

        StreamSetBuffer* uncompressedTwistedStream = mPxDriver.addBuffer<StaticBuffer>(b, b->getStreamSetTy(1, twistWidth),
                                                                                            this->getDefaultBufferBlocks(), 1);
        Kernel* lz4I4AioK = mPxDriver.addKernelInstance<LZ4TwistDecompressionKernel>(b, twistWidth);
        lz4I4AioK->setInitialArguments({mFileSize});
        mPxDriver.makeKernelCall(lz4I4AioK, {
                compressedByteStream,

                blockInfo.isCompress,
                blockInfo.blockStart,
                blockInfo.blockEnd,

                twistedStream
        }, {
                uncompressedTwistedStream
                                 });

        kernel::Kernel* untwistK = mPxDriver.addKernelInstance<kernel::UntwistMultipleByPEXTKernel>(b, numOfStreams, twistWidth);
        mPxDriver.makeKernelCall(untwistK, {uncompressedTwistedStream}, retStreams);
    } else {
        // TODO
    }
    return retStreams;
}
