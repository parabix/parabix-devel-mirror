
#include <numeric>
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
    return this->decompressBitStreams(compressedByteStream, {compressedBitStream})[0];
}

unsigned LZ4GrepByteStreamGenerator::calculateTwistWidth(unsigned numOfStreams) {
    if (numOfStreams <= 2) {
        return numOfStreams;
    } else if (numOfStreams <= 4) {
        return 4;
    } else if (numOfStreams <= 8) {
        return 8;
    } else {
        llvm::report_fatal_error("Twist: Unsupported numOfStreams " + std::to_string(numOfStreams));;
    }
}

std::vector<parabix::StreamSetBuffer *>
LZ4GrepByteStreamGenerator::decompressBitStreams(parabix::StreamSetBuffer *compressedByteStream,
                                                 std::vector<parabix::StreamSetBuffer *> compressedBitStreams) {
    auto & b = mPxDriver.getBuilder();

    std::vector<unsigned> numOfStreams(compressedBitStreams.size());
    std::transform(compressedBitStreams.begin(), compressedBitStreams.end(), numOfStreams.begin(), [](StreamSetBuffer* b){return b->getNumOfStreams();});
    unsigned totalStreamNum = std::accumulate(numOfStreams.begin(), numOfStreams.end(), 0u);

    unsigned twistWidth = this->calculateTwistWidth(totalStreamNum);
    StreamSetBuffer* twistedStream = this->twist(b, compressedBitStreams, twistWidth);

    LZ4BlockInfo blockInfo = this->getBlockInfo(compressedByteStream);
    StreamSetBuffer* uncompressedTwistedStream = mPxDriver.addBuffer<StaticBuffer>(b, b->getStreamSetTy(1, twistWidth), this->getDefaultBufferBlocks(), 1);
    std::vector<StreamSetBuffer*> inputStreams = {
            compressedByteStream,

            blockInfo.isCompress,
            blockInfo.blockStart,
            blockInfo.blockEnd,

            twistedStream
    };
    std::vector<StreamSetBuffer*> outputStreams = {
            uncompressedTwistedStream
    };

    if (twistWidth <= 4) {
        Kernel* lz4I4AioK = mPxDriver.addKernelInstance<LZ4TwistDecompressionKernel>(b, twistWidth);
        lz4I4AioK->setInitialArguments({mFileSize});
        mPxDriver.makeKernelCall(lz4I4AioK, inputStreams, outputStreams);

    } else {
        Kernel* lz4AioK = mPxDriver.addKernelInstance<LZ4ByteStreamDecompressionKernel>(b, true);
        lz4AioK->setInitialArguments({mFileSize});
        mPxDriver.makeKernelCall(lz4AioK, inputStreams, outputStreams);
    }
    return this->untwist(b, uncompressedTwistedStream, twistWidth, numOfStreams);
}

parabix::StreamSetBuffer* LZ4GrepByteStreamGenerator::twist(const std::unique_ptr<kernel::KernelBuilder> &b,
                                                            std::vector<StreamSetBuffer*> inputStreams,
                                                            unsigned twistWidth
) {
    std::vector<unsigned> numsOfStreams(inputStreams.size());
    std::transform(inputStreams.begin(), inputStreams.end(), numsOfStreams.begin(), [](StreamSetBuffer* b){return b->getNumOfStreams();});
    unsigned totalNumOfStreams = std::accumulate(numsOfStreams.begin(), numsOfStreams.end(), 0u);
    assert(totalNumOfStreams <= twistWidth);

    if (twistWidth == 1) {
        for (unsigned i = 0; i < inputStreams.size(); i++) {
            if (inputStreams[i]->getNumOfStreams() == 1) {
                return inputStreams[i];
            }
        }
    } else if (twistWidth == 2 || twistWidth == 4) {
        StreamSetBuffer* twistedCharClasses = mPxDriver.addBuffer<StaticBuffer>(b, b->getStreamSetTy(1, twistWidth),
                                                                                this->getDefaultBufferBlocks(), 1);
        kernel::Kernel* twistK = mPxDriver.addKernelInstance<kernel::TwistMultipleByPDEPKernel>(b, numsOfStreams, twistWidth);
        mPxDriver.makeKernelCall(twistK, inputStreams, {twistedCharClasses});
        return twistedCharClasses;
    } else if (twistWidth == 8) {
        StreamSetBuffer * const mtxByteStream = mPxDriver.addBuffer<StaticBuffer>(b, b->getStreamSetTy(1, twistWidth),
                                                                                  this->getDefaultBufferBlocks());
        Kernel * p2sK = mPxDriver.addKernelInstance<P2SMultipleStreamsKernel>(b, cc::BitNumbering::BigEndian, numsOfStreams);
        mPxDriver.makeKernelCall(p2sK, inputStreams, {mtxByteStream});
        return mtxByteStream;
    } else {
        llvm::report_fatal_error("Twist: Unsupported twistWidth " + std::to_string(twistWidth));;
    }
}

std::vector<StreamSetBuffer*> LZ4GrepByteStreamGenerator::untwist(const std::unique_ptr<kernel::KernelBuilder> &b,
                                                              parabix::StreamSetBuffer *inputStream,
                                                              unsigned twistWidth,
                                                              std::vector<unsigned> numOfStreams
) {
    unsigned totalNumOfStreams = std::accumulate(numOfStreams.begin(), numOfStreams.end(), 0u);
    assert(totalNumOfStreams <= twistWidth);
    if (twistWidth == 1) {
        std::vector<unsigned> fakeStreamNums;
        for (unsigned i = 0; i < numOfStreams.size(); i++) {
            if (numOfStreams[i] == 0) {
                fakeStreamNums.push_back(0);
            }
        }
        auto fakeStreams = this->generateFakeStreams(b, inputStream, fakeStreamNums);

        std::vector<StreamSetBuffer*> retBuffers;
        unsigned j = 0;
        for (unsigned i = 0; i < numOfStreams.size(); i++) {
            if (numOfStreams[i] == 0) {
                retBuffers.push_back(fakeStreams[j]);
                j++;
            } else {
                retBuffers.push_back(inputStream);
            }
        }
        return retBuffers;
    } else{
        std::vector<StreamSetBuffer*> retBuffers;
        for (unsigned i = 0; i < numOfStreams.size(); i++) {
            retBuffers.push_back(mPxDriver.addBuffer<StaticBuffer>(b, b->getStreamSetTy(numOfStreams[i]), this->getDefaultBufferBlocks(), 1));
        }


        if (twistWidth == 2 || twistWidth == 4) {
            kernel::Kernel* untwistK = mPxDriver.addKernelInstance<kernel::UntwistMultipleByPEXTKernel>(b, numOfStreams, twistWidth);
            mPxDriver.makeKernelCall(untwistK, {inputStream}, retBuffers);
            return retBuffers;
        } else if (twistWidth == 8) {
            Kernel * s2pk = mPxDriver.addKernelInstance<S2PMultipleStreamsKernel>(b, cc::BitNumbering::BigEndian, true, numOfStreams);
            mPxDriver.makeKernelCall(s2pk, {inputStream}, retBuffers);
            return retBuffers;
        } else {
            llvm::report_fatal_error("Twist: Unsupported twistWidth " + std::to_string(twistWidth));;
        }
    }
}


