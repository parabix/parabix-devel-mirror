
#include <numeric>
#include "lz4_grep_bytestream_generator.h"
#include <kernels/lz4/twist_kernel.h>
#include <kernels/lz4/decompression/lz4_twist_decompression.h>
#include <kernels/lz4/untwist_kernel.h>
#include <kernels/s2p_kernel.h>
#include <kernels/p2s_kernel.h>
#include <kernels/lz4/decompression/lz4_bytestream_decompression.h>
#include <kernels/kernel_builder.h>
#include <kernels/pipeline_builder.h>

using namespace kernel;

StreamSet *LZ4GrepByteStreamGenerator::generateUncompressedByteStream() {
    StreamSet* compressedByteStream = loadByteStream();
    StreamSet * uncompressedByteStream = byteStreamDecompression(compressedByteStream);
    return uncompressedByteStream;
}
StreamSet *LZ4GrepByteStreamGenerator::generateUncompressedBitStreams() {
    StreamSet * uncompressedByteStream = generateUncompressedByteStream();
    return s2p(uncompressedByteStream);
}

StreamSet * LZ4GrepByteStreamGenerator::decompressBitStream(StreamSet *compressedByteStream, StreamSet *compressedBitStream) {
    return decompressBitStreams(compressedByteStream, {compressedBitStream})[0];
}

StreamSets LZ4GrepByteStreamGenerator::decompressBitStreams(StreamSet * compressedByteStream, StreamSets compressedBitStreams) {

    std::vector<unsigned> numOfStreams(compressedBitStreams.size());
    std::transform(compressedBitStreams.begin(), compressedBitStreams.end(), numOfStreams.begin(),
                   [](StreamSet* b){
                        return b->getNumElements();
                   });

    const auto totalStreamNum = std::accumulate(numOfStreams.begin(), numOfStreams.end(), 0u);

    const auto twistWidth = calculateTwistWidth(totalStreamNum);
    StreamSet * twistedStream = twist(compressedBitStreams, twistWidth);

    LZ4BlockInfo blockInfo = getBlockInfo(compressedByteStream);

    StreamSet * const uncompressedTwistedStream = mPipeline->CreateStreamSet(1, twistWidth);

    if (twistWidth <= 4) {    
        mPipeline->CreateKernelCall<LZ4TwistDecompressionKernel>(mFileSize, compressedByteStream, blockInfo, twistedStream, uncompressedTwistedStream);
    } else {
        mPipeline->CreateKernelCall<LZ4ByteStreamDecompressionKernel>(mFileSize, compressedByteStream, blockInfo, twistedStream, uncompressedTwistedStream);
    }

    return untwist(uncompressedTwistedStream, twistWidth, numOfStreams);
}

StreamSet * LZ4GrepByteStreamGenerator::twist(const StreamSets & inputStreams, const unsigned twistWidth) {

    if (twistWidth == 1) {
        for (unsigned i = 0; i < inputStreams.size(); i++) {
            if (inputStreams[i]->getNumElements() == 1) {
                return inputStreams[i];
            }
        }
        llvm_unreachable("did not find input stream");
    }

    StreamSet * const twistedOutput = mPipeline->CreateStreamSet(1, twistWidth);
    if (twistWidth == 2 || twistWidth == 4) {
        mPipeline->CreateKernelCall<TwistMultipleByPDEPKernel>(inputStreams, twistedOutput);
    } else if (twistWidth == 8) {
        mPipeline->CreateKernelCall<P2SMultipleStreamsKernel>(inputStreams, twistedOutput, cc::BitNumbering::BigEndian);
    } else {
        llvm::report_fatal_error("Twist: Unsupported twistWidth " + std::to_string(twistWidth));
    }
    return twistedOutput;
}

StreamSets LZ4GrepByteStreamGenerator::untwist(StreamSet * inputStream, const unsigned twistWidth, const std::vector<unsigned> & numOfStreams) {
    StreamSets retBuffers;
    if (twistWidth == 1) {
        std::vector<unsigned> fakeStreamNums;
        for (unsigned i = 0; i < numOfStreams.size(); i++) {
            if (numOfStreams[i] == 0) {
                fakeStreamNums.push_back(0);
            }
        }
        auto fakeStreams = generateFakeStreams(inputStream, fakeStreamNums);
        unsigned j = 0;
        for (unsigned i = 0; i < numOfStreams.size(); i++) {
            if (numOfStreams[i] == 0) {
                retBuffers.push_back(fakeStreams[j++]);
            } else {
                retBuffers.push_back(inputStream);
            }
        }
    } else{        
        for (unsigned i = 0; i < numOfStreams.size(); i++) {
            retBuffers.push_back(mPipeline->CreateStreamSet(numOfStreams[i]));
        }
        if (twistWidth == 2 || twistWidth == 4) {
            mPipeline->CreateKernelCall<UntwistMultipleByPEXTKernel>(inputStream, retBuffers);
        } else if (twistWidth == 8) {
            mPipeline->CreateKernelCall<S2PMultipleStreamsKernel>(inputStream, retBuffers, cc::BitNumbering::BigEndian, true);
        } else {
            llvm::report_fatal_error("Twist: Unsupported twistWidth " + std::to_string(twistWidth));;
        }
    }
    return retBuffers;
}


