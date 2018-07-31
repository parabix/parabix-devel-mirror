
#include "LZ4Generator.h"

#include <boost/filesystem.hpp>
#include <boost/iostreams/device/mapped_file.hpp>


#include <cc/cc_compiler.h>

#include <kernels/cc_kernel.h>
#include <kernels/s2p_kernel.h>
#include <kernels/p2s_kernel.h>
#include <kernels/source_kernel.h>
#include <kernels/stdout_kernel.h>
#include <kernels/kernel_builder.h>
#include <kernels/deletion.h>
#include <kernels/swizzle.h>
#include <kernels/pdep_kernel.h>
#include <kernels/swizzled_multiple_pdep_kernel.h>
#include <kernels/lz4/lz4_block_decoder.h>
#include <kernels/lz4/decompression/lz4_bytestream_decompression.h>
#include <kernels/lz4/decompression/lz4_parallel_bytestream_decompression.h>
#include <kernels/lz4/decompression/lz4_swizzled_decompression.h>
#include <kernels/lz4/decompression/lz4_bitstream_decompression.h>
#include <kernels/lz4/decompression/lz4_twist_decompression.h>
#include <kernels/bitstream_pdep_kernel.h>
#include <kernels/lz4/twist_kernel.h>
#include <kernels/lz4/untwist_kernel.h>

namespace re { class CC; }

using namespace llvm;
using namespace parabix;
using namespace kernel;

LZ4Generator::LZ4Generator():mPxDriver("lz4d"), mLz4BlockSize(4 * 1024 * 1024) {
}

MainFunctionType LZ4Generator::getMainFunc() {
    return reinterpret_cast<MainFunctionType>(mPxDriver.getMain());
}

void LZ4Generator::generateDecompressionPipeline(const std::string &outputFile) {
    auto & b = mPxDriver.getBuilder();

    this->generateMainFunc(b);
    this->generateLoadByteStreamAndBitStream(b);
    parabix::StreamSetBuffer* uncompressedByteStream = this->generateAIODecompression(b);

    Kernel * outK = mPxDriver.addKernelInstance<FileSink>(b, 8);
    outK->setInitialArguments({b->GetString(outputFile)});
    mPxDriver.makeKernelCall(outK, {uncompressedByteStream}, {});

    mPxDriver.generatePipelineIR();
    mPxDriver.deallocateBuffers();

    b->CreateRetVoid();

    mPxDriver.finalizeObject();
}


void LZ4Generator::generateMainFunc(const std::unique_ptr<kernel::KernelBuilder> & iBuilder) {
    Module * M = iBuilder->getModule();
    Type * const sizeTy = iBuilder->getSizeTy();
    Type * const boolTy = iBuilder->getIntNTy(sizeof(bool) * 8);
    Type * const voidTy = iBuilder->getVoidTy();
    Type * const inputType = iBuilder->getInt8PtrTy();

    Function * const main = cast<Function>(M->getOrInsertFunction("Main", voidTy, inputType, sizeTy, sizeTy, boolTy, nullptr));
    main->setCallingConv(CallingConv::C);
    Function::arg_iterator args = main->arg_begin();
    mInputStream = &*(args++);
    mInputStream->setName("input");

    mHeaderSize = &*(args++);
    mHeaderSize->setName("mHeaderSize");

    mFileSize = &*(args++);
    mFileSize->setName("mFileSize");

    mHasBlockChecksum = &*(args++);
    mHasBlockChecksum->setName("mHasBlockChecksum");
    // TODO for now, we do not handle blockCheckSum
    mHasBlockChecksum = iBuilder->getInt1(false);

    iBuilder->SetInsertPoint(BasicBlock::Create(M->getContext(), "entry", main, 0));
}

void LZ4Generator::generateLoadByteStream(const std::unique_ptr<kernel::KernelBuilder> & iBuilder) {
    mCompressedByteStream = mPxDriver.addBuffer<ExternalBuffer>(iBuilder, iBuilder->getStreamSetTy(1, 8));
    kernel::Kernel * sourceK = mPxDriver.addKernelInstance<MemorySourceKernel>(iBuilder);
    sourceK->setInitialArguments({mInputStream, mFileSize});
    mPxDriver.makeKernelCall(sourceK, {}, {mCompressedByteStream});
}
void LZ4Generator::generateLoadByteStreamAndBitStream(const std::unique_ptr<kernel::KernelBuilder> & iBuilder) {
    mCompressedByteStream = mPxDriver.addBuffer<ExternalBuffer>(iBuilder, iBuilder->getStreamSetTy(1, 8));
    mCompressedBasisBits = mPxDriver.addBuffer<StaticBuffer>(iBuilder, iBuilder->getStreamSetTy(8, 1), this->getInputBufferBlocks(iBuilder));

    kernel::Kernel * sourceK = mPxDriver.addKernelInstance<MemorySourceKernel>(iBuilder);
    sourceK->setInitialArguments({mInputStream, mFileSize});
    mPxDriver.makeKernelCall(sourceK, {}, {mCompressedByteStream});
    Kernel * s2pk = mPxDriver.addKernelInstance<S2PKernel>(iBuilder, cc::BitNumbering::BigEndian);
    mPxDriver.makeKernelCall(s2pk, {mCompressedByteStream}, {mCompressedBasisBits});
}

StreamSetBuffer * LZ4Generator::generateBitStreamAIODecompression(const std::unique_ptr<kernel::KernelBuilder> & iBuilder) {
    return this->convertCompressedBitsStreamWithBitStreamAioApproach({mCompressedBasisBits}, "combined")[0];
}

std::vector<StreamSetBuffer*> LZ4Generator::convertCompressedBitsStreamWithBitStreamAioApproach(
        std::vector<StreamSetBuffer*> compressedBitStreams, std::string prefix) {
    auto mGrepDriver = &mPxDriver;
    auto & iBuilder = mGrepDriver->getBuilder();

    LZ4BlockInfo blockInfo = this->getBlockInfo(iBuilder);

    size_t numOfStreams = compressedBitStreams[0]->getNumOfStreams();

    // 1, 2, 4, 8

    if (numOfStreams <= 2) {
        StreamSetBuffer* twistedCharClasses = mGrepDriver->addBuffer<StaticBuffer>(iBuilder, iBuilder->getStreamSetTy(1, 2), this->getInputBufferBlocks(iBuilder));
        kernel::Kernel* twistK = mGrepDriver->addKernelInstance<kernel::TwistByPDEPKernel>(iBuilder, numOfStreams, 2);
        mGrepDriver->makeKernelCall(twistK, {compressedBitStreams[0]}, {twistedCharClasses});

        StreamSetBuffer* uncompressedTwistedCharClasses = mGrepDriver->addBuffer<StaticBuffer>(iBuilder, iBuilder->getStreamSetTy(1, 2), this->getInputBufferBlocks(iBuilder));
        Kernel* lz4I4AioK = mPxDriver.addKernelInstance<LZ4TwistDecompressionKernel>(iBuilder, 2);
        lz4I4AioK->setInitialArguments({mFileSize});
        mGrepDriver->makeKernelCall(lz4I4AioK, {
                mCompressedByteStream,

                blockInfo.isCompress,
                blockInfo.blockStart,
                blockInfo.blockEnd,

                twistedCharClasses
        }, {
                                            uncompressedTwistedCharClasses
                                    });

        StreamSetBuffer* untwistedCharClasses = mGrepDriver->addBuffer<StaticBuffer>(iBuilder, iBuilder->getStreamSetTy(numOfStreams), this->getInputBufferBlocks(iBuilder));
        kernel::Kernel* untwistK = mGrepDriver->addKernelInstance<kernel::UntwistByPEXTKernel>(iBuilder, numOfStreams, 2);
        mGrepDriver->makeKernelCall(untwistK, {uncompressedTwistedCharClasses}, {untwistedCharClasses});
        return {untwistedCharClasses};
    }

    if (numOfStreams <= 4) {
        StreamSetBuffer* twistedCharClasses = mGrepDriver->addBuffer<StaticBuffer>(iBuilder, iBuilder->getStreamSetTy(1, 4), this->getInputBufferBlocks(iBuilder));
        kernel::Kernel* twistK = mGrepDriver->addKernelInstance<kernel::TwistByPDEPKernel>(iBuilder, numOfStreams, 4);
        mGrepDriver->makeKernelCall(twistK, {compressedBitStreams[0]}, {twistedCharClasses});


        StreamSetBuffer* uncompressedTwistedCharClasses = mGrepDriver->addBuffer<StaticBuffer>(iBuilder, iBuilder->getStreamSetTy(1, 4), this->getInputBufferBlocks(iBuilder));

        Kernel* lz4I4AioK = mPxDriver.addKernelInstance<LZ4TwistDecompressionKernel>(iBuilder, 4);
        lz4I4AioK->setInitialArguments({mFileSize});
        mGrepDriver->makeKernelCall(lz4I4AioK, {
                mCompressedByteStream,

                blockInfo.isCompress,
                blockInfo.blockStart,
                blockInfo.blockEnd,

                twistedCharClasses
        }, {
                                            uncompressedTwistedCharClasses
                                    });

        StreamSetBuffer* untwistedCharClasses = mGrepDriver->addBuffer<StaticBuffer>(iBuilder, iBuilder->getStreamSetTy(numOfStreams), this->getInputBufferBlocks(iBuilder));
        kernel::Kernel* untwistK = mGrepDriver->addKernelInstance<kernel::UntwistByPEXTKernel>(iBuilder, numOfStreams, 4);
        mGrepDriver->makeKernelCall(untwistK, {uncompressedTwistedCharClasses}, {untwistedCharClasses});
        return {untwistedCharClasses};
    }

    std::vector<StreamSetBuffer *> inputStreams = {
            mCompressedByteStream,

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
        outputStream.push_back(mPxDriver.addBuffer<StaticBuffer>(iBuilder, iBuilder->getStreamSetTy(numOfStreams, 1), this->getInputBufferBlocks(iBuilder)));
    }

    Kernel* lz4AioK = mPxDriver.addKernelInstance<LZ4BitStreamDecompressionKernel>(iBuilder, numbersOfStreams);
    lz4AioK->setInitialArguments({mFileSize});
    mPxDriver.makeKernelCall(lz4AioK, inputStreams, outputStream);

    return outputStream;
}


StreamSetBuffer * LZ4Generator::generateSwizzledAIODecompression(const std::unique_ptr<kernel::KernelBuilder> & iBuilder) {
    LZ4BlockInfo blockInfo = this->getBlockInfo(iBuilder);

    // Produce unswizzled bit streams
    StreamSetBuffer * u16Swizzle0 = mPxDriver.addBuffer<StaticBuffer>(iBuilder, iBuilder->getStreamSetTy(4), this->getInputBufferBlocks(iBuilder), 1);
    StreamSetBuffer * u16Swizzle1 = mPxDriver.addBuffer<StaticBuffer>(iBuilder, iBuilder->getStreamSetTy(4), this->getInputBufferBlocks(iBuilder), 1);
    Kernel * unSwizzleK = mPxDriver.addKernelInstance<SwizzleGenerator>(iBuilder, 8, 2, 1, 64, "source");
    mPxDriver.makeKernelCall(unSwizzleK, {mCompressedBasisBits}, {u16Swizzle0, u16Swizzle1});



    StreamSetBuffer * decompressedSwizzled0 = mPxDriver.addBuffer<StaticBuffer>(iBuilder, iBuilder->getStreamSetTy(4), this->getInputBufferBlocks(iBuilder), 1);
    StreamSetBuffer * decompressedSwizzled1 = mPxDriver.addBuffer<StaticBuffer>(iBuilder, iBuilder->getStreamSetTy(4), this->getInputBufferBlocks(iBuilder), 1);


    Kernel* lz4AioK = mPxDriver.addKernelInstance<LZ4SwizzledDecompressionKernel>(iBuilder, 4, 2, 4);
    lz4AioK->setInitialArguments({mFileSize});
    mPxDriver.makeKernelCall(
            lz4AioK,
            {
                    mCompressedByteStream,

                    blockInfo.isCompress,
                    blockInfo.blockStart,
                    blockInfo.blockEnd,

                    u16Swizzle0,
                    u16Swizzle1
            }, {
                    decompressedSwizzled0,
                    decompressedSwizzled1
            });


    StreamSetBuffer * const decompressionBitStream = mPxDriver.addBuffer<StaticBuffer>(iBuilder, iBuilder->getStreamSetTy(8, 1), this->getDecompressedBufferBlocks(iBuilder));

    Kernel * unSwizzleK2 = mPxDriver.addKernelInstance<SwizzleGenerator>(iBuilder, 8, 1, 2);
    mPxDriver.makeKernelCall(unSwizzleK2, {decompressedSwizzled0, decompressedSwizzled1}, {decompressionBitStream});

    return decompressionBitStream;
}

parabix::StreamSetBuffer * LZ4Generator::generateParallelAIODecompression(const std::unique_ptr<kernel::KernelBuilder> & iBuilder, bool enableGather, bool enableScatter, int minParallelLevel) {
    LZ4BlockInfo blockInfo = this->getBlockInfo(iBuilder);
    StreamSetBuffer * const decompressionByteStream = mPxDriver.addBuffer<StaticBuffer>(iBuilder, iBuilder->getStreamSetTy(1, 8), this->getDecompressedBufferBlocks(iBuilder), 1);

    Kernel* lz4AioK = mPxDriver.addKernelInstance<LZ4ParallelByteStreamDecompressionKernel>(iBuilder, mLz4BlockSize, enableGather, enableScatter, minParallelLevel);
    lz4AioK->setInitialArguments({mFileSize});
    mPxDriver.makeKernelCall(
            lz4AioK,
            {
                    mCompressedByteStream,

                    blockInfo.isCompress,
                    blockInfo.blockStart,
                    blockInfo.blockEnd
            }, {
                    decompressionByteStream
            });

    return decompressionByteStream;

}

StreamSetBuffer * LZ4Generator::generateAIODecompression(const std::unique_ptr<kernel::KernelBuilder> & iBuilder) {
    LZ4BlockInfo blockInfo = this->getBlockInfo(iBuilder);

    StreamSetBuffer * const decompressionByteStream = mPxDriver.addBuffer<StaticBuffer>(iBuilder, iBuilder->getStreamSetTy(1, 8), this->getDecompressedBufferBlocks(iBuilder), 1);
    Kernel* lz4AioK = mPxDriver.addKernelInstance<LZ4ByteStreamDecompressionKernel>(iBuilder);
    lz4AioK->setInitialArguments({mFileSize});
    mPxDriver.makeKernelCall(
            lz4AioK,
            {
                    mCompressedByteStream,

                    // Block Data
                    blockInfo.isCompress,
                    blockInfo.blockStart,
                    blockInfo.blockEnd
            }, {
                    decompressionByteStream
            });

    return decompressionByteStream;
}

int LZ4Generator::get4MbBufferBlocks() {
    return mLz4BlockSize / codegen::BlockSize;
}

int LZ4Generator::getInputBufferBlocks(const std::unique_ptr<kernel::KernelBuilder> & b) {
    return this->get4MbBufferBlocks() * 2;
}
int LZ4Generator::getDecompressedBufferBlocks(const std::unique_ptr<kernel::KernelBuilder> & b) {
    return this->get4MbBufferBlocks() * 2;
}

LZ4BlockInfo LZ4Generator::getBlockInfo(const std::unique_ptr<kernel::KernelBuilder> & b) {
    LZ4BlockInfo blockInfo;
    blockInfo.isCompress = mPxDriver.addBuffer<StaticBuffer>(b, b->getStreamSetTy(1, 8), this->getInputBufferBlocks(b), 1);
    blockInfo.blockStart = mPxDriver.addBuffer<StaticBuffer>(b, b->getStreamSetTy(1, 64), this->getInputBufferBlocks(b), 1);
    blockInfo.blockEnd = mPxDriver.addBuffer<StaticBuffer>(b, b->getStreamSetTy(1, 64), this->getInputBufferBlocks(b), 1);

    Kernel * blockDecoderK = mPxDriver.addKernelInstance<LZ4BlockDecoderKernel>(b);
    blockDecoderK->setInitialArguments({b->CreateTrunc(mHasBlockChecksum, b->getInt1Ty()), mHeaderSize, mFileSize});
    mPxDriver.makeKernelCall(blockDecoderK, {mCompressedByteStream}, {blockInfo.isCompress, blockInfo.blockStart, blockInfo.blockEnd});

    return blockInfo;
}


// Kernel Pipeline
