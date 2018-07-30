
#include "LZ4Generator.h"

#include <boost/filesystem.hpp>
#include <boost/iostreams/device/mapped_file.hpp>


#include <cc/cc_compiler.h>

#include <kernels/cc_kernel.h>
#include <kernels/s2p_kernel.h>
#include <kernels/p2s_kernel.h>
#include <kernels/source_kernel.h>
#include <kernels/stdout_kernel.h>
#include <kernels/lz4/lz4_generate_deposit_stream.h>
#include <kernels/kernel_builder.h>
#include <kernels/deletion.h>
#include <kernels/swizzle.h>
#include <kernels/pdep_kernel.h>
#include <kernels/swizzled_multiple_pdep_kernel.h>
#include <kernels/lz4/lz4_swizzled_match_copy_kernel.h>
#include <kernels/lz4/lz4_bitstream_match_copy_kernel.h>
#include <kernels/lz4/lz4_block_decoder.h>
#include <kernels/lz4/lz4_index_builder.h>
#include <kernels/lz4/aio/lz4_bytestream_aio.h>
#include <kernels/lz4/aio/lz4_parallel_bytestream_aio.h>
#include <kernels/lz4/aio/lz4_swizzled_aio.h>
#include <kernels/lz4/aio/lz4_bitstream_aio.h>
#include <kernels/lz4/aio/lz4_twist_aio.h>
#include <kernels/bitstream_pdep_kernel.h>
#include <kernels/lz4/lz4_bitstream_not_kernel.h>
#include <kernels/lz4/aio/twist_kernel.h>
#include <kernels/lz4/aio/untwist_kernel.h>

namespace re { class CC; }

using namespace llvm;
using namespace parabix;
using namespace kernel;

LZ4Generator::LZ4Generator():mPxDriver("lz4d"), mLz4BlockSize(4 * 1024 * 1024) {
    mCompressionMarker = NULL;
}

MainFunctionType LZ4Generator::getMainFunc() {
    return reinterpret_cast<MainFunctionType>(mPxDriver.getMain());
}

void LZ4Generator::generateNewExtractOnlyPipeline(const std::string &outputFile) {
    auto & iBuilder = mPxDriver.getBuilder();
    this->generateMainFunc(iBuilder);

    StreamSetBuffer * const DecompressedByteStream = mPxDriver.addBuffer<StaticBuffer>(iBuilder, iBuilder->getStreamSetTy(1, 8), this->getDecompressedBufferBlocks(iBuilder));

    // GeneratePipeline
    this->generateLoadByteStreamAndBitStream(iBuilder);

    //// Decode Block Information
    StreamSetBuffer * const BlockData_IsCompressed = mPxDriver.addBuffer<StaticBuffer>(iBuilder, iBuilder->getStreamSetTy(1, 8), this->getInputBufferBlocks(iBuilder), 1);
    StreamSetBuffer * const BlockData_BlockStart = mPxDriver.addBuffer<StaticBuffer>(iBuilder, iBuilder->getStreamSetTy(1, 64), this->getInputBufferBlocks(iBuilder), 1);
    StreamSetBuffer * const BlockData_BlockEnd = mPxDriver.addBuffer<StaticBuffer>(iBuilder, iBuilder->getStreamSetTy(1, 64), this->getInputBufferBlocks(iBuilder), 1);

    //// Generate Helper Markers Extenders, FX, XF
    StreamSetBuffer * const Extenders = mPxDriver.addBuffer<StaticBuffer>(iBuilder, iBuilder->getStreamSetTy(1, 1), this->getInputBufferBlocks(iBuilder), 1);
    mMatchOffsetMarker = mPxDriver.addBuffer<StaticBuffer>(iBuilder, iBuilder->getStreamSetTy(1, 1), this->getInputBufferBlocks(iBuilder));
    Kernel * extenderK = mPxDriver.addKernelInstance<ParabixCharacterClassKernelBuilder>(iBuilder, "extenders", std::vector<re::CC *>{re::makeCC(0xFF)}, 8);
    mPxDriver.makeKernelCall(extenderK, {mCompressedBasisBits}, {Extenders});


    Kernel * blockDecoderK = mPxDriver.addKernelInstance<LZ4BlockDecoderKernel>(iBuilder);
    blockDecoderK->setInitialArguments({iBuilder->CreateTrunc(mHasBlockChecksum, iBuilder->getInt1Ty()), mHeaderSize, mFileSize});
    mPxDriver.makeKernelCall(blockDecoderK, {mCompressedByteStream}, {BlockData_IsCompressed, BlockData_BlockStart, BlockData_BlockEnd});

    //// Generate Extract/Deposit Markers, M0_Start, M0_End, MatchOffset
    //TODO handle uncompressed part
    StreamSetBuffer * const UncompressedStartPos = mPxDriver.addBuffer<StaticBuffer>(iBuilder, iBuilder->getStreamSetTy(1, 64), this->getInputBufferBlocks(iBuilder), 1);
    StreamSetBuffer * const UncompressedLength = mPxDriver.addBuffer<StaticBuffer>(iBuilder, iBuilder->getStreamSetTy(1, 64), this->getInputBufferBlocks(iBuilder), 1);
    StreamSetBuffer * const UncompressedOutputPos = mPxDriver.addBuffer<StaticBuffer>(iBuilder, iBuilder->getStreamSetTy(1, 64), this->getInputBufferBlocks(iBuilder), 1);

    mCompressionMarker = mPxDriver.addBuffer<StaticBuffer>(iBuilder, iBuilder->getStreamSetTy(1, 1), this->getInputBufferBlocks(iBuilder));
    mM0Marker = mPxDriver.addBuffer<StaticBuffer>(iBuilder, iBuilder->getStreamSetTy(1, 1), this->getDecompressedBufferBlocks(iBuilder));
    mDepositMarker = mPxDriver.addBuffer<StaticBuffer>(iBuilder, iBuilder->getStreamSetTy(1, 1), this->getDecompressedBufferBlocks(iBuilder));

    Kernel* Lz4IndexBuilderK = mPxDriver.addKernelInstance<LZ4IndexBuilderKernel>(iBuilder);
    Lz4IndexBuilderK->setInitialArguments({mFileSize});
    mPxDriver.makeKernelCall(
            Lz4IndexBuilderK,
            {
                    mCompressedByteStream,
                    Extenders,

                    // Block Data
                    BlockData_IsCompressed,
                    BlockData_BlockStart,
                    BlockData_BlockEnd
            }, {
                    //Uncompressed Data
                    UncompressedStartPos,
                    UncompressedLength,
                    UncompressedOutputPos,

                    mCompressionMarker,
                    mM0Marker,
                    mMatchOffsetMarker
            });

    Kernel * generateDepositK = mPxDriver.addKernelInstance<LZ4GenerateDepositStreamKernel>(iBuilder);
    mPxDriver.makeKernelCall(generateDepositK, {mM0Marker}, {mDepositMarker});


    // Deletion
    StreamSetBuffer * deletedBits = mPxDriver.addBuffer<StaticBuffer>(iBuilder, iBuilder->getStreamSetTy(8), this->getInputBufferBlocks(iBuilder));
    StreamSetBuffer * deletionCounts = mPxDriver.addBuffer<StaticBuffer>(iBuilder, iBuilder->getStreamSetTy(), this->getInputBufferBlocks(iBuilder));

    Kernel * delK = mPxDriver.addKernelInstance<PEXTFieldCompressKernel>(iBuilder, 64, 8);
    mPxDriver.makeKernelCall(delK, {mCompressedBasisBits, mCompressionMarker}, {deletedBits, deletionCounts});

    StreamSetBuffer * compressedBits = mPxDriver.addBuffer<StaticBuffer>(iBuilder, iBuilder->getStreamSetTy(8), this->getInputBufferBlocks(iBuilder));
    Kernel * streamCompressionK = mPxDriver.addKernelInstance<StreamCompressKernel>(iBuilder, 64, 8);
    mPxDriver.makeKernelCall(streamCompressionK, {deletedBits, deletionCounts}, {compressedBits});


    StreamSetBuffer * const extractedBits = compressedBits;

    Kernel * p2sK = mPxDriver.addKernelInstance<P2SKernel>(iBuilder);
    mPxDriver.makeKernelCall(p2sK, {extractedBits}, {DecompressedByteStream});

    // --------------------------------------------------------
    // End
    Kernel * outK = mPxDriver.addKernelInstance<FileSink>(iBuilder, 8);

    outK->setInitialArguments({iBuilder->GetString(outputFile)});
    mPxDriver.makeKernelCall(outK, {DecompressedByteStream}, {});

    mPxDriver.generatePipelineIR();
    mPxDriver.deallocateBuffers();

    iBuilder->CreateRetVoid();

    mPxDriver.finalizeObject();
}

void LZ4Generator::generateExtractOnlyPipeline(const std::string &outputFile) {
    auto & iBuilder = mPxDriver.getBuilder();
    this->generateMainFunc(iBuilder);

    StreamSetBuffer * const DecompressedByteStream = mPxDriver.addBuffer<StaticBuffer>(iBuilder, iBuilder->getStreamSetTy(1, 8), this->getDecompressedBufferBlocks(iBuilder));

    // GeneratePipeline
    this->generateLoadByteStreamAndBitStream(iBuilder);
    this->generateExtractAndDepositMarkers(iBuilder);
    StreamSetBuffer * const extractedBits = this->generateBitStreamExtractData(iBuilder);

    Kernel * p2sK = mPxDriver.addKernelInstance<P2SKernel>(iBuilder);
    mPxDriver.makeKernelCall(p2sK, {extractedBits}, {DecompressedByteStream});

    // --------------------------------------------------------
    // End
    Kernel * outK = mPxDriver.addKernelInstance<FileSink>(iBuilder, 8);

    outK->setInitialArguments({iBuilder->GetString(outputFile)});
    mPxDriver.makeKernelCall(outK, {DecompressedByteStream}, {});

    mPxDriver.generatePipelineIR();
    mPxDriver.deallocateBuffers();

    iBuilder->CreateRetVoid();

    mPxDriver.finalizeObject();
}

void LZ4Generator::generateSwizzledExtractOnlyPipeline(const std::string &outputFile) {
    auto & iBuilder = mPxDriver.getBuilder();
    this->generateMainFunc(iBuilder);

    StreamSetBuffer * const DecompressedByteStream = mPxDriver.addBuffer<StaticBuffer>(iBuilder, iBuilder->getStreamSetTy(1, 8), this->getDecompressedBufferBlocks(iBuilder));

    // GeneratePipeline
    this->generateLoadByteStreamAndBitStream(iBuilder);
    this->generateExtractAndDepositMarkers(iBuilder);
    auto swizzle = this->generateSwizzleExtractData(iBuilder);


    // Produce unswizzled bit streams
    StreamSetBuffer * extractedbits = mPxDriver.addBuffer<StaticBuffer>(iBuilder, iBuilder->getStreamSetTy(8), this->getInputBufferBlocks(iBuilder));
    Kernel * unSwizzleK = mPxDriver.addKernelInstance<SwizzleGenerator>(iBuilder, 8, 1, 2);

    mPxDriver.makeKernelCall(unSwizzleK, {swizzle.first, swizzle.second}, {extractedbits});


    Kernel * p2sK = mPxDriver.addKernelInstance<P2SKernel>(iBuilder);
    mPxDriver.makeKernelCall(p2sK, {extractedbits}, {DecompressedByteStream});

    // --------------------------------------------------------
    // End
    Kernel * outK = mPxDriver.addKernelInstance<FileSink>(iBuilder, 8);

    outK->setInitialArguments({iBuilder->GetString(outputFile)});
    mPxDriver.makeKernelCall(outK, {DecompressedByteStream}, {});

    mPxDriver.generatePipelineIR();
    mPxDriver.deallocateBuffers();

    iBuilder->CreateRetVoid();

    mPxDriver.finalizeObject();
}

void LZ4Generator::generateExtractAndDepositOnlyPipeline(const std::string &outputFile) {
    auto & iBuilder = mPxDriver.getBuilder();
    this->generateMainFunc(iBuilder);

    StreamSetBuffer * const DecompressedByteStream = mPxDriver.addBuffer<StaticBuffer>(iBuilder, iBuilder->getStreamSetTy(1, 8), this->getDecompressedBufferBlocks(iBuilder));

    // GeneratePipeline
    this->generateLoadByteStreamAndBitStream(iBuilder);
    this->generateExtractAndDepositMarkers(iBuilder);
    StreamSetBuffer * const extractedBits = this->generateBitStreamExtractData(iBuilder);

    StreamSetBuffer * depositedBits = mPxDriver.addBuffer<StaticBuffer>(iBuilder, iBuilder->getStreamSetTy(8), this->getDecompressedBufferBlocks(iBuilder));
    Kernel * bitStreamPDEPk = mPxDriver.addKernelInstance<BitStreamPDEPKernel>(iBuilder, 8);
    mPxDriver.makeKernelCall(bitStreamPDEPk, {mDepositMarker, extractedBits}, {depositedBits});

    Kernel * p2sK = mPxDriver.addKernelInstance<P2SKernel>(iBuilder);
    mPxDriver.makeKernelCall(p2sK, {depositedBits}, {DecompressedByteStream});

    // --------------------------------------------------------
    // End
    Kernel * outK = mPxDriver.addKernelInstance<FileSink>(iBuilder, 8);

    outK->setInitialArguments({iBuilder->GetString(outputFile)});
    mPxDriver.makeKernelCall(outK, {DecompressedByteStream}, {});

    mPxDriver.generatePipelineIR();
    mPxDriver.deallocateBuffers();

    iBuilder->CreateRetVoid();

    mPxDriver.finalizeObject();
}

void LZ4Generator::generateSwizzledExtractAndDepositOnlyPipeline(const std::string &outputFile) {
    auto & iBuilder = mPxDriver.getBuilder();
    this->generateMainFunc(iBuilder);

    StreamSetBuffer * const DecompressedByteStream = mPxDriver.addBuffer<StaticBuffer>(iBuilder, iBuilder->getStreamSetTy(1, 8), this->getDecompressedBufferBlocks(iBuilder));

    // GeneratePipeline
    this->generateLoadByteStreamAndBitStream(iBuilder);
    this->generateExtractAndDepositMarkers(iBuilder);

    auto swizzle = this->generateSwizzleExtractData(iBuilder);

    StreamSetBuffer * depositedSwizzle0 = mPxDriver.addBuffer<StaticBuffer>(iBuilder, iBuilder->getStreamSetTy(4), this->getInputBufferBlocks(iBuilder), 1);
    StreamSetBuffer * depositedSwizzle1 = mPxDriver.addBuffer<StaticBuffer>(iBuilder, iBuilder->getStreamSetTy(4), this->getInputBufferBlocks(iBuilder), 1);

    Kernel * multiplePdepK = mPxDriver.addKernelInstance<SwizzledMultiplePDEPkernel>(iBuilder, 4, 2);
    mPxDriver.makeKernelCall(multiplePdepK, {mDepositMarker, swizzle.first, swizzle.second}, {depositedSwizzle0, depositedSwizzle1});

    // Produce unswizzled bit streams
    StreamSetBuffer * depositedBits = mPxDriver.addBuffer<StaticBuffer>(iBuilder, iBuilder->getStreamSetTy(8), this->getInputBufferBlocks(iBuilder));
    Kernel * unSwizzleK = mPxDriver.addKernelInstance<SwizzleGenerator>(iBuilder, 8, 1, 2);
    mPxDriver.makeKernelCall(unSwizzleK, {depositedSwizzle0, depositedSwizzle1}, {depositedBits});

    Kernel * p2sK = mPxDriver.addKernelInstance<P2SKernel>(iBuilder);
    mPxDriver.makeKernelCall(p2sK, {depositedBits}, {DecompressedByteStream});

    // --------------------------------------------------------
    // End
    Kernel * outK = mPxDriver.addKernelInstance<FileSink>(iBuilder, 8);
    outK->setInitialArguments({iBuilder->GetString(outputFile)});
    mPxDriver.makeKernelCall(outK, {DecompressedByteStream}, {});

    mPxDriver.generatePipelineIR();
    mPxDriver.deallocateBuffers();

    iBuilder->CreateRetVoid();

    mPxDriver.finalizeObject();
}

void LZ4Generator::generatePipeline(const std::string &outputFile) {
    auto & iBuilder = mPxDriver.getBuilder();
    this->generateMainFunc(iBuilder);

    StreamSetBuffer * const DecompressedByteStream = mPxDriver.addBuffer<StaticBuffer>(iBuilder, iBuilder->getStreamSetTy(1, 8), this->getDecompressedBufferBlocks(iBuilder));

    // GeneratePipeline
    this->generateLoadByteStreamAndBitStream(iBuilder);
    this->generateExtractAndDepositMarkers(iBuilder);
    StreamSetBuffer * const extractedBits = this->generateBitStreamExtractData(iBuilder);

    StreamSetBuffer * depositedBits = mPxDriver.addBuffer<StaticBuffer>(iBuilder, iBuilder->getStreamSetTy(8), this->getDecompressedBufferBlocks(iBuilder));
    Kernel * bitStreamPDEPk = mPxDriver.addKernelInstance<BitStreamPDEPKernel>(iBuilder, 8);
    mPxDriver.makeKernelCall(bitStreamPDEPk, {mDepositMarker, extractedBits}, {depositedBits});

    StreamSetBuffer * matchCopiedBits = mPxDriver.addBuffer<StaticBuffer>(iBuilder, iBuilder->getStreamSetTy(8), this->getInputBufferBlocks(iBuilder));
    Kernel * bitStreamMatchCopyK = mPxDriver.addKernelInstance<LZ4BitStreamMatchCopyKernel>(iBuilder, 8);
    mPxDriver.makeKernelCall(bitStreamMatchCopyK, {mMatchOffsetMarker, mM0Marker, mCompressedByteStream, depositedBits}, {matchCopiedBits});

    Kernel * p2sK = mPxDriver.addKernelInstance<P2SKernel>(iBuilder);
    mPxDriver.makeKernelCall(p2sK, {matchCopiedBits}, {DecompressedByteStream});

    // --------------------------------------------------------
    // End
    Kernel * outK = mPxDriver.addKernelInstance<FileSink>(iBuilder, 8);

    outK->setInitialArguments({iBuilder->GetString(outputFile)});
    mPxDriver.makeKernelCall(outK, {DecompressedByteStream}, {});

    mPxDriver.generatePipelineIR();
    mPxDriver.deallocateBuffers();

    iBuilder->CreateRetVoid();

    mPxDriver.finalizeObject();
}

void LZ4Generator::generateSwizzledPipeline(const std::string &outputFile) {
    auto & iBuilder = mPxDriver.getBuilder();
    this->generateMainFunc(iBuilder);

    StreamSetBuffer * const DecompressedByteStream = mPxDriver.addBuffer<StaticBuffer>(iBuilder, iBuilder->getStreamSetTy(1, 8), this->getDecompressedBufferBlocks(iBuilder));

    // GeneratePipeline
    this->generateLoadByteStreamAndBitStream(iBuilder);
    this->generateExtractAndDepositMarkers(iBuilder);

    auto swizzle = this->generateSwizzleExtractData(iBuilder);

    StreamSetBuffer * depositedSwizzle0 = mPxDriver.addBuffer<StaticBuffer>(iBuilder, iBuilder->getStreamSetTy(4), this->getDecompressedBufferBlocks(iBuilder), 1);
    StreamSetBuffer * depositedSwizzle1 = mPxDriver.addBuffer<StaticBuffer>(iBuilder, iBuilder->getStreamSetTy(4), this->getDecompressedBufferBlocks(iBuilder), 1);

    Kernel * multiplePdepK = mPxDriver.addKernelInstance<SwizzledMultiplePDEPkernel>(iBuilder, 4, 2);
    mPxDriver.makeKernelCall(multiplePdepK, {mDepositMarker, swizzle.first, swizzle.second}, {depositedSwizzle0, depositedSwizzle1});

    StreamSetBuffer * matchCopiedSwizzle0 = mPxDriver.addBuffer<StaticBuffer>(iBuilder, iBuilder->getStreamSetTy(4), this->getDecompressedBufferBlocks(iBuilder), 1);
    StreamSetBuffer * matchCopiedSwizzle1 = mPxDriver.addBuffer<StaticBuffer>(iBuilder, iBuilder->getStreamSetTy(4), this->getDecompressedBufferBlocks(iBuilder), 1);

    Kernel * swizzledMatchCopyK = mPxDriver.addKernelInstance<LZ4SwizzledMatchCopyKernel>(iBuilder, 4, 2, 4);
    mPxDriver.makeKernelCall(swizzledMatchCopyK, {mMatchOffsetMarker, mM0Marker, mCompressedByteStream, depositedSwizzle0, depositedSwizzle1}, {matchCopiedSwizzle0, matchCopiedSwizzle1});


    // Produce unswizzled bit streams
    StreamSetBuffer * matchCopiedBits = mPxDriver.addBuffer<StaticBuffer>(iBuilder, iBuilder->getStreamSetTy(8), this->getInputBufferBlocks(iBuilder));
    Kernel * unSwizzleK = mPxDriver.addKernelInstance<SwizzleGenerator>(iBuilder, 8, 1, 2);
    mPxDriver.makeKernelCall(unSwizzleK, {matchCopiedSwizzle0, matchCopiedSwizzle1}, {matchCopiedBits});


    Kernel * p2sK = mPxDriver.addKernelInstance<P2SKernel>(iBuilder);
    mPxDriver.makeKernelCall(p2sK, {matchCopiedBits}, {DecompressedByteStream});

    // --------------------------------------------------------
    // End
    Kernel * outK = mPxDriver.addKernelInstance<FileSink>(iBuilder, 8);
    outK->setInitialArguments({iBuilder->GetString(outputFile)});
    mPxDriver.makeKernelCall(outK, {DecompressedByteStream}, {});

    mPxDriver.generatePipelineIR();
    mPxDriver.deallocateBuffers();

    iBuilder->CreateRetVoid();

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

    //// Decode Block Information
    StreamSetBuffer * const BlockData_IsCompressed = mPxDriver.addBuffer<StaticBuffer>(iBuilder, iBuilder->getStreamSetTy(1, 8), this->getInputBufferBlocks(iBuilder), 1);
    StreamSetBuffer * const BlockData_BlockStart = mPxDriver.addBuffer<StaticBuffer>(iBuilder, iBuilder->getStreamSetTy(1, 64), this->getInputBufferBlocks(iBuilder), 1);
    StreamSetBuffer * const BlockData_BlockEnd = mPxDriver.addBuffer<StaticBuffer>(iBuilder, iBuilder->getStreamSetTy(1, 64), this->getInputBufferBlocks(iBuilder), 1);


    Kernel * blockDecoderK = mPxDriver.addKernelInstance<LZ4BlockDecoderKernel>(iBuilder);
    blockDecoderK->setInitialArguments({iBuilder->CreateTrunc(mHasBlockChecksum, iBuilder->getInt1Ty()), mHeaderSize, mFileSize});
    mPxDriver.makeKernelCall(blockDecoderK, {mCompressedByteStream}, {BlockData_IsCompressed, BlockData_BlockStart, BlockData_BlockEnd});



    size_t numOfStreams = compressedBitStreams[0]->getNumOfStreams();

    // 1, 2, 4, 8

    if (numOfStreams <= 2) {
        StreamSetBuffer* twistedCharClasses = mGrepDriver->addBuffer<StaticBuffer>(iBuilder, iBuilder->getStreamSetTy(1, 2), this->getInputBufferBlocks(iBuilder));
        kernel::Kernel* twistK = mGrepDriver->addKernelInstance<kernel::TwistByPDEPKernel>(iBuilder, numOfStreams, 2);
        mGrepDriver->makeKernelCall(twistK, {compressedBitStreams[0]}, {twistedCharClasses});

        StreamSetBuffer* uncompressedTwistedCharClasses = mGrepDriver->addBuffer<StaticBuffer>(iBuilder, iBuilder->getStreamSetTy(1, 2), this->getInputBufferBlocks(iBuilder));
        Kernel* lz4I4AioK = mPxDriver.addKernelInstance<LZ4TwistAioKernel>(iBuilder, 2);
        lz4I4AioK->setInitialArguments({mFileSize});
        mGrepDriver->makeKernelCall(lz4I4AioK, {
                mCompressedByteStream,

                // Block Data
                BlockData_IsCompressed,
                BlockData_BlockStart,
                BlockData_BlockEnd,

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

        Kernel* lz4I4AioK = mPxDriver.addKernelInstance<LZ4TwistAioKernel>(iBuilder, 4);
        lz4I4AioK->setInitialArguments({mFileSize});
        mGrepDriver->makeKernelCall(lz4I4AioK, {
                mCompressedByteStream,

                // Block Data
                BlockData_IsCompressed,
                BlockData_BlockStart,
                BlockData_BlockEnd,

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

            // Block Data
            BlockData_IsCompressed,
            BlockData_BlockStart,
            BlockData_BlockEnd
    };

    std::vector<StreamSetBuffer *> outputStream;
    std::vector<unsigned> numbersOfStreams;

    for (unsigned i = 0; i < compressedBitStreams.size(); i++) {
        unsigned numOfStreams = compressedBitStreams[i]->getNumOfStreams();
        numbersOfStreams.push_back(numOfStreams);
        inputStreams.push_back(compressedBitStreams[i]);
        outputStream.push_back(mPxDriver.addBuffer<StaticBuffer>(iBuilder, iBuilder->getStreamSetTy(numOfStreams, 1), this->getInputBufferBlocks(iBuilder)));
    }

    Kernel* lz4AioK = mPxDriver.addKernelInstance<LZ4BitStreamAioKernel>(iBuilder, numbersOfStreams);
    lz4AioK->setInitialArguments({mFileSize});
    mPxDriver.makeKernelCall(lz4AioK, inputStreams, outputStream);

    return outputStream;
}


StreamSetBuffer * LZ4Generator::generateSwizzledAIODecompression(const std::unique_ptr<kernel::KernelBuilder> & iBuilder) {
    //// Decode Block Information
    StreamSetBuffer * const BlockData_IsCompressed = mPxDriver.addBuffer<StaticBuffer>(iBuilder, iBuilder->getStreamSetTy(1, 8), this->getInputBufferBlocks(iBuilder), 1);
    StreamSetBuffer * const BlockData_BlockStart = mPxDriver.addBuffer<StaticBuffer>(iBuilder, iBuilder->getStreamSetTy(1, 64), this->getInputBufferBlocks(iBuilder), 1);
    StreamSetBuffer * const BlockData_BlockEnd = mPxDriver.addBuffer<StaticBuffer>(iBuilder, iBuilder->getStreamSetTy(1, 64), this->getInputBufferBlocks(iBuilder), 1);

    //// Generate Helper Markers Extenders, FX, XF
//    StreamSetBuffer * const Extenders = mPxDriver.addBuffer<StaticBuffer>(iBuilder, iBuilder->getStreamSetTy(1, 1), this->getInputBufferBlocks(iBuilder), 1);
//    mMatchOffsetMarker = mPxDriver.addBuffer<StaticBuffer>(iBuilder, iBuilder->getStreamSetTy(1, 1), this->getInputBufferBlocks(iBuilder));
//    Kernel * extenderK = mPxDriver.addKernelInstance<ParabixCharacterClassKernelBuilder>(iBuilder, "extenders", std::vector<re::CC *>{re::makeCC(0xFF)}, 8);
//    mPxDriver.makeKernelCall(extenderK, {mCompressedBasisBits}, {Extenders});


    Kernel * blockDecoderK = mPxDriver.addKernelInstance<LZ4BlockDecoderKernel>(iBuilder);
    blockDecoderK->setInitialArguments({iBuilder->CreateTrunc(mHasBlockChecksum, iBuilder->getInt1Ty()), mHeaderSize, mFileSize});
    mPxDriver.makeKernelCall(blockDecoderK, {mCompressedByteStream}, {BlockData_IsCompressed, BlockData_BlockStart, BlockData_BlockEnd});


    // Produce unswizzled bit streams
    StreamSetBuffer * u16Swizzle0 = mPxDriver.addBuffer<StaticBuffer>(iBuilder, iBuilder->getStreamSetTy(4), this->getInputBufferBlocks(iBuilder), 1);
    StreamSetBuffer * u16Swizzle1 = mPxDriver.addBuffer<StaticBuffer>(iBuilder, iBuilder->getStreamSetTy(4), this->getInputBufferBlocks(iBuilder), 1);
    Kernel * unSwizzleK = mPxDriver.addKernelInstance<SwizzleGenerator>(iBuilder, 8, 2, 1, 64, "source");
    mPxDriver.makeKernelCall(unSwizzleK, {mCompressedBasisBits}, {u16Swizzle0, u16Swizzle1});



    StreamSetBuffer * decompressedSwizzled0 = mPxDriver.addBuffer<StaticBuffer>(iBuilder, iBuilder->getStreamSetTy(4), this->getInputBufferBlocks(iBuilder), 1);
    StreamSetBuffer * decompressedSwizzled1 = mPxDriver.addBuffer<StaticBuffer>(iBuilder, iBuilder->getStreamSetTy(4), this->getInputBufferBlocks(iBuilder), 1);


    Kernel* lz4AioK = mPxDriver.addKernelInstance<LZ4SwizzledAioKernel>(iBuilder, 4, 2, 4);
    lz4AioK->setInitialArguments({mFileSize});
    mPxDriver.makeKernelCall(
            lz4AioK,
            {
                    mCompressedByteStream,

//                    Extenders,

                    // Block Data
                    BlockData_IsCompressed,
                    BlockData_BlockStart,
                    BlockData_BlockEnd,

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
    //// Decode Block Information
    StreamSetBuffer * const BlockData_IsCompressed = mPxDriver.addBuffer<StaticBuffer>(iBuilder, iBuilder->getStreamSetTy(1, 8), this->getInputBufferBlocks(iBuilder), 1);
    StreamSetBuffer * const BlockData_BlockStart = mPxDriver.addBuffer<StaticBuffer>(iBuilder, iBuilder->getStreamSetTy(1, 64), this->getInputBufferBlocks(iBuilder), 1);
    StreamSetBuffer * const BlockData_BlockEnd = mPxDriver.addBuffer<StaticBuffer>(iBuilder, iBuilder->getStreamSetTy(1, 64), this->getInputBufferBlocks(iBuilder), 1);

    //// Generate Helper Markers Extenders
//    StreamSetBuffer * const Extenders = mPxDriver.addBuffer<StaticBuffer>(iBuilder, iBuilder->getStreamSetTy(1, 1), this->getInputBufferBlocks(iBuilder), 1);
//    mMatchOffsetMarker = mPxDriver.addBuffer<StaticBuffer>(iBuilder, iBuilder->getStreamSetTy(1, 1), this->getInputBufferBlocks(iBuilder));
//    Kernel * extenderK = mPxDriver.addKernelInstance<ParabixCharacterClassKernelBuilder>(iBuilder, "extenders", std::vector<re::CC *>{re::makeCC(0xFF)}, 8);
//    mPxDriver.makeKernelCall(extenderK, {mCompressedBasisBits}, {Extenders});

    Kernel * blockDecoderK = mPxDriver.addKernelInstance<LZ4BlockDecoderKernel>(iBuilder);
    blockDecoderK->setInitialArguments({iBuilder->CreateTrunc(mHasBlockChecksum, iBuilder->getInt1Ty()), mHeaderSize, mFileSize});
    mPxDriver.makeKernelCall(blockDecoderK, {mCompressedByteStream}, {BlockData_IsCompressed, BlockData_BlockStart, BlockData_BlockEnd});


    StreamSetBuffer * const decompressionByteStream = mPxDriver.addBuffer<StaticBuffer>(iBuilder, iBuilder->getStreamSetTy(1, 8), this->getDecompressedBufferBlocks(iBuilder), 1);

    Kernel* lz4AioK = mPxDriver.addKernelInstance<LZ4ParallelByteStreamAioKernel>(iBuilder, mLz4BlockSize, enableGather, enableScatter, minParallelLevel);
    lz4AioK->setInitialArguments({mFileSize});
    mPxDriver.makeKernelCall(
            lz4AioK,
            {
                    mCompressedByteStream,

//                    Extenders,

                    // Block Data
                    BlockData_IsCompressed,
                    BlockData_BlockStart,
                    BlockData_BlockEnd
            }, {
                    decompressionByteStream
            });

    return decompressionByteStream;

}

StreamSetBuffer * LZ4Generator::generateAIODecompression(const std::unique_ptr<kernel::KernelBuilder> & iBuilder) {
    LZ4BlockInfo blockInfo = this->getBlockInfo(iBuilder);

    StreamSetBuffer * const decompressionByteStream = mPxDriver.addBuffer<StaticBuffer>(iBuilder, iBuilder->getStreamSetTy(1, 8), this->getDecompressedBufferBlocks(iBuilder), 1);
    Kernel* lz4AioK = mPxDriver.addKernelInstance<LZ4ByteStreamAioKernel>(iBuilder);
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

void LZ4Generator::generateExtractAndDepositMarkers(const std::unique_ptr<kernel::KernelBuilder> & iBuilder) {
    //// Decode Block Information
    StreamSetBuffer * const BlockData_IsCompressed = mPxDriver.addBuffer<StaticBuffer>(iBuilder, iBuilder->getStreamSetTy(1, 8), this->getInputBufferBlocks(iBuilder), 1);
    StreamSetBuffer * const BlockData_BlockStart = mPxDriver.addBuffer<StaticBuffer>(iBuilder, iBuilder->getStreamSetTy(1, 64), this->getInputBufferBlocks(iBuilder), 1);
    StreamSetBuffer * const BlockData_BlockEnd = mPxDriver.addBuffer<StaticBuffer>(iBuilder, iBuilder->getStreamSetTy(1, 64), this->getInputBufferBlocks(iBuilder), 1);

    //// Generate Helper Markers Extenders, FX, XF
    StreamSetBuffer * const Extenders = mPxDriver.addBuffer<StaticBuffer>(iBuilder, iBuilder->getStreamSetTy(1, 1), this->getInputBufferBlocks(iBuilder), 1);
    mMatchOffsetMarker = mPxDriver.addBuffer<StaticBuffer>(iBuilder, iBuilder->getStreamSetTy(1, 1), this->getInputBufferBlocks(iBuilder));
    Kernel * extenderK = mPxDriver.addKernelInstance<ParabixCharacterClassKernelBuilder>(iBuilder, "extenders", std::vector<re::CC *>{re::makeCC(0xFF)}, 8);
    mPxDriver.makeKernelCall(extenderK, {mCompressedBasisBits}, {Extenders});


    Kernel * blockDecoderK = mPxDriver.addKernelInstance<LZ4BlockDecoderKernel>(iBuilder);
    blockDecoderK->setInitialArguments({iBuilder->CreateTrunc(mHasBlockChecksum, iBuilder->getInt1Ty()), mHeaderSize, mFileSize});
    mPxDriver.makeKernelCall(blockDecoderK, {mCompressedByteStream}, {BlockData_IsCompressed, BlockData_BlockStart, BlockData_BlockEnd});

    //// Generate Extract/Deposit Markers, M0_Start, M0_End, MatchOffset

    //TODO handle uncompressed part
    StreamSetBuffer * const UncompressedStartPos = mPxDriver.addBuffer<StaticBuffer>(iBuilder, iBuilder->getStreamSetTy(1, 64), this->getInputBufferBlocks(iBuilder), 1);
    StreamSetBuffer * const UncompressedLength = mPxDriver.addBuffer<StaticBuffer>(iBuilder, iBuilder->getStreamSetTy(1, 64), this->getInputBufferBlocks(iBuilder), 1);
    StreamSetBuffer * const UncompressedOutputPos = mPxDriver.addBuffer<StaticBuffer>(iBuilder, iBuilder->getStreamSetTy(1, 64), this->getInputBufferBlocks(iBuilder), 1);

    mDeletionMarker = mPxDriver.addBuffer<StaticBuffer>(iBuilder, iBuilder->getStreamSetTy(1, 1), this->getInputBufferBlocks(iBuilder));
    mM0Marker = mPxDriver.addBuffer<StaticBuffer>(iBuilder, iBuilder->getStreamSetTy(1, 1), this->getDecompressedBufferBlocks(iBuilder));
    mDepositMarker = mPxDriver.addBuffer<StaticBuffer>(iBuilder, iBuilder->getStreamSetTy(1, 1), this->getDecompressedBufferBlocks(iBuilder));

    Kernel* Lz4IndexBuilderK = mPxDriver.addKernelInstance<LZ4IndexBuilderKernel>(iBuilder);
    Lz4IndexBuilderK->setInitialArguments({mFileSize});
    mPxDriver.makeKernelCall(
            Lz4IndexBuilderK,
            {
                    mCompressedByteStream,
                    Extenders,

                    // Block Data
                    BlockData_IsCompressed,
                    BlockData_BlockStart,
                    BlockData_BlockEnd
            }, {
                    //Uncompressed Data
                    UncompressedStartPos,
                    UncompressedLength,
                    UncompressedOutputPos,

                    mDeletionMarker,
                    mM0Marker,
                    mMatchOffsetMarker
            });

    Kernel * generateDepositK = mPxDriver.addKernelInstance<LZ4GenerateDepositStreamKernel>(iBuilder);
    mPxDriver.makeKernelCall(generateDepositK, {mM0Marker}, {mDepositMarker});

}

std::pair<StreamSetBuffer*, StreamSetBuffer*> LZ4Generator::generateSwizzleExtractData(const std::unique_ptr<kernel::KernelBuilder> & iBuilder) {
    StreamSetBuffer * u16Swizzle0 = mPxDriver.addBuffer<StaticBuffer>(iBuilder, iBuilder->getStreamSetTy(4), this->getInputBufferBlocks(iBuilder), 1);
    StreamSetBuffer * u16Swizzle1 = mPxDriver.addBuffer<StaticBuffer>(iBuilder, iBuilder->getStreamSetTy(4), this->getInputBufferBlocks(iBuilder), 1);

    Kernel * delK = mPxDriver.addKernelInstance<SwizzledDeleteByPEXTkernel>(iBuilder, 8, 64);
    mPxDriver.makeKernelCall(delK, {mDeletionMarker, mCompressedBasisBits}, {u16Swizzle0, u16Swizzle1});
    return std::make_pair(u16Swizzle0, u16Swizzle1);
}

void LZ4Generator::generateCompressionMarker(const std::unique_ptr<kernel::KernelBuilder> & iBuilder) {
    if (!mCompressionMarker) {
        mCompressionMarker = mPxDriver.addBuffer<StaticBuffer>(iBuilder, iBuilder->getStreamSetTy(1, 1), this->getInputBufferBlocks(iBuilder));
        Kernel * bitstreamNotK = mPxDriver.addKernelInstance<LZ4BitStreamNotKernel>(iBuilder);
        mPxDriver.makeKernelCall(bitstreamNotK, {mDeletionMarker}, {mCompressionMarker});
    }
}

parabix::StreamSetBuffer* LZ4Generator::generateBitStreamExtractData(const std::unique_ptr<kernel::KernelBuilder> & iBuilder) {
    this->generateCompressionMarker(iBuilder);

    // Deletion
    StreamSetBuffer * deletedBits = mPxDriver.addBuffer<StaticBuffer>(iBuilder, iBuilder->getStreamSetTy(8), this->getInputBufferBlocks(iBuilder));
    StreamSetBuffer * deletionCounts = mPxDriver.addBuffer<StaticBuffer>(iBuilder, iBuilder->getStreamSetTy(), this->getInputBufferBlocks(iBuilder));

    Kernel * delK = mPxDriver.addKernelInstance<PEXTFieldCompressKernel>(iBuilder, 64, 8);
    mPxDriver.makeKernelCall(delK, {mCompressedBasisBits, mCompressionMarker}, {deletedBits, deletionCounts});

    StreamSetBuffer * compressedBits = mPxDriver.addBuffer<StaticBuffer>(iBuilder, iBuilder->getStreamSetTy(8), this->getInputBufferBlocks(iBuilder));
    Kernel * streamCompressionK = mPxDriver.addKernelInstance<StreamCompressKernel>(iBuilder, 64, 8);
    mPxDriver.makeKernelCall(streamCompressionK, {deletedBits, deletionCounts}, {compressedBits});

    return compressedBits;
}

int LZ4Generator::get4MbBufferBlocks() {
    return mLz4BlockSize / codegen::BlockSize;
}

int LZ4Generator::getInputBufferBlocks(const std::unique_ptr<kernel::KernelBuilder> & b) {
    return this->get4MbBufferBlocks() * 2 * b->getBitBlockWidth() / 64;
}
int LZ4Generator::getDecompressedBufferBlocks(const std::unique_ptr<kernel::KernelBuilder> & b) {
    return this->get4MbBufferBlocks() * 2 * b->getBitBlockWidth() / 64;
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
