//
// Created by wxy325 on 2018/6/18.
//

#include "LZParabixGenerator.h"

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
#include <kernels/lzparabix/LZParabixBlockDecoder.h>
#include <kernels/lzparabix/LZParabixAioKernel.h>

namespace re { class CC; }

using namespace llvm;
using namespace parabix;
using namespace kernel;

LZParabixGenerator::LZParabixGenerator(): mPxDriver("lzParabixDecoder"), mLzParabixBlockSize(4 * 1024 * 1024) {

}

MainFunctionType LZParabixGenerator::getMainFunc() {
    return reinterpret_cast<MainFunctionType>(mPxDriver.getMain());
}



void LZParabixGenerator::generatePipeline(const std::string &outputFile) {
    auto & iBuilder = mPxDriver.getBuilder();
    this->generateMainFunc(iBuilder);

    this->generateLoadByteStreamAndBitStream(iBuilder);
    auto decompressedBitStream = this->generateAioBitStreamDecompressoin(iBuilder, {mCompressedBasisBits})[0];

    auto decompressedByteStream = mPxDriver.addBuffer<StaticBuffer>(iBuilder, iBuilder->getStreamSetTy(1, 8), this->getInputBufferBlocks(iBuilder));
    Kernel * p2sK = mPxDriver.addKernelInstance<P2SKernel>(iBuilder);
    mPxDriver.makeKernelCall(p2sK, {decompressedBitStream}, {decompressedByteStream});

    // --------------------------------------------------------
    // End
    Kernel * outK = mPxDriver.addKernelInstance<FileSink>(iBuilder, 8);
    outK->setInitialArguments({iBuilder->GetString(outputFile)});
    mPxDriver.makeKernelCall(outK, {decompressedByteStream}, {});

    mPxDriver.generatePipelineIR();
    mPxDriver.deallocateBuffers();

    iBuilder->CreateRetVoid();

    mPxDriver.finalizeObject();
}

std::vector<parabix::StreamSetBuffer*> LZParabixGenerator::generateAioBitStreamDecompressoin(
        const std::unique_ptr<kernel::KernelBuilder> & iBuilder,
        std::vector<parabix::StreamSetBuffer*> bitStreamSets
) {
    StreamSetBuffer * const BlockData_BlockStart = mPxDriver.addBuffer<StaticBuffer>(iBuilder, iBuilder->getStreamSetTy(1, 64), this->getInputBufferBlocks(iBuilder), 1);
    StreamSetBuffer * const BlockData_BlockEnd = mPxDriver.addBuffer<StaticBuffer>(iBuilder, iBuilder->getStreamSetTy(1, 64), this->getInputBufferBlocks(iBuilder), 1);
    Kernel * blockDecoderK = mPxDriver.addKernelInstance<LZParabixBlockDecoderKernel>(iBuilder);
    blockDecoderK->setInitialArguments({mFileSize});
    mPxDriver.makeKernelCall(blockDecoderK, {mCompressedByteStream}, {BlockData_BlockStart, BlockData_BlockEnd});

    std::vector<parabix::StreamSetBuffer*> inputStreamSetParams = {mCompressedByteStream, BlockData_BlockStart, BlockData_BlockEnd};

    std::vector<unsigned> numsOfStreams;
    std::vector<parabix::StreamSetBuffer*> outputStreamSets;

    for (unsigned i = 0; i < bitStreamSets.size(); i++) {
        unsigned numOfStreams = bitStreamSets[i]->getNumOfStreams();
        inputStreamSetParams.push_back(bitStreamSets[i]);
        numsOfStreams.push_back(numOfStreams);
        outputStreamSets.push_back(mPxDriver.addBuffer<StaticBuffer>(iBuilder, iBuilder->getStreamSetTy(numOfStreams, 1), this->getInputBufferBlocks(iBuilder)));
    }

    Kernel * aioK = mPxDriver.addKernelInstance<LZParabixAioKernel>(iBuilder, numsOfStreams);
    aioK->setInitialArguments({mFileSize});

    mPxDriver.makeKernelCall(aioK, inputStreamSetParams, outputStreamSets);
    return outputStreamSets;
}


void LZParabixGenerator::generateLoadByteStreamAndBitStream(const std::unique_ptr<kernel::KernelBuilder> & iBuilder) {
    mCompressedByteStream = mPxDriver.addBuffer<ExternalBuffer>(iBuilder, iBuilder->getStreamSetTy(1, 8));
    mCompressedBasisBits = mPxDriver.addBuffer<StaticBuffer>(iBuilder, iBuilder->getStreamSetTy(8, 1), this->getInputBufferBlocks(iBuilder));

    kernel::Kernel * sourceK = mPxDriver.addKernelInstance<MemorySourceKernel>(iBuilder);
    sourceK->setInitialArguments({mInputStream, mFileSize});
    mPxDriver.makeKernelCall(sourceK, {}, {mCompressedByteStream});
    Kernel * s2pk = mPxDriver.addKernelInstance<S2PKernel>(iBuilder, cc::BitNumbering::LittleEndian);
    mPxDriver.makeKernelCall(s2pk, {mCompressedByteStream}, {mCompressedBasisBits});
}

void LZParabixGenerator::generateMainFunc(const std::unique_ptr<kernel::KernelBuilder> &iBuilder) {
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


int LZParabixGenerator::get4MbBufferBlocks() {
    return mLzParabixBlockSize / codegen::BlockSize;
}

int LZParabixGenerator::getInputBufferBlocks(const std::unique_ptr<kernel::KernelBuilder> & b) {
    return this->get4MbBufferBlocks() * 2;
}
int LZParabixGenerator::getDecompressedBufferBlocks(const std::unique_ptr<kernel::KernelBuilder> & b) {
    return this->get4MbBufferBlocks() * 2;
}

