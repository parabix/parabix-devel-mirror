//
// Created by wxy325 on 2018/6/24.
//

#include "LZParabixCompressorGenerator.h"

#include <boost/filesystem.hpp>
#include <boost/iostreams/device/mapped_file.hpp>


#include <cc/cc_compiler.h>

#include <kernels/cc_kernel.h>
#include <kernels/s2p_kernel.h>
#include <kernels/p2s_kernel.h>
#include <kernels/source_kernel.h>
#include <kernels/stdout_kernel.h>
#include <kernels/kernel_builder.h>
#include <kernels/lzparabix/encoder/LZParabixCompressionKernel.h>


namespace re { class CC; }

using namespace llvm;
using namespace parabix;
using namespace kernel;


LZParabixCompressorGenerator::LZParabixCompressorGenerator(): mPxDriver("lzParabixCompressor"), mLzParabixBlockSize(4 * 1024 * 1024) {

}

CompressorMainFunctionType LZParabixCompressorGenerator::getMainFunc() {
    return reinterpret_cast<CompressorMainFunctionType>(mPxDriver.getMain());
}

void LZParabixCompressorGenerator::generatePipeline(const std::string &outputFile) {
    auto & iBuilder = mPxDriver.getBuilder();
    this->generateMainFunc(iBuilder);

    this->generateLoadByteStreamAndBitStream(iBuilder);


    parabix::StreamSetBuffer* outputStream = mPxDriver.addBuffer<StaticBuffer>(iBuilder, iBuilder->getStreamSetTy(1, 8), this->getInputBufferBlocks(iBuilder));

    parabix::StreamSetBuffer* literalByteBuffer = mPxDriver.addBuffer<StaticBuffer>(iBuilder, iBuilder->getStreamSetTy(1, 8), this->getInputBufferBlocks(iBuilder));
    parabix::StreamSetBuffer* tokenByteBuffer = mPxDriver.addBuffer<StaticBuffer>(iBuilder, iBuilder->getStreamSetTy(1, 8), this->getInputBufferBlocks(iBuilder));

    parabix::StreamSetBuffer* strToBlockIndex = mPxDriver.addBuffer<StaticBuffer>(iBuilder, iBuilder->getStreamSetTy(1, 64), HASHNBCELLS4);
    parabix::StreamSetBuffer* strToMatchPos = mPxDriver.addBuffer<StaticBuffer>(iBuilder, iBuilder->getStreamSetTy(1, 32), HASHNBCELLS4);

    Kernel * compressionKernel = mPxDriver.addKernelInstance<LZParabixCompressionKernel>(iBuilder);
    compressionKernel->setInitialArguments({mFileSize});
    mPxDriver.makeKernelCall(compressionKernel, {mCompressedByteStream, mCompressedBasisBits}, {outputStream, strToBlockIndex, strToMatchPos, literalByteBuffer, tokenByteBuffer});

    // --------------------------------------------------------
    // End
    Kernel * outK = mPxDriver.addKernelInstance<FileSink>(iBuilder, 8);
    outK->setInitialArguments({iBuilder->GetString(outputFile)});
    mPxDriver.makeKernelCall(outK, {outputStream}, {});


    mPxDriver.generatePipelineIR();
    mPxDriver.deallocateBuffers();

    iBuilder->CreateRetVoid();

    mPxDriver.finalizeObject();
}

void LZParabixCompressorGenerator::generateLoadByteStreamAndBitStream(
        const std::unique_ptr<kernel::KernelBuilder> &iBuilder) {
    mCompressedByteStream = mPxDriver.addBuffer<ExternalBuffer>(iBuilder, iBuilder->getStreamSetTy(1, 8));
    mCompressedBasisBits = mPxDriver.addBuffer<StaticBuffer>(iBuilder, iBuilder->getStreamSetTy(8, 1), this->getInputBufferBlocks(iBuilder));

    kernel::Kernel * sourceK = mPxDriver.addKernelInstance<MemorySourceKernel>(iBuilder);
    sourceK->setInitialArguments({mInputStream, mFileSize});
    mPxDriver.makeKernelCall(sourceK, {}, {mCompressedByteStream});
    Kernel * s2pk = mPxDriver.addKernelInstance<S2PKernel>(iBuilder, cc::BitNumbering::BigEndian);
    mPxDriver.makeKernelCall(s2pk, {mCompressedByteStream}, {mCompressedBasisBits});

}

void LZParabixCompressorGenerator::generateMainFunc(const std::unique_ptr<kernel::KernelBuilder> &iBuilder) {
    Module * M = iBuilder->getModule();
    Type * const sizeTy = iBuilder->getSizeTy();
    Type * const voidTy = iBuilder->getVoidTy();
    Type * const inputType = iBuilder->getInt8PtrTy();

    Function * const main = cast<Function>(M->getOrInsertFunction("Main", voidTy, inputType, sizeTy, nullptr));
    main->setCallingConv(CallingConv::C);
    Function::arg_iterator args = main->arg_begin();
    mInputStream = &*(args++);
    mInputStream->setName("input");

    mFileSize = &*(args++);
    mFileSize->setName("mFileSize");


    iBuilder->SetInsertPoint(BasicBlock::Create(M->getContext(), "entry", main, 0));
}

size_t LZParabixCompressorGenerator::getInputBufferBlocks(const std::unique_ptr<kernel::KernelBuilder> &iBuilder) {
    return mLzParabixBlockSize / codegen::BlockSize * 2;
}






