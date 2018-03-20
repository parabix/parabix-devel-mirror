

#include "LZ4GeneratorNew.h"

#include <boost/filesystem.hpp>
#include <boost/iostreams/device/mapped_file.hpp>

#include <llvm/Support/CommandLine.h>
#include <llvm/Support/PrettyStackTrace.h>

#include <cc/cc_compiler.h>

#include <lz4FrameDecoder.h>
#include <kernels/streamset.h>
#include <kernels/cc_kernel.h>
#include <kernels/s2p_kernel.h>
#include <kernels/p2s_kernel.h>
#include <kernels/source_kernel.h>
#include <kernels/stdout_kernel.h>
#include <kernels/lz4/lz4_extract_e_m0.h>
#include <kernels/lz4/lz4_generate_deposit_stream.h>
#include <kernels/lz4/lz4_numbers_to_bitstream_kernel.h>
#include <kernels/lz4/lz4_bitstream_not_kernel.h>
#include <kernels/kernel_builder.h>
#include <kernels/lz4/lz4_block_decoder.h>
#include <kernels/deletion.h>
#include <kernels/swizzle.h>
#include <kernels/pdep_kernel.h>
#include <kernels/lz4/lz4_multiple_pdep_kernel.h>
#include <kernels/lz4/lz4_match_copy_kernel.h>
#include <kernels/lz4/lz4_swizzled_match_copy_kernel.h>
#include <kernels/lz4/lz4_block_decoder_new.h>
#include <kernels/lz4/lz4_index_builder.h>

namespace re { class CC; }

using namespace llvm;
using namespace parabix;
using namespace kernel;

LZ4GeneratorNew::LZ4GeneratorNew():LZ4Generator() {

}

int LZ4GeneratorNew::get4MbBufferBlocks() {
    return 4 * 1024 * 1024 / codegen::BlockSize;
}

int LZ4GeneratorNew::getInputBufferBlocks() {
    return this->get4MbBufferBlocks();
}
int LZ4GeneratorNew::getDecompressedBufferBlocks() {
    return this->get4MbBufferBlocks();
}


void LZ4GeneratorNew::generateExtractAndDepositMarkers(const std::unique_ptr<kernel::KernelBuilder> & iBuilder) {
    //// Decode Block Information
    StreamSetBuffer * const BlockData_IsCompressed = pxDriver.addBuffer<CircularBuffer>(iBuilder, iBuilder->getStreamSetTy(1, 1), this->get4MbBufferBlocks());
    StreamSetBuffer * const BlockData_BlockStart = pxDriver.addBuffer<CircularBuffer>(iBuilder, iBuilder->getStreamSetTy(1, 64), this->get4MbBufferBlocks());
    StreamSetBuffer * const BlockData_BlockEnd = pxDriver.addBuffer<CircularBuffer>(iBuilder, iBuilder->getStreamSetTy(1, 64), this->get4MbBufferBlocks());

    //// Generate Helper Markers Extenders, FX, XF
    StreamSetBuffer * const Extenders = pxDriver.addBuffer<CircularBuffer>(iBuilder, iBuilder->getStreamSetTy(1, 1), this->get4MbBufferBlocks());
    StreamSetBuffer * const CC_0xFX = pxDriver.addBuffer<CircularBuffer>(iBuilder, iBuilder->getStreamSetTy(1, 1), this->get4MbBufferBlocks());
    StreamSetBuffer * const CC_0xXF = pxDriver.addBuffer<CircularBuffer>(iBuilder, iBuilder->getStreamSetTy(1, 1), this->get4MbBufferBlocks());


    Kernel * extenderK = pxDriver.addKernelInstance<ParabixCharacterClassKernelBuilder>(iBuilder, "extenders", std::vector<re::CC *>{re::makeCC(0xFF)}, 8);
    pxDriver.makeKernelCall(extenderK, {BasisBits}, {Extenders});

    re::CC* xfCC = re::makeCC(0x0f);
    re::CC* fxCC = re::makeCC(0xf0);
    for (re::codepoint_t i = 1; i <= 0xf; i++) {
        xfCC = re::makeCC(xfCC, re::makeCC(i * 0x10 + 0x0f));
        fxCC = re::makeCC(fxCC, re::makeCC(0xf0 + i));
    }

    Kernel * CC_0xFXKernel = pxDriver.addKernelInstance<ParabixCharacterClassKernelBuilder>(iBuilder, "CC_0xFX", std::vector<re::CC *>{fxCC}, 8);
    pxDriver.makeKernelCall(CC_0xFXKernel, {BasisBits}, {CC_0xFX});

    Kernel * CC_0xXFKernel = pxDriver.addKernelInstance<ParabixCharacterClassKernelBuilder>(iBuilder, "CC_0xXF", std::vector<re::CC *>{xfCC}, 8);
    pxDriver.makeKernelCall(CC_0xXFKernel, {BasisBits}, {CC_0xXF});


    //// Generate Extract/Deposit Markers, M0_Start, M0_End, MatchOffset

    size_t m0BufferSize = this->getDecompressedBufferBlocks() * 2;
    size_t e1BufferSize = this->getInputBufferBlocks();

    M0_Start = pxDriver.addBuffer<CircularBuffer>(iBuilder, iBuilder->getStreamSetTy(1, 64), this->getInputBufferBlocks());
    M0_End = pxDriver.addBuffer<CircularBuffer>(iBuilder, iBuilder->getStreamSetTy(1, 64), this->getInputBufferBlocks());

    //TODO handle uncompressed part
    StreamSetBuffer * const UncompressedStartPos = pxDriver.addBuffer<CircularBuffer>(iBuilder, iBuilder->getStreamSetTy(1, 64), this->get4MbBufferBlocks());
    StreamSetBuffer * const UncompressedLength = pxDriver.addBuffer<CircularBuffer>(iBuilder, iBuilder->getStreamSetTy(1, 64), this->get4MbBufferBlocks());
    StreamSetBuffer * const UncompressedOutputPos = pxDriver.addBuffer<CircularBuffer>(iBuilder, iBuilder->getStreamSetTy(1, 64), this->get4MbBufferBlocks());

    EMarker = pxDriver.addBuffer<CircularBuffer>(iBuilder, iBuilder->getStreamSetTy(1, 1), e1BufferSize);
    StreamSetBuffer * const M0Marker = pxDriver.addBuffer<CircularBuffer>(iBuilder, iBuilder->getStreamSetTy(1, 1), m0BufferSize);
    DepositMarker = pxDriver.addBuffer<CircularBuffer>(iBuilder, iBuilder->getStreamSetTy(1, 1), m0BufferSize);
    Match_Offset = pxDriver.addBuffer<CircularBuffer>(iBuilder, iBuilder->getStreamSetTy(1, 64), this->getInputBufferBlocks());


    Kernel * blockDecoderK = pxDriver.addKernelInstance<LZ4BlockDecoderNewKernel>(iBuilder);
    blockDecoderK->setInitialArguments({iBuilder->CreateTrunc(hasBlockChecksum, iBuilder->getInt1Ty()), headerSize});
    pxDriver.makeKernelCall(blockDecoderK, {ByteStream, Extenders}, {BlockData_IsCompressed, BlockData_BlockStart, BlockData_BlockEnd});



    Kernel* Lz4IndexBuilderK = pxDriver.addKernelInstance<LZ4IndexBuilderKernel>(iBuilder);
    pxDriver.makeKernelCall(
            Lz4IndexBuilderK,
            {
                    ByteStream,
                    Extenders,
                    CC_0xFX,
                    CC_0xXF,

                    // Block Data
                    BlockData_IsCompressed,
                    BlockData_BlockStart,
                    BlockData_BlockEnd
            }, {
                    //Uncompressed Data
                    UncompressedStartPos,
                    UncompressedLength,
                    UncompressedOutputPos,

                    EMarker,
                    M0_Start,
                    M0_End,
                    Match_Offset
            });


    Kernel * buildM0StartMarkerK = pxDriver.addKernelInstance<LZ4NumbersToBitstreamKernel>("buildM0Marker", iBuilder);
    pxDriver.makeKernelCall(buildM0StartMarkerK, {M0_Start, M0_End}, {M0Marker});


    Kernel * generateDepositK = pxDriver.addKernelInstance<LZ4GenerateDepositStreamKernel>(iBuilder);
    pxDriver.makeKernelCall(generateDepositK, {M0Marker}, {DepositMarker}); // TODO deposit

}