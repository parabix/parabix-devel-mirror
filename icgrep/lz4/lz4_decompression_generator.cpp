
#include "lz4_decompression_generator.h"
#include "lz4_frame_decoder.h"

#include <boost/filesystem.hpp>
#include <boost/iostreams/device/mapped_file.hpp>
#include <kernels/kernel_builder.h>
#include <kernels/stdout_kernel.h>
#include <kernels/pipeline_builder.h>
#include <llvm/Support/raw_ostream.h>

namespace re { class CC; }

using namespace llvm;
using namespace kernel;

LZ4DecompressionGenerator::LZ4DecompressionGenerator()
: LZ4BaseGenerator() {
    mPipeline = makeInternalPipeline();
}

inline std::unique_ptr<kernel::PipelineBuilder> LZ4DecompressionGenerator::makeInternalPipeline() {
    Bindings inputs;

    auto & b = mPxDriver.getBuilder();

    Type * const int8PtrTy = b->getInt8PtrTy();
    Type * const sizeTy = b->getSizeTy();
    Type * const boolTy = b->getIntNTy(sizeof(bool) * 8);

    inputs.emplace_back(int8PtrTy, "input");
    inputs.emplace_back(sizeTy, "headerSize");
    inputs.emplace_back(sizeTy, "fileSize");
    inputs.emplace_back(boolTy, "hasBlockChecksum");
    inputs.emplace_back(int8PtrTy, "outputFile");

    return mPxDriver.makePipeline(inputs, Bindings{});
}

int LZ4DecompressionGenerator::decompress(std::string&& inputFileName, std::string&& outputFileName, bool overwriteOutput) {
    LZ4FrameDecoder lz4Frame(inputFileName);
    if (!lz4Frame.isValid()) {
        llvm::errs() << "Invalid LZ4 file.\n";
        return -1;
    }

    if (boost::filesystem::exists(outputFileName)) {
        if (overwriteOutput) {
            boost::filesystem::remove(outputFileName);
        } else {
            llvm::errs() << outputFileName + " existed. Use -f argument to overwrite.\n";
            return -1;
        }
    }

    boost::iostreams::mapped_file_source mappedFile;
    mappedFile.open(inputFileName, lz4Frame.getBlocksLength() + lz4Frame.getBlocksStart());
    char * fileBuffer = const_cast<char *>(mappedFile.data());

    auto main = generateDecompressionPipeline();

    main(fileBuffer, lz4Frame.getBlocksStart(), lz4Frame.getBlocksStart() + lz4Frame.getBlocksLength(), lz4Frame.hasBlockChecksum(), outputFileName.c_str());
    mappedFile.close();
    return 0;
}

MainFunctionType LZ4DecompressionGenerator::generateDecompressionPipeline() {
    StreamSet * compressedByteStream = loadByteStream();
    StreamSet * uncompressedByteStream = byteStreamDecompression(compressedByteStream);
    Scalar * outputFileName = mPipeline->getInputScalar("outputFile");
    mPipeline->CreateKernelCall<FileSink>(outputFileName, uncompressedByteStream);
    return reinterpret_cast<MainFunctionType>(mPipeline->compile());
}
