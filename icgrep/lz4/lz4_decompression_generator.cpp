
#include "lz4_decompression_generator.h"
#include "lz4_frame_decoder.h"

#include <boost/filesystem.hpp>
#include <boost/iostreams/device/mapped_file.hpp>
#include <kernels/kernel_builder.h>
#include <kernels/p2s_kernel.h>
#include <kernels/stdout_kernel.h>


#include <llvm/Support/raw_ostream.h>

namespace re { class CC; }

using namespace llvm;
using namespace parabix;
using namespace kernel;

LZ4DecompressionGenerator::LZ4DecompressionGenerator():LZ4BaseGenerator() {
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
    char *fileBuffer = const_cast<char *>(mappedFile.data());

    this->generateDecompressionPipeline(outputFileName);

    auto main = this->getMainFunc();
    main(fileBuffer, lz4Frame.getBlocksStart(), lz4Frame.getBlocksStart() + lz4Frame.getBlocksLength(), lz4Frame.hasBlockChecksum());
    mappedFile.close();
    return 0;
}


MainFunctionType LZ4DecompressionGenerator::getMainFunc() {
    return reinterpret_cast<MainFunctionType>(mPxDriver.getMain());
}

void LZ4DecompressionGenerator::generateDecompressionPipeline(const std::string &outputFile) {
    auto & b = mPxDriver.getBuilder();
    this->generateMainFunc(b);



    StreamSetBuffer* compressedByteStream = this->loadByteStream();
    StreamSetBuffer* uncompressedByteStream = this->byteStreamDecompression(compressedByteStream);

    Kernel * outK = mPxDriver.addKernelInstance<FileSink>(b, 8);
    outK->setInitialArguments({b->GetString(outputFile)});
    mPxDriver.makeKernelCall(outK, {uncompressedByteStream}, {});

    mPxDriver.generatePipelineIR();
    mPxDriver.deallocateBuffers();

    b->CreateRetVoid();

    mPxDriver.finalizeObject();
}


void LZ4DecompressionGenerator::generateMainFunc(const std::unique_ptr<kernel::KernelBuilder> & iBuilder) {
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
