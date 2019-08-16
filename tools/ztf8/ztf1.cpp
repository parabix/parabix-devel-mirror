/*
 *  Copyright (c) 2019 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#include <kernel/streamutils/deletion.h>                      // for DeletionKernel
#include <kernel/io/source_kernel.h>
#include <kernel/basis/p2s_kernel.h>
#include <kernel/basis/s2p_kernel.h>                    // for S2PKernel
#include <kernel/io/stdout_kernel.h>                 // for StdOutKernel_
#include <kernel/streamutils/pdep_kernel.h>
#include <llvm/IR/Function.h>                      // for Function, Function...
#include <llvm/IR/Module.h>                        // for Module
#include <llvm/Support/CommandLine.h>              // for ParseCommandLineOp...
#include <llvm/Support/Debug.h>                    // for dbgs
#include <pablo/pablo_kernel.h>                    // for PabloKernel
#include <toolchain/pablo_toolchain.h>
#include <pablo/parse/pablo_source_kernel.h>
#include <pablo/parse/pablo_parser.h>
#include <pablo/parse/simple_lexer.h>
#include <pablo/parse/rd_parser.h>
#include <kernel/core/kernel_builder.h>
#include <pablo/pe_zeroes.h>
#include <toolchain/toolchain.h>
#include <kernel/pipeline/driver/cpudriver.h>
#include <kernel/core/streamset.h>
#include <kernel/streamutils/streams_merge.h>
#include <llvm/ADT/StringRef.h>
#include <llvm/Support/raw_ostream.h>
#include <pablo/builder.hpp>
#include <fcntl.h>
#include <iostream>
#include <iomanip>
#include <kernel/pipeline/pipeline_builder.h>

using namespace pablo;
using namespace pablo::parse;
using namespace kernel;
using namespace llvm;
using namespace codegen;

static cl::OptionCategory ztf1Options("ztf1 Options", "ztf1 options.");
static cl::opt<std::string> inputFile(cl::Positional, cl::desc("<input file>"), cl::Required, cl::cat(ztf1Options));
static cl::opt<bool> Decompression("d", cl::desc("Decompress from ZTF-1 to UTF08."), cl::cat(ztf1Options), cl::init(false));
static cl::alias DecompressionAlias("decompress", cl::desc("Alias for -d"), cl::aliasopt(Decompression));

typedef void (*ztf1FunctionType)(uint32_t fd);

ztf1FunctionType ztf1_compression_gen (CPUDriver & driver, std::shared_ptr<PabloParser> parser, std::shared_ptr<SourceFile> ztf1PabloSrc) {

    auto & b = driver.getBuilder();
    Type * const int32Ty = b->getInt32Ty();
    auto P = driver.makePipeline({Binding{int32Ty, "fd"}});

    Scalar * const fileDescriptor = P->getInputScalar("fd");

    // Source data
    StreamSet * const codeUnitStream = P->CreateStreamSet(1, 8);
    P->CreateKernelCall<MMapSourceKernel>(fileDescriptor, codeUnitStream);

    StreamSet * const u8basis = P->CreateStreamSet(8);
    P->CreateKernelCall<S2PKernel>(codeUnitStream, u8basis);
    
    StreamSet * const compression_mask = P->CreateStreamSet(1);
    P->CreateKernelCall<PabloSourceKernel>(
           parser,
           ztf1PabloSrc,
           "ZTF1_Compression_Mask",
           Bindings { // Input Stream Bindings
               Binding {"bb", u8basis}
           },
           Bindings { // Output Stream Bindings
               Binding {"mask", compression_mask}
           }
        );

    StreamSet * const ztf_basis = P->CreateStreamSet(8);
    FilterByMask(P, compression_mask, u8basis, ztf_basis);
    StreamSet * const ZTF_bytes = P->CreateStreamSet(1, 8);
    P->CreateKernelCall<P2SKernel>(ztf_basis, ZTF_bytes);
    P->CreateKernelCall<StdOutKernel>(ZTF_bytes);
    return reinterpret_cast<ztf1FunctionType>(P->compile());
}

ztf1FunctionType ztf1_decompression_gen (CPUDriver & driver, std::shared_ptr<PabloParser> parser, std::shared_ptr<SourceFile> ztf1PabloSrc) {
    auto & b = driver.getBuilder();
    Type * const int32Ty = b->getInt32Ty();
    auto P = driver.makePipeline({Binding{int32Ty, "fd"}});
    Scalar * const fileDescriptor = P->getInputScalar("fd");

    // Source data
    StreamSet * const ztf1stream = P->CreateStreamSet(1, 8);
    P->CreateKernelCall<MMapSourceKernel>(fileDescriptor, ztf1stream);
    StreamSet * const ztf1basis = P->CreateStreamSet(8);
    P->CreateKernelCall<S2PKernel>(ztf1stream, ztf1basis);

    StreamSet * const insertion_mask = P->CreateStreamSet(1);
    P->CreateKernelCall<PabloSourceKernel>(
           parser,
           ztf1PabloSrc,
           "ZTF1_InsertionMask",
           Bindings { // Input Stream Bindings
               Binding {"bb", ztf1basis}
           },
           Bindings { // Output Stream Bindings
               Binding {"insert_marks", insertion_mask}
           }
        );
    StreamSet * const ztf1_to_utf8_spread_mask = UnitInsertionSpreadMask(P, insertion_mask);
    StreamSet * const ztf1_basis_utf8_indexed = P->CreateStreamSet(8);
    SpreadByMask(P, ztf1_to_utf8_spread_mask, ztf1basis, ztf1_basis_utf8_indexed);
    StreamSet * const u8basis = P->CreateStreamSet(8);
    P->CreateKernelCall<PabloSourceKernel>(
           parser,
           ztf1PabloSrc,
           "ZTF1_Decompression",
           Bindings { // Input Stream Bindings
               Binding {"ztf1_u8_indexed", ztf1_basis_utf8_indexed},
               Binding {"spread_mask", ztf1_to_utf8_spread_mask}
           },
           Bindings { // Output Stream Bindings
               Binding {"u8_basis", u8basis}
           }
        );
    StreamSet * const u8bytes = P->CreateStreamSet(1, 8);
    P->CreateKernelCall<P2SKernel>(u8basis, u8bytes);
    P->CreateKernelCall<StdOutKernel>(u8bytes);
    return reinterpret_cast<ztf1FunctionType>(P->compile());
}

int main(int argc, char *argv[]) {
    codegen::ParseCommandLineOptions(argc, argv, {&ztf1Options, pablo::pablo_toolchain_flags(), codegen::codegen_flags()});
    
    CPUDriver pxDriver("ztf1");
    auto em = ErrorManager::Create();
    auto parser = RecursiveParser::Create(SimpleLexer::Create(em), em);
    auto ztf1Source = SourceFile::Relative("ztf1.pablo");
    if (ztf1Source == nullptr) {
        std::cerr << "pablo-parser: error loading pablo source file: ztf1.pablo\n";
    }
    const int fd = open(inputFile.c_str(), O_RDONLY);
    if (LLVM_UNLIKELY(fd == -1)) {
        errs() << "Error: cannot open " << inputFile << " for processing. Skipped.\n";
    } else {
        if (Decompression) {
            auto ztf1DecompressionFunction = ztf1_decompression_gen(pxDriver, parser, ztf1Source);
            ztf1DecompressionFunction(fd);
        } else {
            auto ztf1CompressionFunction = ztf1_compression_gen(pxDriver, parser, ztf1Source);
            ztf1CompressionFunction(fd);
        }
        close(fd);
    }
    return 0;
}
