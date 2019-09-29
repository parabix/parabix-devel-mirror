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
#include <re/adt/re_name.h>
#include <re/adt/re_re.h>
#include <grep/grep_kernel.h>
#include <re/cc/cc_compiler.h>
#include <re/cc/cc_compiler_target.h>
#include <re/ucd/ucd_compiler.hpp>
#include <re/unicode/resolve_properties.h>
#include <re/unicode/re_name_resolve.h>
#include <pablo/bixnum/bixnum.h>
#include <kernel/core/kernel_builder.h>
#include <pablo/pe_zeroes.h>
#include <toolchain/toolchain.h>
#include <kernel/pipeline/driver/cpudriver.h>
#include <kernel/core/streamset.h>
#include <kernel/streamutils/run_index.h>
#include <kernel/streamutils/streams_merge.h>
#include <kernel/util/bixhash.h>
#include <kernel/util/debug_display.h>
#include <llvm/ADT/StringRef.h>
#include <llvm/Support/raw_ostream.h>
#include <pablo/builder.hpp>
#include <pablo/pe_ones.h>
#include <fcntl.h>
#include <iostream>
#include <iomanip>
#include <kernel/pipeline/pipeline_builder.h>
#include "ztf-logic.h"
#include "ztf-scan.h"

using namespace pablo;
using namespace parse;
using namespace kernel;
using namespace llvm;
using namespace codegen;

static cl::OptionCategory ztfHashOptions("ztfHash Options", "ZTF-Hash options.");
static cl::opt<std::string> inputFile(cl::Positional, cl::desc("<input file>"), cl::Required, cl::cat(ztfHashOptions));
static cl::opt<bool> Decompression("d", cl::desc("Decompress from ZTF-Runs to UTF-8."), cl::cat(ztfHashOptions), cl::init(false));
static cl::alias DecompressionAlias("decompress", cl::desc("Alias for -d"), cl::aliasopt(Decompression));

typedef void (*ztfHashFunctionType)(uint32_t fd);

std::vector<LengthGroup> lenGroups = {LengthGroup{3, 4, 8}, LengthGroup{5, 8, 8}, LengthGroup{9, 16, 8}};

ztfHashFunctionType ztfHash_compression_gen (CPUDriver & driver) {

    auto & b = driver.getBuilder();
    Type * const int32Ty = b->getInt32Ty();
    auto P = driver.makePipeline({Binding{int32Ty, "fd"}});

    Scalar * const fileDescriptor = P->getInputScalar("fd");

    // Source data
    StreamSet * const codeUnitStream = P->CreateStreamSet(1, 8);
    P->CreateKernelCall<MMapSourceKernel>(fileDescriptor, codeUnitStream);

    StreamSet * const u8basis = P->CreateStreamSet(8);
    P->CreateKernelCall<S2PKernel>(codeUnitStream, u8basis);
    
    StreamSet * WordChars = P->CreateStreamSet(1);
    P->CreateKernelCall<WordMarkKernel>(u8basis, WordChars);
    StreamSet * const symbolRuns = P->CreateStreamSet(1);
    P->CreateKernelCall<ZTF_Symbols>(u8basis, WordChars, symbolRuns);

    StreamSet * const runIndex = P->CreateStreamSet(4);
    StreamSet * const overflow = P->CreateStreamSet(1);
    P->CreateKernelCall<RunIndex>(symbolRuns, runIndex, overflow);

    StreamSet * const bixHashes = P->CreateStreamSet(8);
    P->CreateKernelCall<BixHash>(u8basis, symbolRuns, bixHashes);
    
    StreamSet * const hashValues = P->CreateStreamSet(1, 16);
    std::vector<StreamSet *> combinedHashData = {bixHashes, runIndex};
    P->CreateKernelCall<P2S16Kernel>(combinedHashData, hashValues);

    std::vector<StreamSet *> parsedMarks = {P->CreateStreamSet(1), P->CreateStreamSet(1), P->CreateStreamSet(1)};
    P->CreateKernelCall<LengthSorter>(symbolRuns, runIndex, overflow, lenGroups, parsedMarks);

    std::vector<StreamSet *> extractionMasks = {P->CreateStreamSet(1), P->CreateStreamSet(1), P->CreateStreamSet(1)};
    for (unsigned i = 0; i < lenGroups.size(); i++) {
        P->CreateKernelCall<LengthGroupCompressionMask>(lenGroups[i], parsedMarks[i], hashValues, codeUnitStream,  extractionMasks[i]);
    }

    StreamSet * const combinedMask = P->CreateStreamSet(1);
    P->CreateKernelCall<StreamsIntersect>(extractionMasks, combinedMask);
    StreamSet * const encoded = P->CreateStreamSet(8);
    P->CreateKernelCall<ZTF_SymbolEncoder>(lenGroups, u8basis, bixHashes, combinedMask, runIndex, encoded);
    
    StreamSet * const ZTF_basis = P->CreateStreamSet(8);
    FilterByMask(P, combinedMask, encoded, ZTF_basis);

    StreamSet * const ZTF_bytes = P->CreateStreamSet(1, 8);
    P->CreateKernelCall<P2SKernel>(ZTF_basis, ZTF_bytes);
    P->CreateKernelCall<StdOutKernel>(ZTF_bytes);
    return reinterpret_cast<ztfHashFunctionType>(P->compile());
}

ztfHashFunctionType ztfHash_decompression_gen (CPUDriver & driver) {
    auto & b = driver.getBuilder();
    Type * const int32Ty = b->getInt32Ty();
    auto P = driver.makePipeline({Binding{int32Ty, "fd"}});
    Scalar * const fileDescriptor = P->getInputScalar("fd");

    // Source data
    StreamSet * const source = P->CreateStreamSet(1, 8);
    P->CreateKernelCall<MMapSourceKernel>(fileDescriptor, source);
    StreamSet * const ztfHashBasis = P->CreateStreamSet(8);
    P->CreateKernelCall<S2PKernel>(source, ztfHashBasis);
    StreamSet * const ztfInsertionLengths = P->CreateStreamSet(4);
    P->CreateKernelCall<ZTF_ExpansionDecoder>(ztfHashBasis, lenGroups, ztfInsertionLengths);
    StreamSet * const ztfRunSpreadMask = InsertionSpreadMask(P, ztfInsertionLengths);

    StreamSet * const ztfHash_u8_Basis = P->CreateStreamSet(8);
    SpreadByMask(P, ztfRunSpreadMask, ztfHashBasis, ztfHash_u8_Basis);

    std::vector<StreamSet *> decodedMarks = {P->CreateStreamSet(1), P->CreateStreamSet(1), P->CreateStreamSet(1)};
    P->CreateKernelCall<ZTF_DecodeLengths>(ztfHash_u8_Basis,
                                           lenGroups,
                                           decodedMarks);

    StreamSet * WordChars = P->CreateStreamSet(1);
    P->CreateKernelCall<WordMarkKernel>(ztfHash_u8_Basis, WordChars);

    StreamSet * const ztfHash_u8bytes = P->CreateStreamSet(1, 8);
    P->CreateKernelCall<P2SKernel>(ztfHash_u8_Basis, ztfHash_u8bytes);
    
    StreamSet * const symbolRuns = P->CreateStreamSet(1);
    P->CreateKernelCall<ZTF_Symbols>(ztfHash_u8_Basis, WordChars, symbolRuns);

    StreamSet * const runIndex = P->CreateStreamSet(4);
    StreamSet * const overflow = P->CreateStreamSet(1);
    P->CreateKernelCall<RunIndex>(symbolRuns, runIndex, overflow);

    StreamSet * const bixHashes = P->CreateStreamSet(8);
    P->CreateKernelCall<BixHash>(ztfHash_u8_Basis, symbolRuns, bixHashes);
    
    StreamSet * const hashValues = P->CreateStreamSet(1, 16);
    std::vector<StreamSet *> combinedHashData = {bixHashes, runIndex};
    P->CreateKernelCall<P2S16Kernel>(combinedHashData, hashValues);
    
    std::vector<StreamSet *> parsedMarks = {P->CreateStreamSet(1), P->CreateStreamSet(1), P->CreateStreamSet(1)};
    P->CreateKernelCall<LengthSorter>(symbolRuns, runIndex, overflow, lenGroups, parsedMarks);

    StreamSet * u8bytes = ztfHash_u8bytes;
    for (unsigned i = 0; i < lenGroups.size(); i++) {
        StreamSet * input_bytes = u8bytes;
        StreamSet * output_bytes = P->CreateStreamSet(1, 8);
        P->CreateKernelCall<LengthGroupDecompression>(lenGroups[i], parsedMarks[i], hashValues, decodedMarks[i], input_bytes, output_bytes);
        u8bytes = output_bytes;
    }

    P->CreateKernelCall<StdOutKernel>(u8bytes);
    return reinterpret_cast<ztfHashFunctionType>(P->compile());
}

int main(int argc, char *argv[]) {
    codegen::ParseCommandLineOptions(argc, argv, {&ztfHashOptions, pablo_toolchain_flags(), codegen::codegen_flags()});
    
    CPUDriver pxDriver("ztfHash");
    const int fd = open(inputFile.c_str(), O_RDONLY);
    if (LLVM_UNLIKELY(fd == -1)) {
        errs() << "Error: cannot open " << inputFile << " for processing. Skipped.\n";
    } else {
        if (Decompression) {
            auto ztfHashDecompressionFunction = ztfHash_decompression_gen(pxDriver);
            ztfHashDecompressionFunction(fd);
        } else {
            auto ztfHashCompressionFunction = ztfHash_compression_gen(pxDriver);
            ztfHashCompressionFunction(fd);
        }
        close(fd);
    }
    return 0;
}
