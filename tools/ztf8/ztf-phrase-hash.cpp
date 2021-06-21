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
#include <kernel/streamutils/stream_select.h>
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
static cl::opt<int> WordLen("length", cl::desc("Length of words."), cl::init(2));

typedef void (*ztfHashFunctionType)(uint32_t fd);

EncodingInfo encodingScheme1(8,
                             {{3, 3, 2, 0xC0, 8, 0}, //minLen, maxLen, hashBytes, pfxBase, hashBits, ?
                              {4, 4, 2, 0xC8, 8, 0},
                              {5, 8, 2, 0xD0, 8, 0},
                              {9, 16, 3, 0xE0, 8, 0},
                              {17, 32, 4, 0xF0, 8, 0}
                             });

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

    StreamSet * u8bytes = codeUnitStream;
    std::vector<StreamSet *> extractionMasks;
    for (unsigned iter = WordLen; iter > 0; iter--) {
        StreamSet * WordChars = P->CreateStreamSet(1);
        P->CreateKernelCall<WordMarkKernel>(u8basis, WordChars);

        StreamSet * const phraseRuns = P->CreateStreamSet(1);
        StreamSet * const codewordRuns = P->CreateStreamSet(1);
        P->CreateKernelCall<ZTF_Phrases>(u8basis, WordChars, phraseRuns);
        //codewordRuns
        //P->CreateKernelCall<DebugDisplayKernel>("phraseRuns", phraseRuns);
        //P->CreateKernelCall<DebugDisplayKernel>("codewordRuns", codewordRuns);

        std::vector<StreamSet *> allPhraseRuns;
        std::vector<StreamSet *> allRunIndex;
        std::vector<StreamSet *> allOverflow;
        std::vector<StreamSet *> allHashValues;
        allPhraseRuns.reserve(iter);
        allRunIndex.reserve(iter);
        allOverflow.reserve(iter);
        allHashValues.reserve(iter);
        // analyze all the phrase run sequence with current length of phrase
        for (unsigned i = 0; i < iter; i++) {
            StreamSet * const phraseSeq = P->CreateStreamSet(1);
            // use compSymSeq to aviod a sequence running across codewords
            P->CreateKernelCall<PhraseRunSeq>(phraseRuns, phraseSeq, iter, i);
            //P->CreateKernelCall<DebugDisplayKernel>("phraseSeq", phraseSeq);
            allPhraseRuns.push_back(phraseSeq);
            StreamSet * const runIndex = P->CreateStreamSet(5);
            StreamSet * const overflow = P->CreateStreamSet(1);
            P->CreateKernelCall<RunIndex>(phraseSeq, runIndex, overflow);
            allRunIndex.push_back(runIndex);
            allOverflow.push_back(overflow);
            //P->CreateKernelCall<DebugDisplayKernel>("runIndex", runIndex);
            StreamSet * const bixHashes = P->CreateStreamSet(encodingScheme1.MAX_HASH_BITS);
            P->CreateKernelCall<BixHash>(u8basis, phraseSeq, bixHashes);
            //P->CreateKernelCall<DebugDisplayKernel>("bixHashes", bixHashes);
            std::vector<StreamSet *> combinedHashData = {bixHashes, runIndex};
            StreamSet * const hashValues = P->CreateStreamSet(1, 16);
            P->CreateKernelCall<P2S16Kernel>(combinedHashData, hashValues);
            allHashValues.push_back(hashValues);
            //P->CreateKernelCall<DebugDisplayKernel>("hashValues", hashValues);
        }

        extractionMasks.reserve(encodingScheme1.byLength.size());
        for (unsigned i = 0; i < encodingScheme1.byLength.size(); i++) {
            std::vector<StreamSet *> allGroupMarks;
            // get the groupMarks streams marking all phrases of length l such that groupInfo.lo < l < groupInfo.hi
            // for each of the q phraseSeq streams
            for ( unsigned ii = 0; ii < iter; ii++) {
                StreamSet * groupMarks = P->CreateStreamSet(1);
                P->CreateKernelCall<LengthGroupSelector>(encodingScheme1, i, allPhraseRuns[ii], allRunIndex[ii], allOverflow[ii], groupMarks);
                //P->CreateKernelCall<DebugDisplayKernel>("groupMarks", groupMarks);
                allGroupMarks.push_back(groupMarks);
            }
            StreamSet * extractionMask = P->CreateStreamSet(1);
            StreamSet * compSymSeq = P->CreateStreamSet(1);
            StreamSet * input_bytes = u8bytes;
            StreamSet * output_bytes = P->CreateStreamSet(1, 8);

            // TODO: inlcude length based phrase compression
            P->CreateKernelCall<PhraseCompression>(encodingScheme1, i, allGroupMarks, allHashValues, input_bytes,  extractionMask, output_bytes, compSymSeq);
            //P->CreateKernelCall<DebugDisplayKernel>("extractionMask", extractionMask);
            //P->CreateKernelCall<DebugDisplayKernel>("compSymSeq", compSymSeq);
            extractionMasks.push_back(extractionMask);
            u8bytes = output_bytes;
        }
    }

    StreamSet * const combinedMask = P->CreateStreamSet(1);
    P->CreateKernelCall<StreamsIntersect>(extractionMasks, combinedMask);
    StreamSet * const encoded = P->CreateStreamSet(8);
    P->CreateKernelCall<S2PKernel>(u8bytes, encoded);

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

    return reinterpret_cast<ztfHashFunctionType>(P->compile());
}

int main(int argc, char *argv[]) {
    codegen::ParseCommandLineOptions(argc, argv, {&ztfHashOptions, pablo_toolchain_flags(), codegen::codegen_flags()});

    CPUDriver pxDriver("ztfPhraseHash");
    const int fd = open(inputFile.c_str(), O_RDONLY);
    if (LLVM_UNLIKELY(fd == -1)) {
        errs() << "Error: cannot open " << inputFile << " for processing. Skipped.\n";
    } else {
        if (Decompression) {
            errs() << "Coming soon!" << "\n";
            //auto ztfHashDecompressionFunction = ztfHash_decompression_gen(pxDriver);
            //ztfHashDecompressionFunction(fd);
        } else {
            auto ztfHashCompressionFunction = ztfHash_compression_gen(pxDriver);
            ztfHashCompressionFunction(fd);
        }
        close(fd);
    }
    return 0;
}
