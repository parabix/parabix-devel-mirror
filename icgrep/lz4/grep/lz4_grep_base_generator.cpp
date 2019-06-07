
#include "lz4_grep_base_generator.h"

#include <boost/iostreams/device/mapped_file.hpp>

#include <llvm/Support/PrettyStackTrace.h>

#include <cc/cc_compiler.h>

#include <cc/cc_kernel.h>
#include <kernels/s2p_kernel.h>
#include <kernels/p2s_kernel.h>
#include <kernels/source_kernel.h>
#include <kernels/stdout_kernel.h>
#include <kernels/fake_stream_generating_kernel.h>
#include <re/re_toolchain.h>

#include <re/collect_ccs.h>
#include <re/replaceCC.h>

#include <re/casing.h>
#include <re/exclude_CC.h>
#include <re/to_utf8.h>
#include <re/re_analysis.h>
#include <re/re_name.h>
#include <re/re_name_resolve.h>
#include <re/re_name_gather.h>
#include <re/re_multiplex.h>
#include <re/re_utility.h>

#include <UCD/resolve_properties.h>
#include <kernels/charclasses.h>
#include <grep/grep_kernel.h>
#include <kernels/UCD_property_kernel.h>
#include <kernels/grapheme_kernel.h>
#include <kernels/linebreak_kernel.h>
#include <kernels/streams_merge.h>
#include <kernels/scanmatchgen.h>
#include <kernels/until_n.h>
#include <re/grapheme_clusters.h>
#include <re/printer_re.h>
#include <llvm/Support/raw_ostream.h>
#include <llvm/Support/Debug.h>
#include <kernels/lz4/lz4_block_decoder.h>
#include <kernels/lz4/lz4_match_detector.h>

#include <re/re_seq.h>
#include <kernels/kernel_builder.h>
#include <kernels/pipeline_builder.h>
#include <re/re_alt.h>
#include <kernels/lz4/decompression/lz4_bytestream_decompression.h>
#include <kernels/lz4/lz4_not_kernel.h>

namespace re { class CC; }

using namespace llvm;
using namespace kernel;
using namespace grep;
using namespace re;
using namespace cc;

inline RE * makeNonFinal() {
    CC * const C2_F4 = makeByte(0xC2, 0xF4);
    CC * const E0_F4 = makeByte(0xE0, 0xF4);
    CC * const _80_F4 = makeByte(0x80, 0xBF);
    RE * E0_F4x80_F4 = makeSeq({E0_F4, _80_F4});
    CC * const F0_F4 = makeByte(0xE0, 0xF4);
    RE * F0_F4x80_F4x80_F4 = makeSeq({F0_F4, _80_F4, _80_F4});
    return makeAlt({C2_F4, E0_F4x80_F4, F0_F4x80_F4x80_F4});
}

LZ4GrepBaseGenerator::LZ4GrepBaseGenerator(const FunctionType type)
: LZ4BaseGenerator()
, u8NonFinalRe(makeNonFinal())
, u8FinalRe(makeCC(0x0, 0x10FFFF))
, mMainMethod(nullptr) {
    mGrepRecordBreak = grep::GrepRecordBreakKind::LF;
    mMoveMatchesToEOL = true;
    mPipeline = makeInternalPipeline(type);
}

inline std::unique_ptr<kernel::ProgramBuilder> LZ4GrepBaseGenerator::makeInternalPipeline(const FunctionType type) {
    Bindings inputs;
    Bindings outputs;

    auto & b = mPxDriver.getBuilder();

    Type * const inputType = b->getInt8PtrTy();
    Type * const sizeTy = b->getSizeTy();
    Type * const boolTy = b->getIntNTy(sizeof(bool) * 8);

    inputs.emplace_back(inputType, "input");
    inputs.emplace_back(sizeTy, "headerSize");
    inputs.emplace_back(sizeTy, "fileSize");
    inputs.emplace_back(boolTy, "hasBlockChecksum");

    if (type == FunctionType::CountOnly) {
        outputs.emplace_back(sizeTy, "countResult");
    } else if (type == FunctionType::Match) {
        Type * const intAddrTy = b->getIntAddrTy();
        inputs.emplace_back(intAddrTy, "match_accumulator");
    }

    return mPxDriver.makePipeline(inputs, outputs);
}

void LZ4GrepBaseGenerator::generateScanMatchGrepPipeline(RE* regex, bool enableMultiplexing, bool utf8CC) {
    if (enableMultiplexing) {
        generateMultiplexingScanMatchGrepPipeline(regex, utf8CC);
    } else {
        generateFullyDecompressionScanMatchGrepPipeline(regex);
    }
}

void LZ4GrepBaseGenerator::generateCountOnlyGrepPipeline(RE* regex, bool enableMultiplexing, bool utf8CC){
    if (enableMultiplexing) {
        generateMultiplexingCountOnlyGrepPipeline(regex, utf8CC);
    } else {
        generateFullyDecompressionCountOnlyGrepPipeline(regex);
    }
}

void LZ4GrepBaseGenerator::initREs(RE * re) {
    if (mGrepRecordBreak == GrepRecordBreakKind::Unicode) {
        mBreakCC = makeCC(makeCC(0x0A, 0x0D), makeCC(makeCC(0x85), makeCC(0x2028, 0x2029)));
    } else if (mGrepRecordBreak == GrepRecordBreakKind::Null) {
        mBreakCC = makeByte(0);  // Null
    } else {
        mBreakCC = makeByte(0x0A); // LF
    }
    RE * anchorRE = mBreakCC;
    if (mGrepRecordBreak == GrepRecordBreakKind::Unicode) {
        Name * anchorName = makeName("UTF8_LB", Name::Type::Unicode);
        anchorName->setDefinition(makeUnicodeBreak());
        anchorRE = anchorName;
    }

    mRE = re;
    bool allAnchored = true;

    if (!hasEndAnchor(mRE)) allAnchored = false;
    mRE = resolveModesAndExternalSymbols(mRE);
    mRE = exclude_CC(mRE, mBreakCC);
    mRE = resolveAnchors(mRE, anchorRE);
    gatherUnicodeProperties(mRE, mUnicodeProperties);
    mRE = regular_expression_passes(mRE);

    if (allAnchored && (mGrepRecordBreak != GrepRecordBreakKind::Unicode)) mMoveMatchesToEOL = false;

}

StreamSet * LZ4GrepBaseGenerator::linefeedStreamFromUncompressedBits(StreamSet *uncompressedBasisBits) {
    StreamSet * const LineFeedStream = mPipeline->CreateStreamSet(1, 1);
    mPipeline->CreateKernelCall<LineFeedKernelBuilder>(uncompressedBasisBits, LineFeedStream);
    return LineFeedStream;
}

unsigned LZ4GrepBaseGenerator::calculateTwistWidth(unsigned numOfStreams) {
    if (numOfStreams <= 2) {
        return numOfStreams;
    } else if (numOfStreams <= 4) {
        return 4;
    } else if (numOfStreams <= 8) {
        return 8;
    } else {
        llvm::report_fatal_error("Twist: Unsupported numOfStreams " + std::to_string(numOfStreams));;
    }
}

std::pair<StreamSet *, StreamSet *> LZ4GrepBaseGenerator::multiplexingGrep(RE * re, StreamSet * compressedByteStream, StreamSet * compressedBitStream, bool utf8CC) {

    initREs(re);

    //  Regular Expression Processing and Analysis Phase

    StreamSet * fakeMatchCopiedBits = nullptr;
    StreamSet * u8NoFinalStream = nullptr;
    StreamSet * uncompressedCharClasses = nullptr;

    CC * const linefeedCC = makeCC(0x0A);

    std::shared_ptr<MultiplexedAlphabet> mpx;

    if (utf8CC) {

        const auto requireNonFinal = isRequireNonFinal(mRE);
        Seq * const seq = cast<Seq>(makeSeq({mRE, linefeedCC}));

        auto UnicodeSets = collectCCs(seq, Unicode, std::set<Name *>({makeZeroWidth("\\b{g}")}));;

        mpx = std::make_shared<MultiplexedAlphabet>("mpx", UnicodeSets);

        auto mpxCCs = mpx->getMultiplexedCCs();

        bool mpxContainFinal = false;

        seq->push_back(u8FinalRe);

        auto UnicodeSetsWithU8Final = collectCCs(seq, Unicode, std::set<Name *>({makeZeroWidth("\\b{g}")}));;
        auto u8FinalMpx = std::make_shared<MultiplexedAlphabet>("mpx", UnicodeSetsWithU8Final);
        auto mpxCCsWithU8Final = u8FinalMpx->getMultiplexedCCs();

        if (calculateTwistWidth(mpxCCs.size() + 1) > calculateTwistWidth(mpxCCsWithU8Final.size())) {
            mpxContainFinal = true;
            UnicodeSets = UnicodeSetsWithU8Final;
            mpx = u8FinalMpx;
            mpxCCs = mpxCCsWithU8Final;
        }

        mRE = transformCCs(mpx, mRE);

        StreamSet * CharClasses = mPipeline->CreateStreamSet(mpxCCs.size());

        mPipeline->CreateKernelCall<CharClassesKernel>(std::move(mpxCCs), compressedBitStream, CharClasses);

        if (!requireNonFinal) {
            // We do not need to decompress U8 NonFinal Stream is all of the character class in target regular expression is byte length
            uncompressedCharClasses = decompressBitStream(compressedByteStream, CharClasses);
            auto fakeStreams = generateFakeStreams(uncompressedCharClasses, std::vector<unsigned>{8, 1});
            fakeMatchCopiedBits = fakeStreams[0];
            u8NoFinalStream = fakeStreams[1];
        } else {
            if (mpxContainFinal) {
                auto decompressedStreams = decompressBitStreams(compressedByteStream, {CharClasses/*, compressedNonFinalStream*/});
                uncompressedCharClasses = decompressedStreams[0];
                auto fakeStreams = generateFakeStreams(uncompressedCharClasses, std::vector<unsigned>{8});
                fakeMatchCopiedBits = fakeStreams[0];
                StreamSet * u8FinalStream = mPipeline->CreateStreamSet();
                RE * const mpxU8FinalRe = transformCCs(mpx, u8FinalRe);

                std::unique_ptr<GrepKernelOptions> options = make_unique<GrepKernelOptions>();
                options->addAlphabet(mpx, uncompressedCharClasses);
                options->setSource(fakeMatchCopiedBits);
                options->setRE(mpxU8FinalRe);
                options->setResults(u8FinalStream);
                mPipeline->CreateKernelCall<ICGrepKernel>(std::move(options));

                u8NoFinalStream = mPipeline->CreateStreamSet(1, 1);
                mPipeline->CreateKernelCall<LZ4NotKernel>(u8FinalStream, u8NoFinalStream);
            } else {
                StreamSet * compressedNonFinalStream = mPipeline->CreateStreamSet(1, 1);

                std::unique_ptr<GrepKernelOptions> options = make_unique<GrepKernelOptions>();
                options->setSource(compressedBitStream);
                options->setRE(u8NonFinalRe);
                options->setResults(compressedNonFinalStream);
                mPipeline->CreateKernelCall<ICGrepKernel>(std::move(options));

                auto decompressedStreams = decompressBitStreams(compressedByteStream, {CharClasses, compressedNonFinalStream});
                uncompressedCharClasses = decompressedStreams[0];
                u8NoFinalStream = decompressedStreams[1];
                auto fakeStreams = generateFakeStreams(uncompressedCharClasses, std::vector<unsigned>{8});
                fakeMatchCopiedBits = fakeStreams[0];
            }
        }

    } else { // if (!utf8CC) {

        RE * const targetRe = toUTF8(mRE, true);
        Seq * const seq = cast<Seq>(makeSeq({targetRe, linefeedCC}));
        auto UnicodeSets = collectCCs(seq, Byte, std::set<Name *>({makeZeroWidth("\\b{g}")}));

        mpx = std::make_shared<MultiplexedAlphabet>("mpx", UnicodeSets);

        mRE = transformCCs(mpx, targetRe);

        auto mpx_basis = mpx->getMultiplexedCCs();
        StreamSet * const CharClasses = mPipeline->CreateStreamSet(mpx_basis.size());

        mPipeline->CreateKernelCall<ByteClassesKernel>(std::move(mpx_basis), compressedBitStream, CharClasses);

        uncompressedCharClasses = decompressBitStream(compressedByteStream, CharClasses);
        auto fakeStreams = generateFakeStreams(uncompressedCharClasses, std::vector<unsigned>{8, 1});
        fakeMatchCopiedBits = fakeStreams[0];
        u8NoFinalStream = fakeStreams[1];
    }

    StreamSet * const MatchResults = mPipeline->CreateStreamSet(1, 1);

    // Multiplexing Grep Kernel is not Cachable, since it is possible that two REs with name "mpx_1" have different alphabets
    StreamSet * LineBreakStream = mPipeline->CreateStreamSet(1, 1);

    RE * const transformedLF = transformCCs(mpx, linefeedCC);
    std::unique_ptr<GrepKernelOptions> optionsLF = make_unique<GrepKernelOptions>();

    optionsLF->addAlphabet(mpx, uncompressedCharClasses);
    optionsLF->setSource(fakeMatchCopiedBits);
    optionsLF->setRE(transformedLF);
    optionsLF->setResults(LineBreakStream);
    mPipeline->CreateKernelCall<ICGrepKernel>(std::move(optionsLF));

    std::unique_ptr<GrepKernelOptions> options = make_unique<GrepKernelOptions>();
    options->addAlphabet(mpx, uncompressedCharClasses);
    options->setSource(fakeMatchCopiedBits);
    options->addExternal("UTF8_nonfinal", u8NoFinalStream);
    options->setRE(mRE);
    options->setResults(MatchResults);
    mPipeline->CreateKernelCall<ICGrepKernel>(std::move(options));

    StreamSet * Matches = MatchResults;
    if (mMoveMatchesToEOL) {
        StreamSet * const MovedMatches = mPipeline->CreateStreamSet();
        mPipeline->CreateKernelCall<MatchedLinesKernel>(Matches, LineBreakStream, MovedMatches);
        Matches = MovedMatches;
    }

    return std::pair<StreamSet *, StreamSet *>(LineBreakStream, Matches);
}

std::pair<StreamSet *, StreamSet *> LZ4GrepBaseGenerator::grep(RE * re, StreamSet * byteStream, StreamSet * uncompressedBasisBits, bool ccMultiplexing) {

    initREs(re);

    //  Regular Expression Processing and Analysis Phase
    StreamSet * const MatchResults = mPipeline->CreateStreamSet(1, 1);

    if (uncompressedBasisBits == nullptr) {
        uncompressedBasisBits = s2p(byteStream);
    }

    StreamSet * const LineBreakStream = linefeedStreamFromUncompressedBits(uncompressedBasisBits);

    if (ccMultiplexing) {

        const auto UnicodeSets = collectCCs(mRE, Unicode, std::set<Name *>({makeZeroWidth("\\b{g}")}));

        auto mpx = std::make_shared<MultiplexedAlphabet>("mpx", UnicodeSets);
        mRE = transformCCs(mpx, mRE);
        auto mpx_basis = mpx->getMultiplexedCCs();
        StreamSet * const CharClasses = mPipeline->CreateStreamSet(mpx_basis.size());
        mPipeline->CreateKernelCall<CharClassesKernel>(std::move(mpx_basis), uncompressedBasisBits, CharClasses);

        std::unique_ptr<GrepKernelOptions> options = make_unique<GrepKernelOptions>();
        options->addAlphabet(mpx, CharClasses);
        options->setSource(uncompressedBasisBits);
        options->setRE(mRE);
        options->setResults(MatchResults);
        mPipeline->CreateKernelCall<ICGrepKernel>(std::move(options));


    } else {

        bool anyGCB = hasGraphemeClusterBoundary(mRE);
        bool isSimple = (mGrepRecordBreak != GrepRecordBreakKind::Unicode) && (!anyGCB);
        if (isSimple) {
            mRE = toUTF8(mRE);
        }
        std::unique_ptr<GrepKernelOptions> options = make_unique<GrepKernelOptions>();
        options->setSource(uncompressedBasisBits);
        options->setRE(mRE);
        options->setResults(MatchResults);
        mPipeline->CreateKernelCall<ICGrepKernel>(std::move(options));
    }

    StreamSet * Matches = MatchResults;
    if (mMoveMatchesToEOL) {
        StreamSet * const MovedMatches = mPipeline->CreateStreamSet();
        mPipeline->CreateKernelCall<MatchedLinesKernel>(Matches, LineBreakStream, MovedMatches);
        Matches = MovedMatches;
    }

    return std::pair<StreamSet *, StreamSet *>(LineBreakStream, Matches);

}

void LZ4GrepBaseGenerator::invokeScanMatchGrep(char* fileBuffer, size_t blockStart, size_t blockEnd, bool hasBlockChecksum) {
    auto main = getScanMatchGrepMainFunction();
    std::ostringstream s;
    EmitMatch accum("", false, false, false, s);

    main(fileBuffer, blockStart, blockEnd, hasBlockChecksum, reinterpret_cast<intptr_t>(&accum));
    llvm::outs() << s.str();
}


void LZ4GrepBaseGenerator::generateFullyDecompressionScanMatchGrepPipeline(RE *regex) {
    StreamSet* compressedByteStream = loadByteStream();

    StreamSet * const uncompressedByteStream = byteStreamDecompression(compressedByteStream);
    StreamSet * uncompressedBitStream = s2p(uncompressedByteStream);

    StreamSet * LineBreakStream;
    StreamSet * Matches;
    std::tie(LineBreakStream, Matches) = grep(regex, uncompressedByteStream, uncompressedBitStream);


    Kernel * scanMatchK = mPipeline->CreateKernelCall<ScanMatchKernel>(Matches, LineBreakStream, uncompressedByteStream, match_accumulator);
    mPxDriver.LinkFunction(scanMatchK, "accumulate_match_wrapper", accumulate_match_wrapper);
    mPxDriver.LinkFunction(scanMatchK, "finalize_match_wrapper", finalize_match_wrapper);

    mMainMethod = mPipeline->compile();

}

void LZ4GrepBaseGenerator::generateMultiplexingScanMatchGrepPipeline(RE *regex, bool utf8CC) {

    StreamSet *compressedByteStream = nullptr, *compressedBasisBits = nullptr;
    std::tie(compressedByteStream, compressedBasisBits) = loadByteStreamAndBitStream();

    StreamSet * LineBreakStream;
    StreamSet * Matches;
    std::tie(LineBreakStream, Matches) = multiplexingGrep(regex, compressedByteStream, compressedBasisBits, utf8CC);

    LZ4BlockInfo blockInfo = getBlockInfo(compressedByteStream);

    StreamSet * const decompressionByteStream = mPipeline->CreateStreamSet(1, 8);
    mPipeline->CreateKernelCall<LZ4ByteStreamDecompressionKernel>(mFileSize, compressedByteStream, blockInfo, nullptr, decompressionByteStream );
    Kernel * const scanMatchK = mPipeline->CreateKernelCall<ScanMatchKernel>(Matches, LineBreakStream, decompressionByteStream, match_accumulator);
    mPxDriver.LinkFunction(scanMatchK, "accumulate_match_wrapper", accumulate_match_wrapper);
    mPxDriver.LinkFunction(scanMatchK, "finalize_match_wrapper", finalize_match_wrapper);
    mMainMethod = mPipeline->compile();
}


void LZ4GrepBaseGenerator::generateMultiplexingCountOnlyGrepPipeline(RE *regex, bool utf8CC) {
    StreamSet *compressedByteStream = nullptr, *compressedBasisBits = nullptr;
    std::tie(compressedByteStream, compressedBasisBits) = loadByteStreamAndBitStream();
    StreamSet * Matches = multiplexingGrep(regex, compressedByteStream, compressedBasisBits, utf8CC).second;
    mPipeline->CreateKernelCall<PopcountKernel>(Matches, mPipeline->getOutputScalar("countResult"));
    mMainMethod = mPipeline->compile();
}


void LZ4GrepBaseGenerator::generateFullyDecompressionCountOnlyGrepPipeline(RE *regex) {
    StreamSet * const uncompressedByteStream = generateUncompressedByteStream();
    StreamSet * LineBreakStream;
    StreamSet * Matches;
    std::tie(LineBreakStream, Matches) = grep(regex, uncompressedByteStream, nullptr);
    mPipeline->CreateKernelCall<PopcountKernel>(Matches, mPipeline->getOutputScalar("countResult"));
    mMainMethod = mPipeline->compile();
}


ScanMatchGrepMainFunctionType LZ4GrepBaseGenerator::getScanMatchGrepMainFunction() {
    return reinterpret_cast<ScanMatchGrepMainFunctionType>(mMainMethod);
}
CountOnlyGrepMainFunctionType LZ4GrepBaseGenerator::getCountOnlyGrepMainFunction() {
    return reinterpret_cast<CountOnlyGrepMainFunctionType>(mMainMethod);
}

StreamSets LZ4GrepBaseGenerator::generateFakeStreams(StreamSet * refStream, std::vector<unsigned> numOfStreams) {
    if (numOfStreams.empty()) {
        return StreamSets{};
    }
    StreamSets outputStreams;
    outputStreams.reserve(numOfStreams.size());
    for (const auto k : numOfStreams) {
        outputStreams.push_back(mPipeline->CreateStreamSet(k));
    }
    mPipeline->CreateKernelCall<FakeStreamGeneratingKernel>(refStream, outputStreams);
    return outputStreams;
}



StreamSets LZ4GrepBaseGenerator::decompressBitStreams(StreamSet *compressedByteStream, StreamSets compressedBitStreams) {
    // Default implementation here will be slow
    StreamSets retVec;
    for (unsigned i = 0; i < compressedBitStreams.size(); i++) {
        retVec.push_back(decompressBitStream(compressedByteStream, compressedBitStreams[i]));
    }
    return retVec;
}
