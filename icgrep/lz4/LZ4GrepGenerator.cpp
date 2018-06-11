
#include "LZ4GrepGenerator.h"

#include <boost/iostreams/device/mapped_file.hpp>

#include <llvm/Support/PrettyStackTrace.h>

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
#include <kernels/lz4/lz4_bitstream_not_kernel.h>
#include <kernels/lz4/lz4_fake_stream_generating_kernel.h>
#include <kernels/bitstream_pdep_kernel.h>
#include <kernels/bitstream_gather_pdep_kernel.h>
#include <re/re_toolchain.h>

#include <re/collect_ccs.h>
#include <re/replaceCC.h>

#include <UCD/resolve_properties.h>
#include <kernels/charclasses.h>
#include <kernels/grep_kernel.h>
#include <kernels/UCD_property_kernel.h>
#include <kernels/grapheme_kernel.h>
#include <kernels/linebreak_kernel.h>
#include <kernels/streams_merge.h>
#include <kernels/scanmatchgen.h>
#include <kernels/until_n.h>
#include <re/casing.h>
#include <re/exclude_CC.h>
#include <re/to_utf8.h>
#include <re/re_analysis.h>
#include <re/re_name_resolve.h>
#include <re/re_name_gather.h>
#include <re/re_multiplex.h>
#include <re/re_utility.h>
#include <re/grapheme_clusters.h>
#include <re/printer_re.h>
#include <llvm/Support/raw_ostream.h>
#include <llvm/Support/Debug.h>
#include <kernels/lz4/lz4_block_decoder.h>
#include <kernels/lz4/lz4_swizzled_aio.h>


namespace re { class CC; }

using namespace llvm;
using namespace parabix;
using namespace kernel;
using namespace grep;

LZ4GrepGenerator::LZ4GrepGenerator(bool enableMultiplexing): LZ4Generator(), mEnableMultiplexing(enableMultiplexing) {
    mGrepRecordBreak = grep::GrepRecordBreakKind::LF;
    mMoveMatchesToEOL = true;
}

void LZ4GrepGenerator::initREs(std::vector<re::RE *> & REs) {
    if (mGrepRecordBreak == GrepRecordBreakKind::Unicode) {
        mBreakCC = re::makeCC(re::makeCC(0x0A, 0x0D), re::makeCC(re::makeCC(0x85), re::makeCC(0x2028, 0x2029)));
    } else if (mGrepRecordBreak == GrepRecordBreakKind::Null) {
        mBreakCC = re::makeByte(0);  // Null
    } else {
        mBreakCC = re::makeByte(0x0A); // LF
    }
    re::RE * anchorRE = mBreakCC;
    if (mGrepRecordBreak == GrepRecordBreakKind::Unicode) {
        re::Name * anchorName = re::makeName("UTF8_LB", re::Name::Type::Unicode);
        anchorName->setDefinition(re::makeUnicodeBreak());
        anchorRE = anchorName;
    }

    mREs = REs;
    bool allAnchored = true;
    for(unsigned i = 0; i < mREs.size(); ++i) {
        if (!hasEndAnchor(mREs[i])) allAnchored = false;
        mREs[i] = resolveModesAndExternalSymbols(mREs[i]);
        mREs[i] = re::exclude_CC(mREs[i], mBreakCC);
        mREs[i] = resolveAnchors(mREs[i], anchorRE);
        re::gatherUnicodeProperties(mREs[i], mUnicodeProperties);
        mREs[i] = regular_expression_passes(mREs[i]);
    }
    if (allAnchored && (mGrepRecordBreak != GrepRecordBreakKind::Unicode)) mMoveMatchesToEOL = false;

}


parabix::StreamSetBuffer * LZ4GrepGenerator::linefeedStreamFromDecompressedBits(parabix::StreamSetBuffer *decompressedBasisBits) {
//    auto mGrepDriver = &mPxDriver;
    auto & idb = mPxDriver.getBuilder();
    const unsigned baseBufferSize = this->getInputBufferBlocks(idb);
    StreamSetBuffer * LineFeedStream = mPxDriver.addBuffer<StaticBuffer>(idb, idb->getStreamSetTy(1, 1), baseBufferSize);
    kernel::Kernel * linefeedK = mPxDriver.addKernelInstance<kernel::LineFeedKernelBuilder>(idb, Binding{idb->getStreamSetTy(8), "basis", FixedRate(), Principal()});
    mPxDriver.makeKernelCall(linefeedK, {decompressedBasisBits}, {LineFeedStream});
    return LineFeedStream;
}

StreamSetBuffer * LZ4GrepGenerator::convertCompressedBitsStreamWithAioApproach(parabix::StreamSetBuffer* compressedBitStream, int numberOfStream, std::string prefix) {
    auto mGrepDriver = &mPxDriver;
    auto & iBuilder = mGrepDriver->getBuilder();

    //// Decode Block Information
    StreamSetBuffer * const BlockData_IsCompressed = mPxDriver.addBuffer<StaticBuffer>(iBuilder, iBuilder->getStreamSetTy(1, 8), this->getInputBufferBlocks(iBuilder), 1);
    StreamSetBuffer * const BlockData_BlockStart = mPxDriver.addBuffer<StaticBuffer>(iBuilder, iBuilder->getStreamSetTy(1, 64), this->getInputBufferBlocks(iBuilder), 1);
    StreamSetBuffer * const BlockData_BlockEnd = mPxDriver.addBuffer<StaticBuffer>(iBuilder, iBuilder->getStreamSetTy(1, 64), this->getInputBufferBlocks(iBuilder), 1);

    //// Generate Helper Markers Extenders, FX, XF
    StreamSetBuffer * const Extenders = mPxDriver.addBuffer<StaticBuffer>(iBuilder, iBuilder->getStreamSetTy(1, 1), this->getInputBufferBlocks(iBuilder), 1);
    mMatchOffsetMarker = mPxDriver.addBuffer<StaticBuffer>(iBuilder, iBuilder->getStreamSetTy(1, 1), this->getInputBufferBlocks(iBuilder));
    Kernel * extenderK = mPxDriver.addKernelInstance<ParabixCharacterClassKernelBuilder>(iBuilder, "extenders", std::vector<re::CC *>{re::makeCC(0xFF)}, 8);
    mPxDriver.makeKernelCall(extenderK, {mCompressedBasisBits}, {Extenders});


    Kernel * blockDecoderK = mPxDriver.addKernelInstance<LZ4BlockDecoderNewKernel>(iBuilder);
    blockDecoderK->setInitialArguments({iBuilder->CreateTrunc(mHasBlockChecksum, iBuilder->getInt1Ty()), mHeaderSize, mFileSize});
    mPxDriver.makeKernelCall(blockDecoderK, {mCompressedByteStream}, {BlockData_IsCompressed, BlockData_BlockStart, BlockData_BlockEnd});


    // Produce unswizzled bit streams
    StreamSetBuffer * u16Swizzle0 = mPxDriver.addBuffer<StaticBuffer>(iBuilder, iBuilder->getStreamSetTy(4), this->getInputBufferBlocks(iBuilder), 1);
    Kernel * unSwizzleK = mPxDriver.addKernelInstance<SwizzleGenerator>(iBuilder, 4, 1, 1, 64, "source");
    mPxDriver.makeKernelCall(unSwizzleK, {compressedBitStream}, {u16Swizzle0});

    StreamSetBuffer * decompressedSwizzled0 = mPxDriver.addBuffer<StaticBuffer>(iBuilder, iBuilder->getStreamSetTy(4), this->getInputBufferBlocks(iBuilder), 1);


    Kernel* lz4AioK = mPxDriver.addKernelInstance<LZ4SwizzledAioKernel>(iBuilder, 4, 1, 4);
    lz4AioK->setInitialArguments({mFileSize});
    mPxDriver.makeKernelCall(
            lz4AioK,
            {
                    mCompressedByteStream,
                    Extenders,

                    // Block Data
                    BlockData_IsCompressed,
                    BlockData_BlockStart,
                    BlockData_BlockEnd,

                    u16Swizzle0,
            }, {
                    decompressedSwizzled0,
            });



    StreamSetBuffer * const decompressionBitStream = mPxDriver.addBuffer<StaticBuffer>(iBuilder, iBuilder->getStreamSetTy(8, 1), this->getDecompressedBufferBlocks(iBuilder));
    Kernel * unSwizzleK2 = mPxDriver.addKernelInstance<SwizzleGenerator>(iBuilder, 4, 1, 1, 64, "dst");
    mPxDriver.makeKernelCall(unSwizzleK2, {decompressedSwizzled0}, {decompressionBitStream});

    return decompressionBitStream;

}

StreamSetBuffer * LZ4GrepGenerator::convertCompressedBitsStreamWithSwizzledApproach(parabix::StreamSetBuffer* compressedBitStream, int numberOfStream, std::string prefix) {
    auto mGrepDriver = &mPxDriver;
    auto & idb = mGrepDriver->getBuilder();

    StreamSetBuffer * u16Swizzle0 = mPxDriver.addBuffer<StaticBuffer>(idb, idb->getStreamSetTy(4), this->getInputBufferBlocks(idb), 1);
    Kernel * delK = mPxDriver.addKernelInstance<SwizzledDeleteByPEXTkernel>(idb, 4, 64);
    mPxDriver.makeKernelCall(delK, {mDeletionMarker, compressedBitStream}, {u16Swizzle0});

    StreamSetBuffer * depositedSwizzle0 = mPxDriver.addBuffer<StaticBuffer>(idb, idb->getStreamSetTy(4), this->getInputBufferBlocks(idb), 1);
    Kernel * multiplePdepK = mPxDriver.addKernelInstance<SwizzledMultiplePDEPkernel>(idb, 4, 1);
    mPxDriver.makeKernelCall(multiplePdepK, {mDepositMarker, u16Swizzle0}, {depositedSwizzle0});

    StreamSetBuffer * matchCopiedSwizzle0 = mPxDriver.addBuffer<StaticBuffer>(idb, idb->getStreamSetTy(4), this->getDecompressedBufferBlocks(idb), 1);
    Kernel * swizzledMatchCopyK = mPxDriver.addKernelInstance<LZ4SwizzledMatchCopyKernel>(idb, 4, 1, 4);
    mPxDriver.makeKernelCall(swizzledMatchCopyK, {mMatchOffsetMarker, mM0Marker, mCompressedByteStream, depositedSwizzle0}, {matchCopiedSwizzle0});

    // Produce unswizzled bit streams
    StreamSetBuffer * matchCopiedBits = mPxDriver.addBuffer<StaticBuffer>(idb, idb->getStreamSetTy(8), this->getInputBufferBlocks(idb));
    Kernel * unSwizzleK = mPxDriver.addKernelInstance<SwizzleGenerator>(idb, 4, 1, 1);
    mPxDriver.makeKernelCall(unSwizzleK, {matchCopiedSwizzle0}, {matchCopiedBits});

    return matchCopiedBits;
}
parabix::StreamSetBuffer * LZ4GrepGenerator::convertCompressedBitsStream(parabix::StreamSetBuffer* compressedBitStream, int numberOfStream, std::string prefix) {
    if (numberOfStream == 4) {
        return this->convertCompressedBitsStreamWithSwizzledApproach(compressedBitStream, numberOfStream, prefix);
    }

    auto mGrepDriver = &mPxDriver;
    auto & idb = mGrepDriver->getBuilder();

    // Extract (Deletion)
    this->generateCompressionMarker(idb);

    StreamSetBuffer * deletedBits = mPxDriver.addBuffer<StaticBuffer>(idb, idb->getStreamSetTy(numberOfStream), this->getInputBufferBlocks(idb));
    StreamSetBuffer * deletionCounts = mPxDriver.addBuffer<StaticBuffer>(idb, idb->getStreamSetTy(), this->getInputBufferBlocks(idb));

    Kernel * delK = mPxDriver.addKernelInstance<PEXTFieldCompressKernel>(idb, 64, numberOfStream);
    mPxDriver.makeKernelCall(delK, {compressedBitStream, mCompressionMarker}, {deletedBits, deletionCounts});

    StreamSetBuffer * compressedLineStream = mPxDriver.addBuffer<StaticBuffer>(idb, idb->getStreamSetTy(numberOfStream), this->getInputBufferBlocks(idb));
    Kernel * streamCompressionK = mPxDriver.addKernelInstance<StreamCompressKernel>(idb, 64, numberOfStream);
    mPxDriver.makeKernelCall(streamCompressionK, {deletedBits, deletionCounts}, {compressedLineStream});

    // Deposit
    StreamSetBuffer * depositedBits = mPxDriver.addBuffer<StaticBuffer>(idb, idb->getStreamSetTy(numberOfStream), this->getDecompressedBufferBlocks(idb));
    Kernel * bitStreamPDEPk = mPxDriver.addKernelInstance<BitStreamPDEPKernel>(idb, numberOfStream, prefix + "BitStreamPDEPKernel");
    mPxDriver.makeKernelCall(bitStreamPDEPk, {mDepositMarker, compressedLineStream}, {depositedBits});

    // Match Copy
    StreamSetBuffer * matchCopiedBits = mPxDriver.addBuffer<StaticBuffer>(idb, idb->getStreamSetTy(numberOfStream), this->getDecompressedBufferBlocks(idb));
    Kernel * bitStreamMatchCopyK = mPxDriver.addKernelInstance<LZ4BitStreamMatchCopyKernel>(idb, numberOfStream, prefix + "BitStreamMatchCopyKernel");
    mPxDriver.makeKernelCall(bitStreamMatchCopyK, {mMatchOffsetMarker, mM0Marker, mCompressedByteStream, depositedBits}, {matchCopiedBits});

    return matchCopiedBits;
}

parabix::StreamSetBuffer * LZ4GrepGenerator::linefeedStreamFromCompressedBits() {
    auto mGrepDriver = &mPxDriver;
    auto & idb = mGrepDriver->getBuilder();
    const unsigned baseBufferSize = this->getInputBufferBlocks(idb);

    StreamSetBuffer * CompressedLineFeedStream = mPxDriver.addBuffer<StaticBuffer>(idb, idb->getStreamSetTy(1, 1), baseBufferSize);
    kernel::Kernel * linefeedK = mPxDriver.addKernelInstance<kernel::LineFeedKernelBuilder>(idb, Binding{idb->getStreamSetTy(8), "basis", FixedRate(), Principal()});
    mPxDriver.makeKernelCall(linefeedK, {mCompressedBasisBits}, {CompressedLineFeedStream});
    return this->convertCompressedBitsStream(CompressedLineFeedStream, 1, "LineFeed");
}

void LZ4GrepGenerator::generateMultiplexingCompressedBitStream(std::vector<re::RE *> &REs) {
    this->initREs(REs);
    auto mGrepDriver = &mPxDriver;

    auto & idb = mGrepDriver->getBuilder();
    // TODO: until we automate stream buffer sizing, use this calculation to determine how large our matches buffer needs to be.
    const unsigned baseBufferSize = this->getInputBufferBlocks(idb);


    //  Regular Expression Processing and Analysis Phase
    const auto nREs = mREs.size();

    std::vector<StreamSetBuffer *> MatchResultsBufs(nREs);

    std::map<std::string, StreamSetBuffer *> propertyStream;

    std::vector<std::string> externalStreamNames;
    std::set<re::Name *> UnicodeProperties;

    const auto UnicodeSets = re::collectCCs(mREs[0], &cc::Unicode, std::set<re::Name *>({re::makeZeroWidth("\\b{g}")}));
    StreamSetBuffer * const MatchResults = mGrepDriver->addBuffer<StaticBuffer>(idb, idb->getStreamSetTy(1, 1), baseBufferSize);

    mpx = make_unique<cc::MultiplexedAlphabet>("mpx", UnicodeSets);
    mREs[0] = transformCCs(mpx.get(), mREs[0]);
    std::vector<re::CC *> mpx_basis = mpx->getMultiplexedCCs();
    auto numOfCharacterClasses = mpx_basis.size();
    StreamSetBuffer * CharClasses = mGrepDriver->addBuffer<StaticBuffer>(idb, idb->getStreamSetTy(numOfCharacterClasses), baseBufferSize);

    kernel::Kernel * ccK = mGrepDriver->addKernelInstance<kernel::CharClassesKernel>(idb, std::move(mpx_basis));
    mGrepDriver->makeKernelCall(ccK, {mCompressedBasisBits}, {CharClasses});

    StreamSetBuffer * CompressedLineFeedStream = mPxDriver.addBuffer<StaticBuffer>(idb, idb->getStreamSetTy(1, 1), baseBufferSize);
    kernel::Kernel * linefeedK = mPxDriver.addKernelInstance<kernel::LineFeedKernelBuilder>(idb, Binding{idb->getStreamSetTy(8), "basis", FixedRate(), Principal()});
    mPxDriver.makeKernelCall(linefeedK, {mCompressedBasisBits}, {CompressedLineFeedStream});

    StreamSetBuffer * combinedStream = mGrepDriver->addBuffer<StaticBuffer>(idb, idb->getStreamSetTy(numOfCharacterClasses + 1), baseBufferSize);
    kernel::Kernel* streamCombineKernel = mPxDriver.addKernelInstance<StreamsCombineKernel>(idb, std::vector<unsigned>({1, (unsigned)numOfCharacterClasses}));
    mPxDriver.makeKernelCall(streamCombineKernel, {CompressedLineFeedStream, CharClasses}, {combinedStream});
    StreamSetBuffer * decompressedCombinedStream = this->convertCompressedBitsStreamWithAioApproach(combinedStream, 1 + numOfCharacterClasses, "combined");

    StreamSetBuffer * LineBreakStream = mGrepDriver->addBuffer<StaticBuffer>(idb, idb->getStreamSetTy(1), baseBufferSize);
    StreamSetBuffer * decompressedCharClasses = mGrepDriver->addBuffer<StaticBuffer>(idb, idb->getStreamSetTy(numOfCharacterClasses), baseBufferSize);
    kernel::Kernel* streamSplitKernel = mPxDriver.addKernelInstance<StreamsSplitKernel>(idb, std::vector<unsigned>({1, (unsigned)numOfCharacterClasses}));
    mPxDriver.makeKernelCall(streamSplitKernel, {decompressedCombinedStream}, {LineBreakStream, decompressedCharClasses});



    StreamSetBuffer * fakeMatchCopiedBits = mPxDriver.addBuffer<StaticBuffer>(idb, idb->getStreamSetTy(8), this->getInputBufferBlocks(idb));
    Kernel* fakeStreamGeneratorK = mPxDriver.addKernelInstance<LZ4FakeStreamGeneratingKernel>(idb, numOfCharacterClasses, 8);
    mPxDriver.makeKernelCall(fakeStreamGeneratorK, {decompressedCharClasses}, {fakeMatchCopiedBits});

    kernel::Kernel * icgrepK = mGrepDriver->addKernelInstance<kernel::ICGrepKernel>(idb, mREs[0], externalStreamNames, std::vector<cc::Alphabet *>{mpx.get()});
    mGrepDriver->makeKernelCall(icgrepK, {fakeMatchCopiedBits, decompressedCharClasses}, {MatchResults});
    MatchResultsBufs[0] = MatchResults;

    StreamSetBuffer * MergedResults = MatchResultsBufs[0];
    if (mREs.size() > 1) {
        MergedResults = mGrepDriver->addBuffer<StaticBuffer>(idb, idb->getStreamSetTy(1, 1), baseBufferSize);
        kernel::Kernel * streamsMergeK = mGrepDriver->addKernelInstance<kernel::StreamsMerge>(idb, 1, mREs.size());
        mGrepDriver->makeKernelCall(streamsMergeK, MatchResultsBufs, {MergedResults});
    }
    StreamSetBuffer * Matches = MergedResults;
    if (mMoveMatchesToEOL) {
        StreamSetBuffer * OriginalMatches = Matches;
        kernel::Kernel * matchedLinesK = mGrepDriver->addKernelInstance<kernel::MatchedLinesKernel>(idb);
        Matches = mGrepDriver->addBuffer<StaticBuffer>(idb, idb->getStreamSetTy(1, 1), baseBufferSize);
        mGrepDriver->makeKernelCall(matchedLinesK, {OriginalMatches, LineBreakStream}, {Matches});
    }

//    if (MaxCountFlag > 0) {
//        kernel::Kernel * untilK = mGrepDriver->addKernelInstance<kernel::UntilNkernel>(idb);
//        untilK->setInitialArguments({idb->getSize(MaxCountFlag)});
//        StreamSetBuffer * const AllMatches = Matches;
//        Matches = mGrepDriver->addBuffer<StaticBuffer>(idb, idb->getStreamSetTy(1, 1), baseBufferSize);
//        mGrepDriver->makeKernelCall(untilK, {AllMatches}, {Matches});
//    }

//    return std::pair<StreamSetBuffer *, StreamSetBuffer *>(LineBreakStream, Matches);

};
std::pair<parabix::StreamSetBuffer *, parabix::StreamSetBuffer *> LZ4GrepGenerator::multiplexingGrepPipeline(std::vector<re::RE *> &REs, bool useAio) {

    this->initREs(REs);
    auto mGrepDriver = &mPxDriver;

    auto & idb = mGrepDriver->getBuilder();
    // TODO: until we automate stream buffer sizing, use this calculation to determine how large our matches buffer needs to be.
    const unsigned baseBufferSize = this->getInputBufferBlocks(idb);
    int MaxCountFlag = 0;

    //  Regular Expression Processing and Analysis Phase
    const auto nREs = mREs.size();

    std::vector<StreamSetBuffer *> MatchResultsBufs(nREs);


    std::map<std::string, StreamSetBuffer *> propertyStream;

    std::vector<std::string> externalStreamNames;
    std::set<re::Name *> UnicodeProperties;

    const auto UnicodeSets = re::collectCCs(mREs[0], &cc::Unicode, std::set<re::Name *>({re::makeZeroWidth("\\b{g}")}));
    StreamSetBuffer * const MatchResults = mGrepDriver->addBuffer<StaticBuffer>(idb, idb->getStreamSetTy(1, 1), baseBufferSize);

    mpx = make_unique<cc::MultiplexedAlphabet>("mpx", UnicodeSets);
    mREs[0] = transformCCs(mpx.get(), mREs[0]);
    std::vector<re::CC *> mpx_basis = mpx->getMultiplexedCCs();
    auto numOfCharacterClasses = mpx_basis.size();
    StreamSetBuffer * CharClasses = mGrepDriver->addBuffer<StaticBuffer>(idb, idb->getStreamSetTy(numOfCharacterClasses), baseBufferSize);

    kernel::Kernel * ccK = mGrepDriver->addKernelInstance<kernel::CharClassesKernel>(idb, std::move(mpx_basis));
    mGrepDriver->makeKernelCall(ccK, {mCompressedBasisBits}, {CharClasses});

    StreamSetBuffer * CompressedLineFeedStream = mPxDriver.addBuffer<StaticBuffer>(idb, idb->getStreamSetTy(1, 1), baseBufferSize);
    kernel::Kernel * linefeedK = mPxDriver.addKernelInstance<kernel::LineFeedKernelBuilder>(idb, Binding{idb->getStreamSetTy(8), "basis", FixedRate(), Principal()});
    mPxDriver.makeKernelCall(linefeedK, {mCompressedBasisBits}, {CompressedLineFeedStream});

    StreamSetBuffer * combinedStream = mGrepDriver->addBuffer<StaticBuffer>(idb, idb->getStreamSetTy(numOfCharacterClasses + 1), baseBufferSize);
    kernel::Kernel* streamCombineKernel = mPxDriver.addKernelInstance<StreamsCombineKernel>(idb, std::vector<unsigned>({1, (unsigned)numOfCharacterClasses}));
    mPxDriver.makeKernelCall(streamCombineKernel, {CompressedLineFeedStream, CharClasses}, {combinedStream});
    StreamSetBuffer * decompressedCombinedStream = nullptr;

    if (useAio) {
        decompressedCombinedStream = this->convertCompressedBitsStreamWithAioApproach(combinedStream, 1 + numOfCharacterClasses, "combined");
    } else {
        decompressedCombinedStream = this->convertCompressedBitsStream(combinedStream, 1 + numOfCharacterClasses, "combined");
    }

    StreamSetBuffer * LineBreakStream = mGrepDriver->addBuffer<StaticBuffer>(idb, idb->getStreamSetTy(1), baseBufferSize);
    StreamSetBuffer * decompressedCharClasses = mGrepDriver->addBuffer<StaticBuffer>(idb, idb->getStreamSetTy(numOfCharacterClasses), baseBufferSize);
    kernel::Kernel* streamSplitKernel = mPxDriver.addKernelInstance<StreamsSplitKernel>(idb, std::vector<unsigned>({1, (unsigned)numOfCharacterClasses}));
    mPxDriver.makeKernelCall(streamSplitKernel, {decompressedCombinedStream}, {LineBreakStream, decompressedCharClasses});

    /*
    StreamSetBuffer * LineBreakStream = this->convertCompressedBitsStream(CompressedLineFeedStream, 1, "LineFeed");
    StreamSetBuffer * decompressedCharClasses = this->convertCompressedBitsStream(CharClasses, numOfCharacterClasses, "mpx");
     */

    StreamSetBuffer * fakeMatchCopiedBits = mPxDriver.addBuffer<StaticBuffer>(idb, idb->getStreamSetTy(8), this->getInputBufferBlocks(idb));
    Kernel* fakeStreamGeneratorK = mPxDriver.addKernelInstance<LZ4FakeStreamGeneratingKernel>(idb, numOfCharacterClasses, 8);
    mPxDriver.makeKernelCall(fakeStreamGeneratorK, {decompressedCharClasses}, {fakeMatchCopiedBits});

    kernel::Kernel * icgrepK = mGrepDriver->addKernelInstance<kernel::ICGrepKernel>(idb, mREs[0], externalStreamNames, std::vector<cc::Alphabet *>{mpx.get()});
    mGrepDriver->makeKernelCall(icgrepK, {fakeMatchCopiedBits, decompressedCharClasses}, {MatchResults});
    MatchResultsBufs[0] = MatchResults;

    StreamSetBuffer * MergedResults = MatchResultsBufs[0];
    if (mREs.size() > 1) {
        MergedResults = mGrepDriver->addBuffer<StaticBuffer>(idb, idb->getStreamSetTy(1, 1), baseBufferSize);
        kernel::Kernel * streamsMergeK = mGrepDriver->addKernelInstance<kernel::StreamsMerge>(idb, 1, mREs.size());
        mGrepDriver->makeKernelCall(streamsMergeK, MatchResultsBufs, {MergedResults});
    }
    StreamSetBuffer * Matches = MergedResults;
    if (mMoveMatchesToEOL) {
        StreamSetBuffer * OriginalMatches = Matches;
        kernel::Kernel * matchedLinesK = mGrepDriver->addKernelInstance<kernel::MatchedLinesKernel>(idb);
        Matches = mGrepDriver->addBuffer<StaticBuffer>(idb, idb->getStreamSetTy(1, 1), baseBufferSize);
        mGrepDriver->makeKernelCall(matchedLinesK, {OriginalMatches, LineBreakStream}, {Matches});
    }

    if (MaxCountFlag > 0) {
        kernel::Kernel * untilK = mGrepDriver->addKernelInstance<kernel::UntilNkernel>(idb);
        untilK->setInitialArguments({idb->getSize(MaxCountFlag)});
        StreamSetBuffer * const AllMatches = Matches;
        Matches = mGrepDriver->addBuffer<StaticBuffer>(idb, idb->getStreamSetTy(1, 1), baseBufferSize);
        mGrepDriver->makeKernelCall(untilK, {AllMatches}, {Matches});
    }

    return std::pair<StreamSetBuffer *, StreamSetBuffer *>(LineBreakStream, Matches);
};

std::pair<parabix::StreamSetBuffer *, parabix::StreamSetBuffer *> LZ4GrepGenerator::grepPipeline(
        std::vector<re::RE *> &REs, parabix::StreamSetBuffer *decompressedBasisBits) {

    this->initREs(REs);
    auto mGrepDriver = &mPxDriver;

    auto & idb = mGrepDriver->getBuilder();
    // TODO: until we automate stream buffer sizing, use this calculation to determine how large our matches buffer needs to be.
    const unsigned baseBufferSize = this->getInputBufferBlocks(idb);
    int MaxCountFlag = 0;

    //  Regular Expression Processing and Analysis Phase
    const auto nREs = mREs.size();

    std::vector<StreamSetBuffer *> MatchResultsBufs(nREs);

    StreamSetBuffer * LineBreakStream = this->linefeedStreamFromDecompressedBits(decompressedBasisBits);


    std::map<std::string, StreamSetBuffer *> propertyStream;

    for(unsigned i = 0; i < nREs; ++i) {
        std::vector<std::string> externalStreamNames;
        std::vector<StreamSetBuffer *> icgrepInputSets = {decompressedBasisBits};

        std::set<re::Name *> UnicodeProperties;

        StreamSetBuffer * MatchResults = mGrepDriver->addBuffer<StaticBuffer>(idb, idb->getStreamSetTy(1, 1), baseBufferSize);
        kernel::Kernel * icgrepK = mGrepDriver->addKernelInstance<kernel::ICGrepKernel>(idb, mREs[i], externalStreamNames);
        mGrepDriver->makeKernelCall(icgrepK, icgrepInputSets, {MatchResults});
        MatchResultsBufs[i] = MatchResults;
    }

    StreamSetBuffer * MergedResults = MatchResultsBufs[0];
    if (mREs.size() > 1) {
        MergedResults = mGrepDriver->addBuffer<StaticBuffer>(idb, idb->getStreamSetTy(1, 1), baseBufferSize);
        kernel::Kernel * streamsMergeK = mGrepDriver->addKernelInstance<kernel::StreamsMerge>(idb, 1, mREs.size());
        mGrepDriver->makeKernelCall(streamsMergeK, MatchResultsBufs, {MergedResults});
    }
    StreamSetBuffer * Matches = MergedResults;
    if (mMoveMatchesToEOL) {
        StreamSetBuffer * OriginalMatches = Matches;
        kernel::Kernel * matchedLinesK = mGrepDriver->addKernelInstance<kernel::MatchedLinesKernel>(idb);
        Matches = mGrepDriver->addBuffer<StaticBuffer>(idb, idb->getStreamSetTy(1, 1), baseBufferSize);
        mGrepDriver->makeKernelCall(matchedLinesK, {OriginalMatches, LineBreakStream}, {Matches});
    }

    if (MaxCountFlag > 0) {
        kernel::Kernel * untilK = mGrepDriver->addKernelInstance<kernel::UntilNkernel>(idb);
        untilK->setInitialArguments({idb->getSize(MaxCountFlag)});
        StreamSetBuffer * const AllMatches = Matches;
        Matches = mGrepDriver->addBuffer<StaticBuffer>(idb, idb->getStreamSetTy(1, 1), baseBufferSize);
        mGrepDriver->makeKernelCall(untilK, {AllMatches}, {Matches});
    }

    return std::pair<StreamSetBuffer *, StreamSetBuffer *>(LineBreakStream, Matches);

}

void LZ4GrepGenerator::invokeScanMatchGrep(char* fileBuffer, size_t blockStart, size_t blockEnd, bool hasBlockChecksum) {
    auto main = this->getScanMatchGrepMainFunction();
    std::ostringstream s;
    EmitMatch accum("", false, false, s);

    main(fileBuffer, blockStart, blockEnd, hasBlockChecksum, reinterpret_cast<intptr_t>(&accum));
    llvm::outs() << s.str();
}

void LZ4GrepGenerator::generateScanMatchGrepPipeline(re::RE* regex) {
    auto & iBuilder = mPxDriver.getBuilder();
    this->generateScanMatchMainFunc(iBuilder);

    StreamSetBuffer * const DecompressedByteStream = mPxDriver.addBuffer<StaticBuffer>(iBuilder, iBuilder->getStreamSetTy(1, 8), this->getDecompressedBufferBlocks(iBuilder));

    // GeneratePipeline
    this->generateLoadByteStreamAndBitStream(iBuilder);
    this->generateExtractAndDepositMarkers(iBuilder);

    auto swizzle = this->generateSwizzleExtractData(iBuilder);

    StreamSetBuffer * depositedSwizzle0 = mPxDriver.addBuffer<StaticBuffer>(iBuilder, iBuilder->getStreamSetTy(4), this->getDecompressedBufferBlocks(iBuilder), 1);
    StreamSetBuffer * depositedSwizzle1 = mPxDriver.addBuffer<StaticBuffer>(iBuilder, iBuilder->getStreamSetTy(4), this->getDecompressedBufferBlocks(iBuilder), 1);

    Kernel * multiplePdepK = mPxDriver.addKernelInstance<SwizzledMultiplePDEPkernel>(iBuilder, 4, 2);
    mPxDriver.makeKernelCall(multiplePdepK, {mDepositMarker, swizzle.first, swizzle.second}, {depositedSwizzle0, depositedSwizzle1});

    StreamSetBuffer * matchCopiedSwizzle0 = mPxDriver.addBuffer<StaticBuffer>(iBuilder, iBuilder->getStreamSetTy(4), this->getInputBufferBlocks(iBuilder), 1);
    StreamSetBuffer * matchCopiedSwizzle1 = mPxDriver.addBuffer<StaticBuffer>(iBuilder, iBuilder->getStreamSetTy(4), this->getInputBufferBlocks(iBuilder), 1);

    Kernel * swizzledMatchCopyK = mPxDriver.addKernelInstance<LZ4SwizzledMatchCopyKernel>(iBuilder, 4, 2, 4);
    mPxDriver.makeKernelCall(swizzledMatchCopyK, {mMatchOffsetMarker, mM0Marker, mCompressedByteStream, depositedSwizzle0, depositedSwizzle1}, {matchCopiedSwizzle0, matchCopiedSwizzle1});

    // Produce unswizzled bit streams
    StreamSetBuffer * extractedbits = mPxDriver.addBuffer<StaticBuffer>(iBuilder, iBuilder->getStreamSetTy(8), this->getInputBufferBlocks(iBuilder));
    Kernel * unSwizzleK = mPxDriver.addKernelInstance<SwizzleGenerator>(iBuilder, 8, 1, 2);
    mPxDriver.makeKernelCall(unSwizzleK, {matchCopiedSwizzle0, matchCopiedSwizzle1}, {extractedbits});

    Kernel * p2sK = mPxDriver.addKernelInstance<P2SKernel>(iBuilder);
    mPxDriver.makeKernelCall(p2sK, {extractedbits}, {DecompressedByteStream});

    StreamSetBuffer * LineBreakStream;
    StreamSetBuffer * Matches;
    std::vector<re::RE*> res = {regex};
    std::tie(LineBreakStream, Matches) = grepPipeline(res, extractedbits);

    kernel::Kernel * scanMatchK = mPxDriver.addKernelInstance<kernel::ScanMatchKernel>(iBuilder);
    scanMatchK->setInitialArguments({match_accumulator});
    mPxDriver.makeKernelCall(scanMatchK, {Matches, LineBreakStream, DecompressedByteStream}, {});
    mPxDriver.LinkFunction(*scanMatchK, "accumulate_match_wrapper", &accumulate_match_wrapper);
    mPxDriver.LinkFunction(*scanMatchK, "finalize_match_wrapper", &finalize_match_wrapper);

    mPxDriver.generatePipelineIR();
    mPxDriver.deallocateBuffers();

    iBuilder->CreateRetVoid();

    mPxDriver.finalizeObject();
}

void LZ4GrepGenerator::generateMultiplexingSwizzledAioPipeline(re::RE* regex) {
    auto & iBuilder = mPxDriver.getBuilder();
    this->generateMainFunc(iBuilder);

    // GeneratePipeline
    this->generateLoadByteStreamAndBitStream(iBuilder);

    std::vector<re::RE*> res = {regex};
    this->generateMultiplexingCompressedBitStream(res);

    mPxDriver.generatePipelineIR();
    mPxDriver.deallocateBuffers();

    iBuilder->CreateRetVoid();

    mPxDriver.finalizeObject();
}

void LZ4GrepGenerator::generateMultiplexingSwizzledAioPipeline2(re::RE* regex) {
    auto & iBuilder = mPxDriver.getBuilder();
    this->generateCountOnlyMainFunc(iBuilder);

    // GeneratePipeline
    this->generateLoadByteStreamAndBitStream(iBuilder);
//    this->generateExtractAndDepositMarkers(iBuilder);

    StreamSetBuffer * LineBreakStream;
    StreamSetBuffer * Matches;
    std::vector<re::RE*> res = {regex};
    std::tie(LineBreakStream, Matches) = multiplexingGrepPipeline(res, true);

    kernel::Kernel * matchCountK = mPxDriver.addKernelInstance<kernel::PopcountKernel>(iBuilder);
    mPxDriver.makeKernelCall(matchCountK, {Matches}, {});
    mPxDriver.generatePipelineIR();

    iBuilder->setKernel(matchCountK);
    Value * matchedLineCount = iBuilder->getAccumulator("countResult");
    matchedLineCount = iBuilder->CreateZExt(matchedLineCount, iBuilder->getInt64Ty());

    mPxDriver.deallocateBuffers();

    iBuilder->CreateRet(matchedLineCount);


    mPxDriver.finalizeObject();
}

void LZ4GrepGenerator::generateSwizzledAioPipeline(re::RE* regex) {
    auto & iBuilder = mPxDriver.getBuilder();
    this->generateCountOnlyMainFunc(iBuilder);

    // GeneratePipeline
    this->generateLoadByteStreamAndBitStream(iBuilder);

    StreamSetBuffer * const decompressionBitStream = this->generateSwizzledAIODecompression(iBuilder);

    StreamSetBuffer * LineBreakStream;
    StreamSetBuffer * Matches;
    std::vector<re::RE*> res = {regex};
    std::tie(LineBreakStream, Matches) = grepPipeline(res, decompressionBitStream);
/*
    StreamSetBuffer * const decompressionByteStream = mPxDriver.addBuffer<StaticBuffer>(iBuilder, iBuilder->getStreamSetTy(1, 8), this->getDecompressedBufferBlocks());
    Kernel * p2sK = mPxDriver.addKernelInstance<P2SKernel>(iBuilder);
    mPxDriver.makeKernelCall(p2sK, {decompressionBitStream}, {decompressionByteStream});

    Kernel * outK = mPxDriver.addKernelInstance<FileSink>(iBuilder, 8);
    outK->setInitialArguments({iBuilder->GetString("/Users/wxy325/developer/LZ4-sample-files/workspace/lz4d-normal/8k_.txt")});
    mPxDriver.makeKernelCall(outK, {decompressionByteStream}, {});
*/
    kernel::Kernel * matchCountK = mPxDriver.addKernelInstance<kernel::PopcountKernel>(iBuilder);

    mPxDriver.makeKernelCall(matchCountK, {Matches}, {});
    mPxDriver.generatePipelineIR();

    iBuilder->setKernel(matchCountK);
    Value * matchedLineCount = iBuilder->getAccumulator("countResult");
    matchedLineCount = iBuilder->CreateZExt(matchedLineCount, iBuilder->getInt64Ty());
    mPxDriver.deallocateBuffers();

    iBuilder->CreateRet(matchedLineCount);

    mPxDriver.finalizeObject();
}

void LZ4GrepGenerator::generateParallelAioPipeline(re::RE* regex) {
    auto & iBuilder = mPxDriver.getBuilder();
    this->generateCountOnlyMainFunc(iBuilder);

    this->generateLoadByteStream(iBuilder);
    parabix::StreamSetBuffer * decompressedByteStream = this->generateParallelAIODecompression(iBuilder);


    StreamSetBuffer * const decompressionBitStream = mPxDriver.addBuffer<StaticBuffer>(iBuilder, iBuilder->getStreamSetTy(8, 1), this->getDecompressedBufferBlocks(iBuilder));
    Kernel * s2pk = mPxDriver.addKernelInstance<S2PKernel>(iBuilder, /*aligned = */ true, "a");
//    Kernel * s2pk = mPxDriver.addKernelInstance<S2PByPextKernel>(iBuilder, "a");
    mPxDriver.makeKernelCall(s2pk, {decompressedByteStream}, {decompressionBitStream});


    StreamSetBuffer * LineBreakStream;
    StreamSetBuffer * Matches;
    std::vector<re::RE*> res = {regex};
    std::tie(LineBreakStream, Matches) = grepPipeline(res, decompressionBitStream);


//    Kernel * outK = mPxDriver.addKernelInstance<FileSink>(iBuilder, 8);
//    outK->setInitialArguments({iBuilder->GetString("/Users/wxy325/developer/LZ4-sample-files/workspace/lz4d-normal/8k_.txt")});
//    mPxDriver.makeKernelCall(outK, {decompressedByteStream}, {});

    kernel::Kernel * matchCountK = mPxDriver.addKernelInstance<kernel::PopcountKernel>(iBuilder);
    mPxDriver.makeKernelCall(matchCountK, {Matches}, {});
    mPxDriver.generatePipelineIR();

    iBuilder->setKernel(matchCountK);
    Value * matchedLineCount = iBuilder->getAccumulator("countResult");
    matchedLineCount = iBuilder->CreateZExt(matchedLineCount, iBuilder->getInt64Ty());

    mPxDriver.deallocateBuffers();

    iBuilder->CreateRet(matchedLineCount);

    mPxDriver.finalizeObject();
}

void LZ4GrepGenerator::generateAioPipeline(re::RE *regex) {
    auto & iBuilder = mPxDriver.getBuilder();
    this->generateCountOnlyMainFunc(iBuilder);

    // GeneratePipeline
//    this->generateLoadByteStreamAndBitStream(iBuilder);
    this->generateLoadByteStream(iBuilder);
    parabix::StreamSetBuffer * decompressedByteStream = this->generateAIODecompression(iBuilder);


    StreamSetBuffer * const decompressionBitStream = mPxDriver.addBuffer<StaticBuffer>(iBuilder, iBuilder->getStreamSetTy(8, 1), this->getDecompressedBufferBlocks(iBuilder));
    Kernel * s2pk = mPxDriver.addKernelInstance<S2PKernel>(iBuilder, /*aligned = */ true, "a");
//    Kernel * s2pk = mPxDriver.addKernelInstance<S2PByPextKernel>(iBuilder, "a");
    mPxDriver.makeKernelCall(s2pk, {decompressedByteStream}, {decompressionBitStream});


    StreamSetBuffer * LineBreakStream;
    StreamSetBuffer * Matches;
    std::vector<re::RE*> res = {regex};
    std::tie(LineBreakStream, Matches) = grepPipeline(res, decompressionBitStream);


//    Kernel * outK = mPxDriver.addKernelInstance<FileSink>(iBuilder, 8);
//    outK->setInitialArguments({iBuilder->GetString("/Users/wxy325/developer/LZ4-sample-files/workspace/lz4d-normal/8k_.txt")});
//    mPxDriver.makeKernelCall(outK, {decompressedStream}, {});

    kernel::Kernel * matchCountK = mPxDriver.addKernelInstance<kernel::PopcountKernel>(iBuilder);
    mPxDriver.makeKernelCall(matchCountK, {Matches}, {});
    mPxDriver.generatePipelineIR();

    iBuilder->setKernel(matchCountK);
    Value * matchedLineCount = iBuilder->getAccumulator("countResult");
    matchedLineCount = iBuilder->CreateZExt(matchedLineCount, iBuilder->getInt64Ty());

    mPxDriver.deallocateBuffers();

    iBuilder->CreateRet(matchedLineCount);

    mPxDriver.finalizeObject();

}


void LZ4GrepGenerator::generateCountOnlyGrepPipeline(re::RE *regex, bool enableGather) {
    auto & iBuilder = mPxDriver.getBuilder();
    this->generateCountOnlyMainFunc(iBuilder);

    // GeneratePipeline
    this->generateLoadByteStreamAndBitStream(iBuilder);
    this->generateExtractAndDepositMarkers(iBuilder);

    StreamSetBuffer * LineBreakStream;
    StreamSetBuffer * Matches;
    std::vector<re::RE*> res = {regex};
    if (mEnableMultiplexing) {
        std::tie(LineBreakStream, Matches) = multiplexingGrepPipeline(res);
    } else {
        StreamSetBuffer * const extractedBits = this->generateBitStreamExtractData(iBuilder);
        StreamSetBuffer * depositedBits = mPxDriver.addBuffer<StaticBuffer>(iBuilder, iBuilder->getStreamSetTy(8), this->getDecompressedBufferBlocks(iBuilder));
        Kernel * bitStreamPDEPk = enableGather ? mPxDriver.addKernelInstance<BitStreamGatherPDEPKernel>(iBuilder, 8) : mPxDriver.addKernelInstance<BitStreamPDEPKernel>(iBuilder, 8);
        mPxDriver.makeKernelCall(bitStreamPDEPk, {mDepositMarker, extractedBits}, {depositedBits});

        StreamSetBuffer * matchCopiedBits = mPxDriver.addBuffer<StaticBuffer>(iBuilder, iBuilder->getStreamSetTy(8), this->getInputBufferBlocks(iBuilder));
        Kernel * bitStreamMatchCopyK = mPxDriver.addKernelInstance<LZ4BitStreamMatchCopyKernel>(iBuilder, 8);
        mPxDriver.makeKernelCall(bitStreamMatchCopyK, {mMatchOffsetMarker, mM0Marker, mCompressedByteStream, depositedBits}, {matchCopiedBits});

        std::tie(LineBreakStream, Matches) = grepPipeline(res, matchCopiedBits);
    };

    kernel::Kernel * matchCountK = mPxDriver.addKernelInstance<kernel::PopcountKernel>(iBuilder);
    mPxDriver.makeKernelCall(matchCountK, {Matches}, {});
    mPxDriver.generatePipelineIR();

    iBuilder->setKernel(matchCountK);
    Value * matchedLineCount = iBuilder->getAccumulator("countResult");
    matchedLineCount = iBuilder->CreateZExt(matchedLineCount, iBuilder->getInt64Ty());

    mPxDriver.deallocateBuffers();

    iBuilder->CreateRet(matchedLineCount);

    mPxDriver.finalizeObject();
}


void LZ4GrepGenerator::generateSwizzledCountOnlyGrepPipeline(re::RE *regex) {
    auto & iBuilder = mPxDriver.getBuilder();
    this->generateCountOnlyMainFunc(iBuilder);


    // GeneratePipeline
    this->generateLoadByteStreamAndBitStream(iBuilder);
    this->generateExtractAndDepositMarkers(iBuilder);


    StreamSetBuffer * LineBreakStream;
    StreamSetBuffer * Matches;
    std::vector<re::RE*> res = {regex};
    if (mEnableMultiplexing) {
        std::tie(LineBreakStream, Matches) = multiplexingGrepPipeline(res);
    } else {
        auto swizzle = this->generateSwizzleExtractData(iBuilder);

        StreamSetBuffer * depositedSwizzle0 = mPxDriver.addBuffer<StaticBuffer>(iBuilder, iBuilder->getStreamSetTy(4), this->getInputBufferBlocks(iBuilder), 1);
        StreamSetBuffer * depositedSwizzle1 = mPxDriver.addBuffer<StaticBuffer>(iBuilder, iBuilder->getStreamSetTy(4), this->getInputBufferBlocks(iBuilder), 1);

        Kernel * multiplePdepK = mPxDriver.addKernelInstance<SwizzledMultiplePDEPkernel>(iBuilder, 4, 2);
        mPxDriver.makeKernelCall(multiplePdepK, {mDepositMarker, swizzle.first, swizzle.second}, {depositedSwizzle0, depositedSwizzle1});


        // split PDEP into 2 kernel will be a little slower in single thread environment
/*
    Kernel * pdep1 = mPxDriver.addKernelInstance<PDEPkernel>(iBuilder, 4);
    mPxDriver.makeKernelCall(pdep1, {mDepositMarker, swizzle.first}, {depositedSwizzle0});

    Kernel * pdep2 = mPxDriver.addKernelInstance<PDEPkernel>(iBuilder, 4);
    mPxDriver.makeKernelCall(pdep2, {mDepositMarker, swizzle.second}, {depositedSwizzle1});
*/

        StreamSetBuffer * matchCopiedSwizzle0 = mPxDriver.addBuffer<StaticBuffer>(iBuilder, iBuilder->getStreamSetTy(4), this->getInputBufferBlocks(iBuilder), 1);
        StreamSetBuffer * matchCopiedSwizzle1 = mPxDriver.addBuffer<StaticBuffer>(iBuilder, iBuilder->getStreamSetTy(4), this->getInputBufferBlocks(iBuilder), 1);

        Kernel * swizzledMatchCopyK = mPxDriver.addKernelInstance<LZ4SwizzledMatchCopyKernel>(iBuilder, 4, 2, 4);
        mPxDriver.makeKernelCall(swizzledMatchCopyK, {mMatchOffsetMarker, mM0Marker, mCompressedByteStream, depositedSwizzle0, depositedSwizzle1}, {matchCopiedSwizzle0, matchCopiedSwizzle1});

        // Produce unswizzled bit streams
        StreamSetBuffer * matchCopiedbits = mPxDriver.addBuffer<StaticBuffer>(iBuilder, iBuilder->getStreamSetTy(8), this->getInputBufferBlocks(iBuilder));
        Kernel * unSwizzleK = mPxDriver.addKernelInstance<SwizzleGenerator>(iBuilder, 8, 1, 2);
        mPxDriver.makeKernelCall(unSwizzleK, {matchCopiedSwizzle0, matchCopiedSwizzle1}, {matchCopiedbits});


        std::tie(LineBreakStream, Matches) = grepPipeline(res, matchCopiedbits);
    };

    kernel::Kernel * matchCountK = mPxDriver.addKernelInstance<kernel::PopcountKernel>(iBuilder);
    mPxDriver.makeKernelCall(matchCountK, {Matches}, {});
    mPxDriver.generatePipelineIR();

    iBuilder->setKernel(matchCountK);
    Value * matchedLineCount = iBuilder->getAccumulator("countResult");
    matchedLineCount = iBuilder->CreateZExt(matchedLineCount, iBuilder->getInt64Ty());

    mPxDriver.deallocateBuffers();

    iBuilder->CreateRet(matchedLineCount);

    mPxDriver.finalizeObject();
}

ScanMatchGrepMainFunctionType LZ4GrepGenerator::getScanMatchGrepMainFunction() {
    return reinterpret_cast<ScanMatchGrepMainFunctionType>(mPxDriver.getMain());
}
CountOnlyGrepMainFunctionType LZ4GrepGenerator::getCountOnlyGrepMainFunction() {
    return reinterpret_cast<CountOnlyGrepMainFunctionType>(mPxDriver.getMain());
}

void LZ4GrepGenerator::generateCountOnlyMainFunc(const std::unique_ptr<kernel::KernelBuilder> & iBuilder) {
    Module * M = iBuilder->getModule();
    Type * const int64Ty = iBuilder->getInt64Ty();
    Type * const sizeTy = iBuilder->getSizeTy();
    Type * const boolTy = iBuilder->getIntNTy(sizeof(bool) * 8);
//    Type * const voidTy = iBuilder->getVoidTy();
    Type * const inputType = iBuilder->getInt8PtrTy();

    Function * const main = cast<Function>(M->getOrInsertFunction("Main", int64Ty, inputType, sizeTy, sizeTy, boolTy, nullptr));
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

void LZ4GrepGenerator::generateScanMatchMainFunc(const std::unique_ptr<kernel::KernelBuilder> & iBuilder) {
    Module * M = iBuilder->getModule();
    Type * const sizeTy = iBuilder->getSizeTy();
    Type * const boolTy = iBuilder->getIntNTy(sizeof(bool) * 8);
    Type * const voidTy = iBuilder->getVoidTy();
    Type * const inputType = iBuilder->getInt8PtrTy();
    Type * const intAddrTy = iBuilder->getIntAddrTy();

    Function * const main = cast<Function>(M->getOrInsertFunction("Main", voidTy, inputType, sizeTy, sizeTy, boolTy, intAddrTy, nullptr));
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

    match_accumulator = &*(args++);
    match_accumulator->setName("match_accumulator");

    iBuilder->SetInsertPoint(BasicBlock::Create(M->getContext(), "entry", main, 0));
}
