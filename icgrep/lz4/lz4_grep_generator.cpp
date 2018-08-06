
#include "lz4_grep_generator.h"

#include <boost/iostreams/device/mapped_file.hpp>

#include <llvm/Support/PrettyStackTrace.h>

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
#include <kernels/fake_stream_generating_kernel.h>
#include <kernels/bitstream_pdep_kernel.h>
#include <kernels/bitstream_gather_pdep_kernel.h>
#include <re/re_toolchain.h>

#include <re/collect_ccs.h>
#include <re/replaceCC.h>

#include <re/casing.h>
#include <re/exclude_CC.h>
#include <re/to_utf8.h>
#include <re/re_analysis.h>
#include <re/re_name_resolve.h>
#include <re/re_name_gather.h>
#include <re/re_multiplex.h>
#include <re/re_utility.h>

#include <UCD/resolve_properties.h>
#include <kernels/charclasses.h>
#include <kernels/grep_kernel.h>
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
#include <kernels/lz4/decompression/lz4_swizzled_decompression.h>
#include <kernels/lz4/decompression/lz4_bitstream_decompression.h>
#include <re/re_seq.h>
#include <kernels/lz4/decompression/lz4_bytestream_decompression.h>
#include <kernels/lz4/twist_kernel.h>
#include <kernels/lz4/decompression/lz4_twist_decompression.h>
#include <kernels/lz4/untwist_kernel.h>

namespace re { class CC; }

using namespace llvm;
using namespace parabix;
using namespace kernel;
using namespace grep;

LZ4GrepGenerator::LZ4GrepGenerator(bool enableMultiplexing): LZ4BaseGenerator(), mEnableMultiplexing(enableMultiplexing) {
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


parabix::StreamSetBuffer * LZ4GrepGenerator::linefeedStreamFromUncompressedBits(
        parabix::StreamSetBuffer *uncompressedBasisBits) {
    auto & idb = mPxDriver.getBuilder();
    const unsigned baseBufferSize = this->getDefaultBufferBlocks();
    StreamSetBuffer * LineFeedStream = mPxDriver.addBuffer<StaticBuffer>(idb, idb->getStreamSetTy(1, 1), baseBufferSize);
    kernel::Kernel * linefeedK = mPxDriver.addKernelInstance<kernel::LineFeedKernelBuilder>(idb, Binding{idb->getStreamSetTy(8), "basis", FixedRate(), Principal()}, cc::BitNumbering::BigEndian);
    mPxDriver.makeKernelCall(linefeedK, {uncompressedBasisBits}, {LineFeedStream});
    return LineFeedStream;
}

parabix::StreamSetBuffer * LZ4GrepGenerator::convertCompressedBitsStreamWithTwistApproach(
        parabix::StreamSetBuffer *compressedByteStream,
        parabix::StreamSetBuffer *compressedBitStream,
        std::string prefix
) {
    auto & b = mPxDriver.getBuilder();

    LZ4BlockInfo blockInfo = this->getBlockInfo(compressedByteStream);

    unsigned numOfStreams = compressedBitStream->getNumOfStreams();

    if (numOfStreams == 1) {
        StreamSetBuffer* uncompressedBitStream = mPxDriver.addBuffer<StaticBuffer>(b, b->getStreamSetTy(1, 1),
                                                                                            this->getDefaultBufferBlocks());
        Kernel* lz4I4AioK = mPxDriver.addKernelInstance<LZ4TwistDecompressionKernel>(b, 1);
        lz4I4AioK->setInitialArguments({mFileSize});
        mPxDriver.makeKernelCall(lz4I4AioK, {
                compressedByteStream,

                blockInfo.isCompress,
                blockInfo.blockStart,
                blockInfo.blockEnd,

                compressedBitStream
        }, {
                uncompressedBitStream
                                 });
        return uncompressedBitStream;
    }

    if (numOfStreams <= 2) {
        StreamSetBuffer* twistedCharClasses = mPxDriver.addBuffer<StaticBuffer>(b, b->getStreamSetTy(1, 2),
                                                                                this->getDefaultBufferBlocks());
        kernel::Kernel* twistK = mPxDriver.addKernelInstance<kernel::TwistByPDEPKernel>(b, numOfStreams, 2);
        mPxDriver.makeKernelCall(twistK, {compressedBitStream}, {twistedCharClasses});


        StreamSetBuffer* uncompressedTwistedCharClasses = mPxDriver.addBuffer<StaticBuffer>(b, b->getStreamSetTy(1, 2),
                                                                                            this->getDefaultBufferBlocks());
        Kernel* lz4I4AioK = mPxDriver.addKernelInstance<LZ4TwistDecompressionKernel>(b, 2);
        lz4I4AioK->setInitialArguments({mFileSize});
        mPxDriver.makeKernelCall(lz4I4AioK, {
                compressedByteStream,

                blockInfo.isCompress,
                blockInfo.blockStart,
                blockInfo.blockEnd,

                twistedCharClasses
        }, {
                                         uncompressedTwistedCharClasses
                                 });

        StreamSetBuffer* untwistedCharClasses = mPxDriver.addBuffer<StaticBuffer>(b, b->getStreamSetTy(numOfStreams),
                                                                                  this->getDefaultBufferBlocks());
        kernel::Kernel* untwistK = mPxDriver.addKernelInstance<kernel::UntwistByPEXTKernel>(b, numOfStreams, 2);
        mPxDriver.makeKernelCall(untwistK, {uncompressedTwistedCharClasses}, {untwistedCharClasses});
        return untwistedCharClasses;
    }

    if (numOfStreams <= 4) {
        StreamSetBuffer* twistedCharClasses = mPxDriver.addBuffer<StaticBuffer>(b, b->getStreamSetTy(1, 4),
                                                                                this->getDefaultBufferBlocks());
        kernel::Kernel* twistK = mPxDriver.addKernelInstance<kernel::TwistByPDEPKernel>(b, numOfStreams, 4);
        mPxDriver.makeKernelCall(twistK, {compressedBitStream}, {twistedCharClasses});


        StreamSetBuffer* uncompressedTwistedCharClasses = mPxDriver.addBuffer<StaticBuffer>(b, b->getStreamSetTy(1, 4),
                                                                                            this->getDefaultBufferBlocks());

        Kernel* lz4I4AioK = mPxDriver.addKernelInstance<LZ4TwistDecompressionKernel>(b, 4);
        lz4I4AioK->setInitialArguments({mFileSize});
        mPxDriver.makeKernelCall(lz4I4AioK, {
                compressedByteStream,

                blockInfo.isCompress,
                blockInfo.blockStart,
                blockInfo.blockEnd,

                twistedCharClasses
        }, {
                                         uncompressedTwistedCharClasses
                                 });

        StreamSetBuffer* untwistedCharClasses = mPxDriver.addBuffer<StaticBuffer>(b, b->getStreamSetTy(numOfStreams),
                                                                                  this->getDefaultBufferBlocks());
        kernel::Kernel* untwistK = mPxDriver.addKernelInstance<kernel::UntwistByPEXTKernel>(b, numOfStreams, 4);
        mPxDriver.makeKernelCall(untwistK, {uncompressedTwistedCharClasses}, {untwistedCharClasses});
        return untwistedCharClasses;
    }

    // <= 8
    StreamSetBuffer * const mtxByteStream = mPxDriver.addBuffer<StaticBuffer>(b, b->getStreamSetTy(1, 8),
                                                                              this->getDefaultBufferBlocks());
    Kernel * p2sK = mPxDriver.addKernelInstance<P2SKernel>(b, cc::BitNumbering::BigEndian, prefix, numOfStreams);
    mPxDriver.makeKernelCall(p2sK, {compressedBitStream}, {mtxByteStream});

    StreamSetBuffer * const decompressionMtxByteStream = mPxDriver.addBuffer<StaticBuffer>(b, b->getStreamSetTy(1, 8),
                                                                                           this->getDefaultBufferBlocks(), 1);
    Kernel* lz4AioK = mPxDriver.addKernelInstance<LZ4ByteStreamDecompressionKernel>(b, true);
    lz4AioK->setInitialArguments({mFileSize});
    mPxDriver.makeKernelCall(
            lz4AioK,
            {
                    compressedByteStream,
                    // Block Data
                    blockInfo.isCompress,
                    blockInfo.blockStart,
                    blockInfo.blockEnd,
                    mtxByteStream
            }, {
                    decompressionMtxByteStream
            });

    StreamSetBuffer * const uncompressedMtxBitStream = mPxDriver.addBuffer<StaticBuffer>(b, b->getStreamSetTy(numOfStreams),
                                                                                         this->getDefaultBufferBlocks());

    Kernel * s2pk = mPxDriver.addKernelInstance<S2PKernel>(b, cc::BitNumbering::BigEndian, true, prefix, numOfStreams);
    mPxDriver.makeKernelCall(s2pk, {decompressionMtxByteStream}, {uncompressedMtxBitStream});
    return uncompressedMtxBitStream;
}

StreamSetBuffer * LZ4GrepGenerator::convertCompressedBitsStreamWithSwizzledAioApproach(
        StreamSetBuffer *compressedByteStream,
        StreamSetBuffer *compressedBitStream,
        std::string prefix
) {
    auto mGrepDriver = &mPxDriver;
    auto & b = mGrepDriver->getBuilder();

    LZ4BlockInfo blockInfo = this->getBlockInfo(compressedByteStream);

    // Produce unswizzled bit streams
    StreamSetBuffer * u16Swizzle0 = mPxDriver.addBuffer<StaticBuffer>(b, b->getStreamSetTy(4),
                                                                      this->getDefaultBufferBlocks(), 1);
    Kernel * unSwizzleK = mPxDriver.addKernelInstance<SwizzleGenerator>(b, 4, 1, 1, 64, "source");
    mPxDriver.makeKernelCall(unSwizzleK, {compressedBitStream}, {u16Swizzle0});

    StreamSetBuffer * uncompressedSwizzled0 = mPxDriver.addBuffer<StaticBuffer>(b, b->getStreamSetTy(4),
                                                                                this->getDefaultBufferBlocks(), 1);


    Kernel* lz4AioK = mPxDriver.addKernelInstance<LZ4SwizzledDecompressionKernel>(b, 4, 1, 4);
    lz4AioK->setInitialArguments({mFileSize});
    mPxDriver.makeKernelCall(
            lz4AioK,
            {
                    compressedByteStream,

                    // Block Data
                    blockInfo.isCompress,
                    blockInfo.blockStart,
                    blockInfo.blockEnd,

                    u16Swizzle0,
            }, {
                    uncompressedSwizzled0,
            });



    StreamSetBuffer * const decompressionBitStream = mPxDriver.addBuffer<StaticBuffer>(b, b->getStreamSetTy(8, 1),
                                                                                       this->getDefaultBufferBlocks());
    Kernel * unSwizzleK2 = mPxDriver.addKernelInstance<SwizzleGenerator>(b, 4, 1, 1, 64, "dst");
    mPxDriver.makeKernelCall(unSwizzleK2, {uncompressedSwizzled0}, {decompressionBitStream});

    return decompressionBitStream;

}


std::pair<parabix::StreamSetBuffer *, parabix::StreamSetBuffer *> LZ4GrepGenerator::multiplexingGrepPipeline(
        std::vector<re::RE *> &REs,
        parabix::StreamSetBuffer *compressedByteStream,
        parabix::StreamSetBuffer *compressedBitStream,
        bool useSwizzled,
        bool useByteStream
) {

    this->initREs(REs);
    auto mGrepDriver = &mPxDriver;

    auto & idb = mGrepDriver->getBuilder();
    const unsigned baseBufferSize = this->getDefaultBufferBlocks();
    int MaxCountFlag = 0;

    //  Regular Expression Processing and Analysis Phase
    const auto nREs = mREs.size();

    std::vector<StreamSetBuffer *> MatchResultsBufs(nREs);


    std::map<std::string, StreamSetBuffer *> propertyStream;

    std::vector<std::string> externalStreamNames;
    std::set<re::Name *> UnicodeProperties;

    re::CC* linefeedCC = re::makeByte(0x0A);

    re::Seq* seq = re::makeSeq();
    re::RE* targetRe = mREs[0];
    re::RE* utf8Re = re::toUTF8(targetRe, true);


    seq->push_back(utf8Re);
    seq->push_back(std::move(linefeedCC));

    std::vector<re::CC*> UnicodeSets = re::collectCCs(seq, &cc::Byte, std::set<re::Name *>({re::makeZeroWidth("\\b{g}")}));

    StreamSetBuffer * const MatchResults = mGrepDriver->addBuffer<StaticBuffer>(idb, idb->getStreamSetTy(1, 1), baseBufferSize);

    mpx = make_unique<cc::MultiplexedAlphabet>("mpx", UnicodeSets);
    re::RE* r = transformCCs(mpx.get(), utf8Re);
    mREs[0] = r;


    std::vector<re::CC *> mpx_basis = mpx->getMultiplexedCCs();
    auto numOfCharacterClasses = mpx_basis.size();
    StreamSetBuffer * CharClasses = mGrepDriver->addBuffer<StaticBuffer>(idb, idb->getStreamSetTy(numOfCharacterClasses), baseBufferSize);


    kernel::Kernel * ccK = mGrepDriver->addKernelInstance<kernel::ByteClassesKernel>(idb, std::move(mpx_basis), false, cc::BitNumbering::BigEndian);
    mGrepDriver->makeKernelCall(ccK, {compressedBitStream}, {CharClasses});

    StreamSetBuffer * uncompressedCharClasses = nullptr;
    if (useSwizzled) {
        uncompressedCharClasses = this->convertCompressedBitsStreamWithSwizzledAioApproach(compressedByteStream, CharClasses, "combined");
    } else if (useByteStream){
        uncompressedCharClasses = this->convertCompressedBitsStreamWithTwistApproach(compressedByteStream, CharClasses,
                                                                                     "combined");
    } else {
        auto ret = this->convertCompressedBitsStreamWithBitStreamAioApproach(compressedByteStream, {CharClasses});
        uncompressedCharClasses = ret[0];
    }

    StreamSetBuffer * fakeMatchCopiedBits = mPxDriver.addBuffer<StaticBuffer>(idb, idb->getStreamSetTy(8),
                                                                              this->getDefaultBufferBlocks());
    StreamSetBuffer * u8NoFinalStream = mPxDriver.addBuffer<StaticBuffer>(idb, idb->getStreamSetTy(1),
                                                                          this->getDefaultBufferBlocks(), 1);

    Kernel* fakeStreamGeneratorK = mPxDriver.addKernelInstance<FakeStreamGeneratingKernel>(idb, numOfCharacterClasses, std::vector<unsigned>({8, 1}));
    mPxDriver.makeKernelCall(fakeStreamGeneratorK, {uncompressedCharClasses}, {fakeMatchCopiedBits, u8NoFinalStream});

    StreamSetBuffer * LineBreakStream = mPxDriver.addBuffer<StaticBuffer>(idb, idb->getStreamSetTy(1, 1), this->getDefaultBufferBlocks());
    kernel::Kernel * lineFeedGrepK = mGrepDriver->addKernelInstance<kernel::ICGrepKernel>(idb, transformCCs(mpx.get(), linefeedCC), externalStreamNames, std::vector<cc::Alphabet *>{mpx.get()}, cc::BitNumbering::BigEndian);
    mGrepDriver->makeKernelCall(lineFeedGrepK, {fakeMatchCopiedBits, uncompressedCharClasses}, {LineBreakStream});


    externalStreamNames.push_back("UTF8_nonfinal");

    kernel::Kernel * icgrepK = mGrepDriver->addKernelInstance<kernel::ICGrepKernel>(idb, mREs[0], externalStreamNames, std::vector<cc::Alphabet *>{mpx.get()}, cc::BitNumbering::BigEndian);
    mGrepDriver->makeKernelCall(icgrepK, {fakeMatchCopiedBits, u8NoFinalStream, uncompressedCharClasses}, {MatchResults});
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
        std::vector<re::RE *> &REs, parabix::StreamSetBuffer *uncompressedBasisBits) {

    this->initREs(REs);
    auto mGrepDriver = &mPxDriver;

    auto & idb = mGrepDriver->getBuilder();
    // TODO: until we automate stream buffer sizing, use this calculation to determine how large our matches buffer needs to be.
    const unsigned baseBufferSize = this->getDefaultBufferBlocks();
    int MaxCountFlag = 0;

    //  Regular Expression Processing and Analysis Phase
    const auto nREs = mREs.size();

    std::vector<StreamSetBuffer *> MatchResultsBufs(nREs);

    StreamSetBuffer * LineBreakStream = this->linefeedStreamFromUncompressedBits(uncompressedBasisBits);


    std::map<std::string, StreamSetBuffer *> propertyStream;

    for(unsigned i = 0; i < nREs; ++i) {
        std::vector<std::string> externalStreamNames;
        std::vector<StreamSetBuffer *> icgrepInputSets = {uncompressedBasisBits};

        if (mEnableMultiplexing) {
            const auto UnicodeSets = re::collectCCs(mREs[i], &cc::Unicode, std::set<re::Name *>({re::makeZeroWidth("\\b{g}")}));
            StreamSetBuffer * const MatchResults = mGrepDriver->addBuffer<StaticBuffer>(idb, idb->getStreamSetTy(1, 1), baseBufferSize);

            mpx = make_unique<cc::MultiplexedAlphabet>("mpx", UnicodeSets);
            mREs[i] = transformCCs(mpx.get(), mREs[i]);
            std::vector<re::CC *> mpx_basis = mpx->getMultiplexedCCs();
            auto numOfCharacterClasses = mpx_basis.size();
            StreamSetBuffer * CharClasses = mGrepDriver->addBuffer<StaticBuffer>(idb, idb->getStreamSetTy(numOfCharacterClasses), baseBufferSize);
            kernel::Kernel * ccK = mGrepDriver->addKernelInstance<kernel::CharClassesKernel>(idb, std::move(mpx_basis), false, cc::BitNumbering::BigEndian);
            mGrepDriver->makeKernelCall(ccK, {uncompressedBasisBits}, {CharClasses});

            kernel::Kernel * icgrepK = mGrepDriver->addKernelInstance<kernel::ICGrepKernel>(idb, mREs[i], externalStreamNames, std::vector<cc::Alphabet *>{mpx.get()}, cc::BitNumbering::BigEndian);
            icgrepInputSets.push_back(CharClasses);
            mGrepDriver->makeKernelCall(icgrepK, icgrepInputSets, {MatchResults});
            MatchResultsBufs[i] = MatchResults;
        } else {
            std::set<re::Name *> UnicodeProperties;

            StreamSetBuffer * MatchResults = mGrepDriver->addBuffer<StaticBuffer>(idb, idb->getStreamSetTy(1, 1), baseBufferSize);
            kernel::Kernel * icgrepK = mGrepDriver->addKernelInstance<kernel::ICGrepKernel>(idb, mREs[i], externalStreamNames, std::vector<cc::Alphabet *>(), cc::BitNumbering::BigEndian);
            mGrepDriver->makeKernelCall(icgrepK, icgrepInputSets, {MatchResults});
            MatchResultsBufs[i] = MatchResults;
        }
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

    StreamSetBuffer* compressedByteStream = this->loadByteStream();

    StreamSetBuffer * const uncompressedByteStream = this->byteStreamDecompression(compressedByteStream);

    StreamSetBuffer * uncompressedBitStream = mPxDriver.addBuffer<StaticBuffer>(iBuilder, iBuilder->getStreamSetTy(8),
                                                                                this->getDefaultBufferBlocks());
    Kernel * s2pk = mPxDriver.addKernelInstance<S2PKernel>(iBuilder, cc::BitNumbering::BigEndian);
    mPxDriver.makeKernelCall(s2pk, {uncompressedByteStream}, {uncompressedBitStream});

    StreamSetBuffer * LineBreakStream;
    StreamSetBuffer * Matches;
    std::vector<re::RE*> res = {regex};
    std::tie(LineBreakStream, Matches) = grepPipeline(res, uncompressedBitStream);

    kernel::Kernel * scanMatchK = mPxDriver.addKernelInstance<kernel::ScanMatchKernel>(iBuilder);
    scanMatchK->setInitialArguments({match_accumulator});
    mPxDriver.makeKernelCall(scanMatchK, {Matches, LineBreakStream, uncompressedByteStream}, {});
    mPxDriver.LinkFunction(*scanMatchK, "accumulate_match_wrapper", &accumulate_match_wrapper);
    mPxDriver.LinkFunction(*scanMatchK, "finalize_match_wrapper", &finalize_match_wrapper);

    mPxDriver.generatePipelineIR();
    mPxDriver.deallocateBuffers();

    iBuilder->CreateRetVoid();

    mPxDriver.finalizeObject();
}

void LZ4GrepGenerator::generateMultiplexingSwizzledAioPipeline(re::RE *regex) {
    auto & iBuilder = mPxDriver.getBuilder();
    this->generateCountOnlyMainFunc(iBuilder);

    // GeneratePipeline
    StreamSetBuffer *compressedByteStream = nullptr, *compressedBasisBits = nullptr;
    std::tie(compressedByteStream, compressedBasisBits) = this->loadByteStreamAndBitStream();

    StreamSetBuffer * LineBreakStream;
    StreamSetBuffer * Matches;
    std::vector<re::RE*> res = {regex};
    std::tie(LineBreakStream, Matches) = multiplexingGrepPipeline(res, compressedByteStream, compressedBasisBits, true);

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

void LZ4GrepGenerator::generateByteStreamMultiplexingAioPipeline(re::RE* regex) {
    auto & iBuilder = mPxDriver.getBuilder();
    this->generateCountOnlyMainFunc(iBuilder);

    StreamSetBuffer *compressedByteStream = nullptr, *compressedBasisBits = nullptr;
    std::tie(compressedByteStream, compressedBasisBits) = this->loadByteStreamAndBitStream();

    StreamSetBuffer * LineBreakStream;
    StreamSetBuffer * Matches;
    std::vector<re::RE*> res = {regex};
    std::tie(LineBreakStream, Matches) = multiplexingGrepPipeline(res, compressedByteStream, compressedBasisBits, false, true);

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


void LZ4GrepGenerator::generateMultiplexingBitStreamAioPipeline(re::RE* regex) {
    auto & iBuilder = mPxDriver.getBuilder();
    this->generateCountOnlyMainFunc(iBuilder);

    StreamSetBuffer *compressedByteStream = nullptr, *compressedBasisBits = nullptr;
    std::tie(compressedByteStream, compressedBasisBits) = this->loadByteStreamAndBitStream();

    StreamSetBuffer * LineBreakStream;
    StreamSetBuffer * Matches;
    std::vector<re::RE*> res = {regex};
    std::tie(LineBreakStream, Matches) = multiplexingGrepPipeline(res, compressedByteStream, compressedBasisBits, false);

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

void LZ4GrepGenerator::generateBitStreamAioPipeline(re::RE* regex) {
    auto & iBuilder = mPxDriver.getBuilder();
    this->generateCountOnlyMainFunc(iBuilder);

    StreamSetBuffer *compressedByteStream = nullptr, *compressedBasisBits = nullptr;
    std::tie(compressedByteStream, compressedBasisBits) = this->loadByteStreamAndBitStream();

    StreamSetBuffer * const decompressionBitStream = this->bitStreamDecompression(compressedByteStream, compressedBasisBits);

    StreamSetBuffer * LineBreakStream;
    StreamSetBuffer * Matches;
    std::vector<re::RE*> res = {regex};
    std::tie(LineBreakStream, Matches) = grepPipeline(res, decompressionBitStream);

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

    StreamSetBuffer *compressedByteStream = nullptr, *compressedBasisBits = nullptr;
    std::tie(compressedByteStream, compressedBasisBits) = this->loadByteStreamAndBitStream();

    StreamSetBuffer * const decompressionBitStream = this->swizzledDecompression(compressedByteStream, compressedBasisBits);

    StreamSetBuffer * LineBreakStream;
    StreamSetBuffer * Matches;
    std::vector<re::RE*> res = {regex};
    std::tie(LineBreakStream, Matches) = grepPipeline(res, decompressionBitStream);

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

void LZ4GrepGenerator::generateParallelAioPipeline(re::RE* regex, bool enableGather, bool enableScatter, int minParallelLevel) {
    auto & iBuilder = mPxDriver.getBuilder();
    this->generateCountOnlyMainFunc(iBuilder);

    StreamSetBuffer* compressedByteStream = this->loadByteStream();
    parabix::StreamSetBuffer * uncompressedByteStream = this->parallelByteStreamDecompression(compressedByteStream, enableGather,
                                                                                              enableScatter,
                                                                                              minParallelLevel);


    StreamSetBuffer * const uncompressedBitStream = mPxDriver.addBuffer<StaticBuffer>(iBuilder, iBuilder->getStreamSetTy(8, 1),
                                                                                       this->getDefaultBufferBlocks());
    Kernel * s2pk = mPxDriver.addKernelInstance<S2PKernel>(iBuilder, cc::BitNumbering::BigEndian, /*aligned = */ true, "a");
    mPxDriver.makeKernelCall(s2pk, {uncompressedByteStream}, {uncompressedBitStream});


    StreamSetBuffer * LineBreakStream;
    StreamSetBuffer * Matches;
    std::vector<re::RE*> res = {regex};
    std::tie(LineBreakStream, Matches) = grepPipeline(res, uncompressedBitStream);


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
    StreamSetBuffer* compressedByteStream = this->loadByteStream();
//    this->loadByteStreamAndBitStream(iBuilder);

    parabix::StreamSetBuffer * uncompressedByteStream = this->byteStreamDecompression(compressedByteStream);


    StreamSetBuffer * const uncompressedBitStream = mPxDriver.addBuffer<StaticBuffer>(iBuilder, iBuilder->getStreamSetTy(8, 1),
                                                                                       this->getDefaultBufferBlocks());
    Kernel * s2pk = mPxDriver.addKernelInstance<S2PKernel>(iBuilder, cc::BitNumbering::BigEndian, /*aligned = */ true, "a");
    mPxDriver.makeKernelCall(s2pk, {uncompressedByteStream}, {uncompressedBitStream});


    StreamSetBuffer * LineBreakStream;
    StreamSetBuffer * Matches;

    std::vector<re::RE*> res = {regex};
    std::tie(LineBreakStream, Matches) = grepPipeline(res, uncompressedBitStream);

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
