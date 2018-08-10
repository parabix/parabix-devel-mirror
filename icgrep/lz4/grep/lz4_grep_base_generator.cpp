
#include "lz4_grep_base_generator.h"

#include <boost/iostreams/device/mapped_file.hpp>

#include <llvm/Support/PrettyStackTrace.h>

#include <cc/cc_compiler.h>

#include <kernels/cc_kernel.h>
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

#include <re/re_seq.h>
#include <kernels/kernel_builder.h>

namespace re { class CC; }

using namespace llvm;
using namespace parabix;
using namespace kernel;
using namespace grep;

LZ4GrepBaseGenerator::LZ4GrepBaseGenerator(): LZ4BaseGenerator() {
    mGrepRecordBreak = grep::GrepRecordBreakKind::LF;
    mMoveMatchesToEOL = true;
}



void LZ4GrepBaseGenerator::generateScanMatchGrepPipeline(re::RE* regex) {
    auto & iBuilder = mPxDriver.getBuilder();
    this->generateScanMatchMainFunc(iBuilder);

    StreamSetBuffer* compressedByteStream = this->loadByteStream();

    StreamSetBuffer * const uncompressedByteStream = this->byteStreamDecompression(compressedByteStream);
    StreamSetBuffer * uncompressedBitStream = this->s2p(uncompressedByteStream);

    StreamSetBuffer * LineBreakStream;
    StreamSetBuffer * Matches;
    std::tie(LineBreakStream, Matches) = grep(regex, uncompressedBitStream);

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


void LZ4GrepBaseGenerator::generateCountOnlyGrepPipeline(re::RE* regex, bool enableMultiplexing, bool utf8CC){
    if (enableMultiplexing) {
        this->generateMultiplexingCountOnlyGrepPipeline(regex, utf8CC);
    } else {
        this->generateFullyDecompressionCountOnlyGrepPipeline(regex);
    }
}


void LZ4GrepBaseGenerator::initREs(re::RE * RE) {
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

    mRE = RE;
    bool allAnchored = true;

    if (!hasEndAnchor(mRE)) allAnchored = false;
    mRE = resolveModesAndExternalSymbols(mRE);
    mRE = re::exclude_CC(mRE, mBreakCC);
    mRE = resolveAnchors(mRE, anchorRE);
    re::gatherUnicodeProperties(mRE, mUnicodeProperties);
    mRE = regular_expression_passes(mRE);

    if (allAnchored && (mGrepRecordBreak != GrepRecordBreakKind::Unicode)) mMoveMatchesToEOL = false;

}


parabix::StreamSetBuffer * LZ4GrepBaseGenerator::linefeedStreamFromUncompressedBits(
        parabix::StreamSetBuffer *uncompressedBasisBits) {
    auto & idb = mPxDriver.getBuilder();
    const unsigned baseBufferSize = this->getDefaultBufferBlocks();
    StreamSetBuffer * LineFeedStream = mPxDriver.addBuffer<StaticBuffer>(idb, idb->getStreamSetTy(1, 1), baseBufferSize, 1);
    kernel::Kernel * linefeedK = mPxDriver.addKernelInstance<kernel::LineFeedKernelBuilder>(idb, Binding{idb->getStreamSetTy(8), "basis", FixedRate(), Principal()}, cc::BitNumbering::BigEndian);
    mPxDriver.makeKernelCall(linefeedK, {uncompressedBasisBits}, {LineFeedStream});
    return LineFeedStream;
}






std::pair<parabix::StreamSetBuffer *, parabix::StreamSetBuffer *> LZ4GrepBaseGenerator::multiplexingGrep(
        re::RE *RE,
        parabix::StreamSetBuffer *compressedByteStream,
        parabix::StreamSetBuffer *compressedBitStream,
        bool utf8CC
) {

    this->initREs(RE);
    auto mGrepDriver = &mPxDriver;

    auto & idb = mGrepDriver->getBuilder();
    const unsigned baseBufferSize = this->getDefaultBufferBlocks();
    int MaxCountFlag = 0;

    //  Regular Expression Processing and Analysis Phase
    const auto nREs = 1;

    std::vector<StreamSetBuffer *> MatchResultsBufs(nREs);


    std::map<std::string, StreamSetBuffer *> propertyStream;

    std::vector<std::string> externalStreamNames;
    std::set<re::Name *> UnicodeProperties;


    re::Seq* seq = re::makeSeq();
    re::RE* targetRe = mRE;


    re::CC* linefeedCC = nullptr;
    if (utf8CC) {
        linefeedCC = re::makeCC(0x0A);
    } else {
        linefeedCC = re::makeByte(0x0A);
        targetRe = re::toUTF8(targetRe, true);
    }

    seq->push_back(targetRe);
    seq->push_back(std::move(linefeedCC));

    std::vector<re::CC*> UnicodeSets;
    if (utf8CC) {
        UnicodeSets = re::collectCCs(seq, &cc::Unicode, std::set<re::Name *>({re::makeZeroWidth("\\b{g}")}));
    } else {
        UnicodeSets = re::collectCCs(seq, &cc::Byte, std::set<re::Name *>({re::makeZeroWidth("\\b{g}")}));
    }

    StreamSetBuffer * const MatchResults = mGrepDriver->addBuffer<StaticBuffer>(idb, idb->getStreamSetTy(1, 1), baseBufferSize, 1);

    mpx = make_unique<cc::MultiplexedAlphabet>("mpx", UnicodeSets);
    mRE = transformCCs(mpx.get(), targetRe);


    std::vector<re::CC *> mpx_basis = mpx->getMultiplexedCCs();
    auto numOfCharacterClasses = mpx_basis.size();
    StreamSetBuffer * CharClasses = mGrepDriver->addBuffer<StaticBuffer>(idb, idb->getStreamSetTy(numOfCharacterClasses), baseBufferSize, 1);

    kernel::Kernel * ccK = nullptr;
    if (utf8CC) {
        ccK = mGrepDriver->addKernelInstance<kernel::CharClassesKernel>(idb, std::move(mpx_basis), false, cc::BitNumbering::BigEndian);
    } else {
        ccK = mGrepDriver->addKernelInstance<kernel::ByteClassesKernel>(idb, std::move(mpx_basis), false, cc::BitNumbering::BigEndian);
    }
    mGrepDriver->makeKernelCall(ccK, {compressedBitStream}, {CharClasses});

    StreamSetBuffer * uncompressedCharClasses = this->decompressBitStream(compressedByteStream, CharClasses);


    StreamSetBuffer * fakeMatchCopiedBits = mPxDriver.addBuffer<StaticBuffer>(idb, idb->getStreamSetTy(8),
                                                                              this->getDefaultBufferBlocks(), 1);
    StreamSetBuffer * u8NoFinalStream = mPxDriver.addBuffer<StaticBuffer>(idb, idb->getStreamSetTy(1),
                                                                          this->getDefaultBufferBlocks(), 1);

    Kernel* fakeStreamGeneratorK = mPxDriver.addKernelInstance<FakeStreamGeneratingKernel>(idb, numOfCharacterClasses, std::vector<unsigned>({8, 1}));
    mPxDriver.makeKernelCall(fakeStreamGeneratorK, {uncompressedCharClasses}, {fakeMatchCopiedBits, u8NoFinalStream});

    StreamSetBuffer * LineBreakStream = mPxDriver.addBuffer<StaticBuffer>(idb, idb->getStreamSetTy(1, 1), this->getDefaultBufferBlocks(), 1);
    kernel::Kernel * lineFeedGrepK = mGrepDriver->addKernelInstance<kernel::ICGrepKernel>(idb, transformCCs(mpx.get(), linefeedCC), externalStreamNames, std::vector<cc::Alphabet *>{mpx.get()}, cc::BitNumbering::BigEndian);
    mGrepDriver->makeKernelCall(lineFeedGrepK, {fakeMatchCopiedBits, uncompressedCharClasses}, {LineBreakStream});


    externalStreamNames.push_back("UTF8_nonfinal");

    kernel::Kernel * icgrepK = mGrepDriver->addKernelInstance<kernel::ICGrepKernel>(idb, mRE, externalStreamNames, std::vector<cc::Alphabet *>{mpx.get()}, cc::BitNumbering::BigEndian);
    mGrepDriver->makeKernelCall(icgrepK, {fakeMatchCopiedBits, u8NoFinalStream, uncompressedCharClasses}, {MatchResults});
    MatchResultsBufs[0] = MatchResults;

    StreamSetBuffer * MergedResults = MatchResultsBufs[0];

    StreamSetBuffer * Matches = MergedResults;
    if (mMoveMatchesToEOL) {
        StreamSetBuffer * OriginalMatches = Matches;
        kernel::Kernel * matchedLinesK = mGrepDriver->addKernelInstance<kernel::MatchedLinesKernel>(idb);
        Matches = mGrepDriver->addBuffer<StaticBuffer>(idb, idb->getStreamSetTy(1, 1), baseBufferSize, 1);
        mGrepDriver->makeKernelCall(matchedLinesK, {OriginalMatches, LineBreakStream}, {Matches});
    }

    if (MaxCountFlag > 0) {
        kernel::Kernel * untilK = mGrepDriver->addKernelInstance<kernel::UntilNkernel>(idb);
        untilK->setInitialArguments({idb->getSize(MaxCountFlag)});
        StreamSetBuffer * const AllMatches = Matches;
        Matches = mGrepDriver->addBuffer<StaticBuffer>(idb, idb->getStreamSetTy(1, 1), baseBufferSize, 1);
        mGrepDriver->makeKernelCall(untilK, {AllMatches}, {Matches});
    }

    return std::pair<StreamSetBuffer *, StreamSetBuffer *>(LineBreakStream, Matches);
};

std::pair<parabix::StreamSetBuffer *, parabix::StreamSetBuffer *> LZ4GrepBaseGenerator::grep(
        re::RE *RE, parabix::StreamSetBuffer *uncompressedBasisBits, bool ccMultiplexing) {

    this->initREs(RE);
    auto mGrepDriver = &mPxDriver;

    auto & idb = mGrepDriver->getBuilder();
    // TODO: until we automate stream buffer sizing, use this calculation to determine how large our matches buffer needs to be.
    const unsigned baseBufferSize = this->getDefaultBufferBlocks();
    int MaxCountFlag = 0;

    //  Regular Expression Processing and Analysis Phase
    const auto nREs = 1;

    std::vector<StreamSetBuffer *> MatchResultsBufs(nREs);

    StreamSetBuffer * LineBreakStream = this->linefeedStreamFromUncompressedBits(uncompressedBasisBits);


    std::map<std::string, StreamSetBuffer *> propertyStream;

    for(unsigned i = 0; i < nREs; ++i) {
        std::vector<std::string> externalStreamNames;
        std::vector<StreamSetBuffer *> icgrepInputSets = {uncompressedBasisBits};

        if (ccMultiplexing) {
            const auto UnicodeSets = re::collectCCs(mRE, &cc::Unicode, std::set<re::Name *>({re::makeZeroWidth("\\b{g}")}));
            StreamSetBuffer * const MatchResults = mGrepDriver->addBuffer<StaticBuffer>(idb, idb->getStreamSetTy(1, 1), baseBufferSize, 1);

            std::unique_ptr<cc::MultiplexedAlphabet> mpx = make_unique<cc::MultiplexedAlphabet>("mpx", UnicodeSets);
            mRE = transformCCs(mpx.get(), mRE);
            std::vector<re::CC *> mpx_basis = mpx->getMultiplexedCCs();
            auto numOfCharacterClasses = mpx_basis.size();
            StreamSetBuffer * CharClasses = mGrepDriver->addBuffer<StaticBuffer>(idb, idb->getStreamSetTy(numOfCharacterClasses), baseBufferSize, 1);
            kernel::Kernel * ccK = mGrepDriver->addKernelInstance<kernel::CharClassesKernel>(idb, std::move(mpx_basis), false, cc::BitNumbering::BigEndian);
            mGrepDriver->makeKernelCall(ccK, {uncompressedBasisBits}, {CharClasses});

            kernel::Kernel * icgrepK = mGrepDriver->addKernelInstance<kernel::ICGrepKernel>(idb, mRE, externalStreamNames, std::vector<cc::Alphabet *>{mpx.get()}, cc::BitNumbering::BigEndian);
            icgrepInputSets.push_back(CharClasses);
            mGrepDriver->makeKernelCall(icgrepK, icgrepInputSets, {MatchResults});
            MatchResultsBufs[i] = MatchResults;
        } else {
            std::set<re::Name *> UnicodeProperties;

            StreamSetBuffer * MatchResults = mGrepDriver->addBuffer<StaticBuffer>(idb, idb->getStreamSetTy(1, 1), baseBufferSize, 1);
            kernel::Kernel * icgrepK = mGrepDriver->addKernelInstance<kernel::ICGrepKernel>(idb, mRE, externalStreamNames, std::vector<cc::Alphabet *>(), cc::BitNumbering::BigEndian);
            mGrepDriver->makeKernelCall(icgrepK, icgrepInputSets, {MatchResults});
            MatchResultsBufs[i] = MatchResults;
        }
    }

    StreamSetBuffer * MergedResults = MatchResultsBufs[0];

    StreamSetBuffer * Matches = MergedResults;
    if (mMoveMatchesToEOL) {
        StreamSetBuffer * OriginalMatches = Matches;
        kernel::Kernel * matchedLinesK = mGrepDriver->addKernelInstance<kernel::MatchedLinesKernel>(idb);
        Matches = mGrepDriver->addBuffer<StaticBuffer>(idb, idb->getStreamSetTy(1, 1), baseBufferSize, 1);
        mGrepDriver->makeKernelCall(matchedLinesK, {OriginalMatches, LineBreakStream}, {Matches});
    }

    if (MaxCountFlag > 0) {
        kernel::Kernel * untilK = mGrepDriver->addKernelInstance<kernel::UntilNkernel>(idb);
        untilK->setInitialArguments({idb->getSize(MaxCountFlag)});
        StreamSetBuffer * const AllMatches = Matches;
        Matches = mGrepDriver->addBuffer<StaticBuffer>(idb, idb->getStreamSetTy(1, 1), baseBufferSize, 1);
        mGrepDriver->makeKernelCall(untilK, {AllMatches}, {Matches});
    }

    return std::pair<StreamSetBuffer *, StreamSetBuffer *>(LineBreakStream, Matches);

}

void LZ4GrepBaseGenerator::invokeScanMatchGrep(char* fileBuffer, size_t blockStart, size_t blockEnd, bool hasBlockChecksum) {
    auto main = this->getScanMatchGrepMainFunction();
    std::ostringstream s;
    EmitMatch accum("", false, false, s);

    main(fileBuffer, blockStart, blockEnd, hasBlockChecksum, reinterpret_cast<intptr_t>(&accum));
    llvm::outs() << s.str();
}



void LZ4GrepBaseGenerator::generateMultiplexingCountOnlyGrepPipeline(re::RE *regex, bool utf8CC) {
    auto & iBuilder = mPxDriver.getBuilder();
    this->generateCountOnlyMainFunc(iBuilder);

    StreamSetBuffer *compressedByteStream = nullptr, *compressedBasisBits = nullptr;
    std::tie(compressedByteStream, compressedBasisBits) = this->loadByteStreamAndBitStream();

    StreamSetBuffer * LineBreakStream;
    StreamSetBuffer * Matches;
    std::tie(LineBreakStream, Matches) = multiplexingGrep(regex, compressedByteStream, compressedBasisBits, utf8CC);

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


void LZ4GrepBaseGenerator::generateFullyDecompressionCountOnlyGrepPipeline(re::RE *regex) {
    auto & iBuilder = mPxDriver.getBuilder();
    this->generateCountOnlyMainFunc(iBuilder);

    StreamSetBuffer * const uncompressedBitStream = this->generateUncompressedBitStreams();

    StreamSetBuffer * LineBreakStream;
    StreamSetBuffer * Matches;

    std::tie(LineBreakStream, Matches) = grep(regex, uncompressedBitStream);

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


ScanMatchGrepMainFunctionType LZ4GrepBaseGenerator::getScanMatchGrepMainFunction() {
    return reinterpret_cast<ScanMatchGrepMainFunctionType>(mPxDriver.getMain());
}
CountOnlyGrepMainFunctionType LZ4GrepBaseGenerator::getCountOnlyGrepMainFunction() {
    return reinterpret_cast<CountOnlyGrepMainFunctionType>(mPxDriver.getMain());
}

void LZ4GrepBaseGenerator::generateCountOnlyMainFunc(const std::unique_ptr<kernel::KernelBuilder> & iBuilder) {
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

void LZ4GrepBaseGenerator::generateScanMatchMainFunc(const std::unique_ptr<kernel::KernelBuilder> & iBuilder) {
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
