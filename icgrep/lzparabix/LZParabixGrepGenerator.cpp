//
// Created by wxy325 on 2018/6/19.
//

#include "LZParabixGrepGenerator.h"


#include <boost/iostreams/device/mapped_file.hpp>

#include <llvm/Support/PrettyStackTrace.h>

#include <cc/cc_compiler.h>

#include <kernels/cc_kernel.h>
#include <kernels/s2p_kernel.h>
#include <kernels/p2s_kernel.h>
#include <kernels/source_kernel.h>
#include <kernels/stdout_kernel.h>
#include <kernels/kernel_builder.h>
#include <kernels/swizzle.h>
#include <re/re_toolchain.h>

#include <re/collect_ccs.h>
#include <re/replaceCC.h>
#include <re/re_seq.h>
#include <re/re_cc.h>

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
#include <kernels/fake_stream_generating_kernel.h>
#include <re/re_alt.h>

namespace re { class CC; }

using namespace llvm;
using namespace parabix;
using namespace kernel;
using namespace grep;


LZParabixGrepGenerator::LZParabixGrepGenerator(bool enableMultiplexing): LZParabixGenerator(), mEnableMultiplexing(enableMultiplexing) {
    mGrepRecordBreak = grep::GrepRecordBreakKind::LF;
    mMoveMatchesToEOL = true;
}

void LZParabixGrepGenerator::initREs(std::vector<re::RE *> &REs) {
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

void LZParabixGrepGenerator::generateCountOnlyMainFunc(const std::unique_ptr<kernel::KernelBuilder> &iBuilder) {
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

void LZParabixGrepGenerator::generateCountOnlyAioPipeline(re::RE *regex, bool swizzledDecompression ) {
    auto & iBuilder = mPxDriver.getBuilder();
    this->generateCountOnlyMainFunc(iBuilder);

    this->generateLoadByteStreamAndBitStream(iBuilder);


    StreamSetBuffer * LineBreakStream;
    StreamSetBuffer * Matches;
    std::vector<re::RE*> res = {regex};
    if (mEnableMultiplexing) {
        std::tie(LineBreakStream, Matches) = multiplexingGrepPipeline(res);
    } else {
        std::tie(LineBreakStream, Matches) = grepPipeline(res, swizzledDecompression);
    }

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


std::pair<parabix::StreamSetBuffer *, parabix::StreamSetBuffer *> LZParabixGrepGenerator::multiplexingGrepPipeline(std::vector<re::RE *> &REs) {

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

    re::CC* linefeedCC = re::makeCC(0x0A);

    re::Seq* seq = re::makeSeq();
    seq->push_back(mREs[0]);
    seq->push_back(std::move(linefeedCC));

    const auto UnicodeSets = re::collectCCs(seq, &cc::Unicode, std::set<re::Name *>({re::makeZeroWidth("\\b{g}")}));
    StreamSetBuffer * const MatchResults = mGrepDriver->addBuffer<StaticBuffer>(idb, idb->getStreamSetTy(1, 1), baseBufferSize, 1);

    this->generateBlockData(idb);
    StreamSetBuffer * const LiteralBitStream = this->extractLiteralBitStream(idb);

    bool allCcByteLength = re::isAllCcByteLength(mREs[0]);

    mpx = make_unique<cc::MultiplexedAlphabet>("mpx", UnicodeSets);
    mREs[0] = transformCCs(mpx.get(), mREs[0]);

    std::vector<re::CC *> mpx_basis = mpx->getMultiplexedCCs();
    auto numOfCharacterClasses = mpx_basis.size();
//    llvm::outs() << "numOfCharacterClasses:" << numOfCharacterClasses << "\n";
    StreamSetBuffer * CharClasses = mGrepDriver->addBuffer<StaticBuffer>(idb, idb->getStreamSetTy(numOfCharacterClasses), baseBufferSize, 1);

    kernel::Kernel * ccK = mGrepDriver->addKernelInstance<kernel::CharClassesKernel>(idb, std::move(mpx_basis), false, cc::BitNumbering::BigEndian);
    mGrepDriver->makeKernelCall(ccK, {LiteralBitStream}, {CharClasses});

    StreamSetBuffer * newLineBreak = nullptr;


    StreamSetBuffer * decompressedCharClasses = nullptr;
    StreamSetBuffer * u8NoFinalStream = nullptr;
    StreamSetBuffer * fakeMatchCopiedBits = nullptr;

    if (allCcByteLength) {
        auto ret = this->generateBitStreamDecompression(idb, {CharClasses});
        decompressedCharClasses = ret[0];
        fakeMatchCopiedBits = mPxDriver.addBuffer<StaticBuffer>(idb, idb->getStreamSetTy(8), this->getInputBufferBlocks(idb), 1);
        u8NoFinalStream = mPxDriver.addBuffer<StaticBuffer>(idb, idb->getStreamSetTy(1), this->getInputBufferBlocks(idb), 1);

        Kernel* fakeStreamGeneratorK = mPxDriver.addKernelInstance<FakeStreamGeneratingKernel>(idb, numOfCharacterClasses, std::vector<unsigned>({8, 1}));
        mPxDriver.makeKernelCall(fakeStreamGeneratorK, {decompressedCharClasses}, {fakeMatchCopiedBits, u8NoFinalStream});
    } else {
        re::RE* nonFinalName = re::makeAlt({re::makeByte(0xC2, 0xF4),
                                            re::makeSeq({re::makeByte(0xE0, 0xF4), re::makeByte(0x80, 0xBF)}),
                                            re::makeSeq({re::makeByte(0xF0, 0xF4), re::makeByte(0x80, 0xBF), re::makeByte(0x80, 0xBF)})});
        StreamSetBuffer * compressedU8NoFinalStream = mPxDriver.addBuffer<StaticBuffer>(idb, idb->getStreamSetTy(1, 1), this->getInputBufferBlocks(idb));
        kernel::Kernel * u8NoFinalK = mGrepDriver->addKernelInstance<kernel::ICGrepKernel>(idb, nonFinalName, externalStreamNames, std::vector<cc::Alphabet *>(), cc::BitNumbering::BigEndian);
        mGrepDriver->makeKernelCall(u8NoFinalK, {LiteralBitStream}, {compressedU8NoFinalStream});

        auto ret = this->generateBitStreamDecompression(idb, {CharClasses, compressedU8NoFinalStream});

        decompressedCharClasses = ret[0];
        u8NoFinalStream = ret[1];

        fakeMatchCopiedBits = mPxDriver.addBuffer<StaticBuffer>(idb, idb->getStreamSetTy(8), this->getInputBufferBlocks(idb), 1);
        Kernel* fakeStreamGeneratorK = mPxDriver.addKernelInstance<FakeStreamGeneratingKernel>(idb, numOfCharacterClasses, 8);
        mPxDriver.makeKernelCall(fakeStreamGeneratorK, {decompressedCharClasses}, {fakeMatchCopiedBits});

    }


    newLineBreak = mPxDriver.addBuffer<StaticBuffer>(idb, idb->getStreamSetTy(1, 1), this->getInputBufferBlocks(idb));
    kernel::Kernel * lineFeedGrepK = mGrepDriver->addKernelInstance<kernel::ICGrepKernel>(idb, transformCCs(mpx.get(), linefeedCC), externalStreamNames, std::vector<cc::Alphabet *>{mpx.get()}, cc::BitNumbering::BigEndian);
    mGrepDriver->makeKernelCall(lineFeedGrepK, {fakeMatchCopiedBits, decompressedCharClasses}, {newLineBreak});

    externalStreamNames.push_back("UTF8_nonfinal");

    kernel::Kernel * icgrepK = mGrepDriver->addKernelInstance<kernel::ICGrepKernel>(idb, mREs[0], externalStreamNames, std::vector<cc::Alphabet *>{mpx.get()}, cc::BitNumbering::BigEndian);
    mGrepDriver->makeKernelCall(icgrepK, {fakeMatchCopiedBits, u8NoFinalStream, decompressedCharClasses}, {MatchResults});
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
        mGrepDriver->makeKernelCall(matchedLinesK, {OriginalMatches, newLineBreak}, {Matches});
    }

    if (MaxCountFlag > 0) {
        kernel::Kernel * untilK = mGrepDriver->addKernelInstance<kernel::UntilNkernel>(idb);
        untilK->setInitialArguments({idb->getSize(MaxCountFlag)});
        StreamSetBuffer * const AllMatches = Matches;
        Matches = mGrepDriver->addBuffer<StaticBuffer>(idb, idb->getStreamSetTy(1, 1), baseBufferSize);
        mGrepDriver->makeKernelCall(untilK, {AllMatches}, {Matches});
    }

    return std::pair<StreamSetBuffer *, StreamSetBuffer *>(newLineBreak, Matches);
};


std::pair<parabix::StreamSetBuffer *, parabix::StreamSetBuffer *>
LZParabixGrepGenerator::grepPipeline(std::vector<re::RE *> &REs, bool swizzledDecompression) {

    this->initREs(REs);
    auto mGrepDriver = &mPxDriver;

    auto & idb = mGrepDriver->getBuilder();
    // TODO: until we automate stream buffer sizing, use this calculation to determine how large our matches buffer needs to be.
    const unsigned baseBufferSize = this->getInputBufferBlocks(idb);
    int MaxCountFlag = 0;

    //  Regular Expression Processing and Analysis Phase
    const auto nREs = mREs.size();

    std::vector<StreamSetBuffer *> MatchResultsBufs(nREs);


    this->generateBlockData(idb);
    StreamSetBuffer * const LiteralBitStream = this->extractLiteralBitStream(idb);
//    auto compressedLineBreakStream = this->linefeedStreamFromDecompressedBits(LiteralBitStream);


    StreamSetBuffer * decompressedBasisBits = nullptr;
    if (swizzledDecompression) {
        decompressedBasisBits = this->generateSwizzledBitStreamDecompression(idb, LiteralBitStream);
    } else {
        auto ret = this->generateBitStreamDecompression(idb, {LiteralBitStream});
        decompressedBasisBits = ret[0];
    }

    StreamSetBuffer * LineBreakStream = this->linefeedStreamFromDecompressedBits(decompressedBasisBits);

    std::map<std::string, StreamSetBuffer *> propertyStream;

    for(unsigned i = 0; i < nREs; ++i) {
        std::vector<std::string> externalStreamNames;
        std::vector<StreamSetBuffer *> icgrepInputSets = {decompressedBasisBits};

        std::set<re::Name *> UnicodeProperties;

        StreamSetBuffer * MatchResults = mGrepDriver->addBuffer<StaticBuffer>(idb, idb->getStreamSetTy(1, 1), baseBufferSize);
        kernel::Kernel * icgrepK = mGrepDriver->addKernelInstance<kernel::ICGrepKernel>(idb, mREs[i], externalStreamNames, std::vector<cc::Alphabet *>(), cc::BitNumbering::BigEndian);
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

parabix::StreamSetBuffer *
LZParabixGrepGenerator::linefeedStreamFromDecompressedBits(parabix::StreamSetBuffer *decompressedBasisBits) {
    auto & idb = mPxDriver.getBuilder();
    const unsigned baseBufferSize = this->getInputBufferBlocks(idb);
    StreamSetBuffer * LineFeedStream = mPxDriver.addBuffer<StaticBuffer>(idb, idb->getStreamSetTy(1, 1), baseBufferSize);
    kernel::Kernel * linefeedK = mPxDriver.addKernelInstance<kernel::LineFeedKernelBuilder>(idb, Binding{idb->getStreamSetTy(8), "basis", FixedRate(), Principal()}, cc::BitNumbering::BigEndian);
    mPxDriver.makeKernelCall(linefeedK, {decompressedBasisBits}, {LineFeedStream});
    return LineFeedStream;
}

CountOnlyGrepMainFunctionType LZParabixGrepGenerator::getCountOnlyGrepMainFunction() {
    return reinterpret_cast<CountOnlyGrepMainFunctionType>(mPxDriver.getMain());
}
