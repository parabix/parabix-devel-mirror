//
// Created by wxy325 on 2018/3/15.
//

#include "LZ4GrepGenerator.h"

#include <boost/filesystem.hpp>
#include <boost/iostreams/device/mapped_file.hpp>

#include <llvm/Support/CommandLine.h>
#include <llvm/Support/PrettyStackTrace.h>

#include <cc/cc_compiler.h>

#include <lz4FrameDecoder.h>
#include <kernels/streamset.h>
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
#include <kernels/lz4/lz4_multiple_pdep_kernel.h>
#include <kernels/lz4/lz4_match_copy_kernel.h>
#include <kernels/lz4/lz4_swizzled_match_copy_kernel.h>
#include <re/re_toolchain.h>

#include <re/collect_ccs.h>
#include <re/replaceCC.h>

#include <set>
#include "grep/grep_engine.h"
#include "grep_interface.h"
#include <llvm/IR/Module.h>
#include <boost/filesystem.hpp>
#include <UCD/resolve_properties.h>
#include <kernels/charclasses.h>
#include <kernels/cc_kernel.h>
#include <kernels/grep_kernel.h>
#include <kernels/UCD_property_kernel.h>
#include <kernels/grapheme_kernel.h>
#include <kernels/linebreak_kernel.h>
#include <kernels/streams_merge.h>
#include <kernels/source_kernel.h>
#include <kernels/s2p_kernel.h>
#include <kernels/scanmatchgen.h>
#include <kernels/streamset.h>
#include <kernels/until_n.h>
#include <kernels/kernel_builder.h>
#include <pablo/pablo_kernel.h>
#include <re/re_cc.h>
#include <re/re_name.h>
#include <re/casing.h>
#include <re/exclude_CC.h>
#include <re/to_utf8.h>
#include <re/re_toolchain.h>
#include <toolchain/toolchain.h>
#include <re/re_analysis.h>
#include <re/re_name_resolve.h>
#include <re/re_name_gather.h>
//#include <re/re_collect_unicodesets.h>
#include <re/re_multiplex.h>
#include <re/grapheme_clusters.h>
#include <re/printer_re.h>
#include <toolchain/toolchain.h>
#include <toolchain/cpudriver.h>
#include <iostream>
#include <cc/multiplex_CCs.h>
#include <llvm/Support/raw_ostream.h>
#include <util/aligned_allocator.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <errno.h>
#include <llvm/ADT/STLExtras.h> // for make_unique
#include <llvm/Support/CommandLine.h>
#include <llvm/Support/Debug.h>
#include <sched.h>
#include <cstdio>
#include <cc/multiplex_CCs.h>


namespace re { class CC; }

using namespace llvm;
using namespace parabix;
using namespace kernel;
using namespace grep;


const unsigned ByteCClimit = 6;


LZ4GrepGenerator::LZ4GrepGenerator(): LZ4Generator() {
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
        anchorName->setDefinition(UCD::UnicodeBreakRE());
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



std::pair<parabix::StreamSetBuffer *, parabix::StreamSetBuffer *> LZ4GrepGenerator::grepPipeline(
        std::vector<re::RE *> &REs, parabix::StreamSetBuffer *BasisBits) {

    this->initREs(REs);
    auto mGrepDriver = &pxDriver;


    auto & idb = mGrepDriver->getBuilder();
    // TODO: until we automate stream buffer sizing, use this calculation to determine how large our matches buffer needs to be.
    const unsigned baseBufferSize = this->getInputBufferBlocks();
    bool MultithreadedSimpleRE = false;
    bool PropertyKernels = false;
    bool CC_Multiplexing = false;
    bool InvertMatchFlag = false;
    int MaxCountFlag = 0;




    //  Regular Expression Processing and Analysis Phase
    const auto nREs = mREs.size();
    bool hasGCB[nREs];
    bool anyGCB = false;

    for(unsigned i = 0; i < nREs; ++i) {
        hasGCB[i] = hasGraphemeClusterBoundary(mREs[i]);
        anyGCB |= hasGCB[i];
    }
    StreamSetBuffer * LineBreakStream = mGrepDriver->addBuffer<CircularBuffer>(idb, idb->getStreamSetTy(1, 1), baseBufferSize);
    std::vector<StreamSetBuffer *> MatchResultsBufs(nREs);

    re::RE * prefixRE;
    re::RE * suffixRE;
    // For simple regular expressions with a small number of characters, we
    // can bypass transposition and use the Direct CC compiler.
//    bool isSimple = (nREs == 1) && (mGrepRecordBreak != GrepRecordBreakKind::Unicode) && (!anyGCB);
    bool isSimple = false;
    if (isSimple) {
        mREs[0] = toUTF8(mREs[0]);
    }
    if (isSimple && byteTestsWithinLimit(mREs[0], ByteCClimit)) {
        std::vector<std::string> externalStreamNames;
        std::vector<StreamSetBuffer *> icgrepInputSets = {ByteStream};
        if (MultithreadedSimpleRE && hasTriCCwithinLimit(mREs[0], ByteCClimit, prefixRE, suffixRE)) {
            auto CCs = re::collectCCs(prefixRE, &cc::Byte);
            for (auto cc : CCs) {
                auto ccName = makeName(cc);
                mREs[0] = re::replaceCC(mREs[0], cc, ccName);
                std::string ccNameStr = ccName->getFullName();
                StreamSetBuffer * ccStream = mGrepDriver->addBuffer<CircularBuffer>(idb, idb->getStreamSetTy(1, 1), baseBufferSize);
                kernel::Kernel * ccK = mGrepDriver->addKernelInstance<kernel::DirectCharacterClassKernelBuilder>(idb, ccNameStr, std::vector<re::CC *>{cc});
                mGrepDriver->makeKernelCall(ccK, {ByteStream}, {ccStream});
                externalStreamNames.push_back(ccNameStr);
                icgrepInputSets.push_back(ccStream);
            }
        }
        StreamSetBuffer * MatchResults = mGrepDriver->addBuffer<CircularBuffer>(idb, idb->getStreamSetTy(1, 1), baseBufferSize);
        kernel::Kernel * icgrepK = mGrepDriver->addKernelInstance<kernel::ByteGrepKernel>(idb, mREs[0], externalStreamNames);
        mGrepDriver->makeKernelCall(icgrepK, icgrepInputSets, {MatchResults});
        MatchResultsBufs[0] = MatchResults;
        kernel::Kernel * breakK = mGrepDriver->addKernelInstance<kernel::DirectCharacterClassKernelBuilder>(idb, "breakCC", std::vector<re::CC *>{mBreakCC});
        mGrepDriver->makeKernelCall(breakK, {ByteStream}, {LineBreakStream});
    } else if (isSimple && hasTriCCwithinLimit(mREs[0], ByteCClimit, prefixRE, suffixRE)) {
        std::vector<std::string> externalStreamNames;
        std::vector<StreamSetBuffer *> icgrepInputSets = {ByteStream};
        if (MultithreadedSimpleRE) {
            auto CCs = re::collectCCs(prefixRE, &cc::Byte);
            for (auto cc : CCs) {
                auto ccName = makeName(cc);
                mREs[0] = re::replaceCC(mREs[0], cc, ccName);
                std::string ccNameStr = ccName->getFullName();
                StreamSetBuffer * ccStream = mGrepDriver->addBuffer<CircularBuffer>(idb, idb->getStreamSetTy(1, 1), baseBufferSize);
                kernel::Kernel * ccK = mGrepDriver->addKernelInstance<kernel::DirectCharacterClassKernelBuilder>(idb, ccNameStr, std::vector<re::CC *>{cc});
                mGrepDriver->makeKernelCall(ccK, {ByteStream}, {ccStream});
                externalStreamNames.push_back(ccNameStr);
                icgrepInputSets.push_back(ccStream);
            }
        }
        StreamSetBuffer * MatchResults = mGrepDriver->addBuffer<CircularBuffer>(idb, idb->getStreamSetTy(1, 1), baseBufferSize);
        kernel::Kernel * icgrepK = mGrepDriver->addKernelInstance<kernel::ByteBitGrepKernel>(idb, prefixRE, suffixRE, externalStreamNames);
        mGrepDriver->makeKernelCall(icgrepK, icgrepInputSets, {MatchResults});
        MatchResultsBufs[0] = MatchResults;
        kernel::Kernel * breakK = mGrepDriver->addKernelInstance<kernel::DirectCharacterClassKernelBuilder>(idb, "breakCC", std::vector<re::CC *>{mBreakCC});
        mGrepDriver->makeKernelCall(breakK, {ByteStream}, {LineBreakStream});
    } else {
        StreamSetBuffer * RequiredStreams = mGrepDriver->addBuffer<CircularBuffer>(idb, idb->getStreamSetTy(1, 1), baseBufferSize);
        StreamSetBuffer * UnicodeLB = mGrepDriver->addBuffer<CircularBuffer>(idb, idb->getStreamSetTy(1, 1), baseBufferSize);

        StreamSetBuffer * LineFeedStream = mGrepDriver->addBuffer<CircularBuffer>(idb, idb->getStreamSetTy(1, 1), baseBufferSize);
        kernel::Kernel * linefeedK = mGrepDriver->addKernelInstance<kernel::LineFeedKernelBuilder>(idb, Binding{idb->getStreamSetTy(8), "basis", FixedRate(), Principal()});
        mGrepDriver->makeKernelCall(linefeedK, {BasisBits}, {LineFeedStream});

        kernel::Kernel * requiredStreamsK = mGrepDriver->addKernelInstance<kernel::RequiredStreams_UTF8>(idb);
        mGrepDriver->makeKernelCall(requiredStreamsK, {BasisBits, LineFeedStream}, {RequiredStreams, UnicodeLB});

        if (mGrepRecordBreak == GrepRecordBreakKind::LF) {
            LineBreakStream = LineFeedStream;
        } else if (mGrepRecordBreak == GrepRecordBreakKind::Null) {
            kernel::Kernel * breakK = mGrepDriver->addKernelInstance<kernel::ParabixCharacterClassKernelBuilder>(idb, "Null", std::vector<re::CC *>{mBreakCC}, 8);
            mGrepDriver->makeKernelCall(breakK, {BasisBits}, {LineBreakStream});
        } else {
            LineBreakStream = UnicodeLB;
        }

        std::map<std::string, StreamSetBuffer *> propertyStream;
        if (PropertyKernels) {
            for (auto p : mUnicodeProperties) {
                auto name = p->getFullName();
                StreamSetBuffer * s = mGrepDriver->addBuffer<CircularBuffer>(idb, idb->getStreamSetTy(1, 1), baseBufferSize);
                propertyStream.emplace(std::make_pair(name, s));
                kernel::Kernel * propertyK = mGrepDriver->addKernelInstance<kernel::UnicodePropertyKernelBuilder>(idb, p);
                mGrepDriver->makeKernelCall(propertyK, {BasisBits}, {s});
            }
        }
        StreamSetBuffer * GCB_stream = nullptr;
        if (anyGCB) {
            GCB_stream = mGrepDriver->addBuffer<CircularBuffer>(idb, idb->getStreamSetTy(1, 1), baseBufferSize);
            kernel::Kernel * gcbK = mGrepDriver->addKernelInstance<kernel::GraphemeClusterBreakKernel>(idb);
            mGrepDriver->makeKernelCall(gcbK, {BasisBits, RequiredStreams}, {GCB_stream});
        }

        for(unsigned i = 0; i < nREs; ++i) {
            std::vector<std::string> externalStreamNames;
            std::vector<StreamSetBuffer *> icgrepInputSets = {BasisBits};
            if (mGrepRecordBreak == GrepRecordBreakKind::Unicode) {
                externalStreamNames.push_back("UTF8_LB");
                icgrepInputSets.push_back(LineBreakStream);
                externalStreamNames.push_back("UTF8_nonfinal");
                icgrepInputSets.push_back(RequiredStreams);
            }
            std::set<re::Name *> UnicodeProperties;
            if (PropertyKernels) {
                re::gatherUnicodeProperties(mREs[i], UnicodeProperties);
                for (auto p : UnicodeProperties) {
                    auto name = p->getFullName();
                    auto f = propertyStream.find(name);
                    if (f == propertyStream.end()) report_fatal_error(name + " not found\n");
                    externalStreamNames.push_back(name);
                    icgrepInputSets.push_back(f->second);
                }
            }
            if (hasGCB[i]) {
                externalStreamNames.push_back("\\b{g}");
                icgrepInputSets.push_back(GCB_stream);
            }
            if (CC_Multiplexing) {
                const auto UnicodeSets = re::collectCCs(mREs[i], &cc::Unicode, std::set<re::Name *>({re::makeZeroWidth("\\b{g}")}));
                StreamSetBuffer * const MatchResults = mGrepDriver->addBuffer<CircularBuffer>(idb, idb->getStreamSetTy(1, 1), baseBufferSize);
                if (UnicodeSets.size() <= 1) {
                    kernel::Kernel * icgrepK = mGrepDriver->addKernelInstance<kernel::ICGrepKernel>(idb, mREs[i], externalStreamNames);
                    mGrepDriver->makeKernelCall(icgrepK, icgrepInputSets, {MatchResults});
                    MatchResultsBufs[i] = MatchResults;
                } else {
                    mpx = make_unique<cc::MultiplexedAlphabet>("mpx", UnicodeSets);
                    mREs[i] = transformCCs(mpx.get(), mREs[i]);
                    std::vector<re::CC *> mpx_basis = mpx->getMultiplexedCCs();
                    auto numOfCharacterClasses = mpx_basis.size();
                    StreamSetBuffer * CharClasses = mGrepDriver->addBuffer<CircularBuffer>(idb, idb->getStreamSetTy(numOfCharacterClasses), baseBufferSize);
                    kernel::Kernel * ccK = mGrepDriver->addKernelInstance<kernel::CharClassesKernel>(idb, std::move(mpx_basis));
                    mGrepDriver->makeKernelCall(ccK, {BasisBits}, {CharClasses});
                    //                kernel::Kernel * ccK = mGrepDriver->addKernelInstance<kernel::CharClassesKernel>(idb, std::move(mpx_basis), true);
                    //                mGrepDriver->makeKernelCall(ccK, {ByteStream}, {CharClasses});
                    kernel::Kernel * icgrepK = mGrepDriver->addKernelInstance<kernel::ICGrepKernel>(idb, mREs[i], externalStreamNames, std::vector<cc::Alphabet *>{mpx.get()});
                    icgrepInputSets.push_back(CharClasses);
                    mGrepDriver->makeKernelCall(icgrepK, icgrepInputSets, {MatchResults});
                    MatchResultsBufs[i] = MatchResults;
                }
            } else {
                StreamSetBuffer * MatchResults = mGrepDriver->addBuffer<CircularBuffer>(idb, idb->getStreamSetTy(1, 1), baseBufferSize);
                kernel::Kernel * icgrepK = mGrepDriver->addKernelInstance<kernel::ICGrepKernel>(idb, mREs[i], externalStreamNames);
                mGrepDriver->makeKernelCall(icgrepK, icgrepInputSets, {MatchResults});
                MatchResultsBufs[i] = MatchResults;
            }
        }
    }

    StreamSetBuffer * MergedResults = MatchResultsBufs[0];
    if (mREs.size() > 1) {
        MergedResults = mGrepDriver->addBuffer<CircularBuffer>(idb, idb->getStreamSetTy(1, 1), baseBufferSize);
        kernel::Kernel * streamsMergeK = mGrepDriver->addKernelInstance<kernel::StreamsMerge>(idb, 1, mREs.size());
        mGrepDriver->makeKernelCall(streamsMergeK, MatchResultsBufs, {MergedResults});
    }
    StreamSetBuffer * Matches = MergedResults;
    if (mMoveMatchesToEOL) {
        StreamSetBuffer * OriginalMatches = Matches;
        kernel::Kernel * matchedLinesK = mGrepDriver->addKernelInstance<kernel::MatchedLinesKernel>(idb);
        Matches = mGrepDriver->addBuffer<CircularBuffer>(idb, idb->getStreamSetTy(1, 1), baseBufferSize);
        mGrepDriver->makeKernelCall(matchedLinesK, {OriginalMatches, LineBreakStream}, {Matches});
    }
    if (InvertMatchFlag) {
        kernel::Kernel * invertK = mGrepDriver->addKernelInstance<kernel::InvertMatchesKernel>(idb);
        StreamSetBuffer * OriginalMatches = Matches;
        Matches = mGrepDriver->addBuffer<CircularBuffer>(idb, idb->getStreamSetTy(1, 1), baseBufferSize);
        mGrepDriver->makeKernelCall(invertK, {OriginalMatches, LineBreakStream}, {Matches});
    }
    if (MaxCountFlag > 0) {
        kernel::Kernel * untilK = mGrepDriver->addKernelInstance<kernel::UntilNkernel>(idb);
        untilK->setInitialArguments({idb->getSize(MaxCountFlag)});
        StreamSetBuffer * const AllMatches = Matches;
        Matches = mGrepDriver->addBuffer<CircularBuffer>(idb, idb->getStreamSetTy(1, 1), baseBufferSize);
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
    auto & iBuilder = pxDriver.getBuilder();
    this->generateScanMatchMainFunc(iBuilder);

    StreamSetBuffer * const DecompressedByteStream = pxDriver.addBuffer<CircularBuffer>(iBuilder, iBuilder->getStreamSetTy(1, 8), this->getDecompressedBufferBlocks());

    // GeneratePipeline
    this->generateLoadByteStreamAndBitStream(iBuilder);
    this->generateExtractAndDepositMarkers(iBuilder);

    auto swizzle = this->generateSwizzleExtractData(iBuilder);

    //TODO buffer blocks should be decompressedBufferBlocks
    StreamSetBuffer * depositedSwizzle0 = pxDriver.addBuffer<CircularCopybackBuffer>(iBuilder, iBuilder->getStreamSetTy(4), this->getInputBufferBlocks(), 1);
    StreamSetBuffer * depositedSwizzle1 = pxDriver.addBuffer<CircularCopybackBuffer>(iBuilder, iBuilder->getStreamSetTy(4), this->getInputBufferBlocks(), 1);

    Kernel * multiplePdepK = pxDriver.addKernelInstance<LZ4MultiplePDEPkernel>(iBuilder, 4, 2, 4);
    pxDriver.makeKernelCall(multiplePdepK, {DepositMarker, swizzle.first, swizzle.second}, {depositedSwizzle0, depositedSwizzle1});

    StreamSetBuffer * matchCopiedSwizzle0 = pxDriver.addBuffer<CircularCopybackBuffer>(iBuilder, iBuilder->getStreamSetTy(4), this->getInputBufferBlocks(), 1);
    StreamSetBuffer * matchCopiedSwizzle1 = pxDriver.addBuffer<CircularCopybackBuffer>(iBuilder, iBuilder->getStreamSetTy(4), this->getInputBufferBlocks(), 1);

    Kernel * swizzledMatchCopyK = pxDriver.addKernelInstance<LZ4SwizzledMatchCopyKernel>(iBuilder, 4, 2, 4);
    pxDriver.makeKernelCall(swizzledMatchCopyK, {MatchOffsetMarker, M0Marker, M0CountMarker, ByteStream, depositedSwizzle0, depositedSwizzle1}, {matchCopiedSwizzle0, matchCopiedSwizzle1});


    // Produce unswizzled bit streams
    StreamSetBuffer * extractedbits = pxDriver.addBuffer<CircularBuffer>(iBuilder, iBuilder->getStreamSetTy(8), this->getInputBufferBlocks());
    Kernel * unSwizzleK = pxDriver.addKernelInstance<SwizzleGenerator>(iBuilder, 8, 1, 2);
    pxDriver.makeKernelCall(unSwizzleK, {matchCopiedSwizzle0, matchCopiedSwizzle1}, {extractedbits});



    Kernel * p2sK = pxDriver.addKernelInstance<P2SKernel>(iBuilder);
    pxDriver.makeKernelCall(p2sK, {extractedbits}, {DecompressedByteStream});

    StreamSetBuffer * LineBreakStream;
    StreamSetBuffer * Matches;
    std::vector<re::RE*> res = {regex};
    std::tie(LineBreakStream, Matches) = grepPipeline(res, extractedbits);

    kernel::Kernel * scanMatchK = pxDriver.addKernelInstance<kernel::ScanMatchKernel>(iBuilder);
    scanMatchK->setInitialArguments({match_accumulator});
    pxDriver.makeKernelCall(scanMatchK, {Matches, LineBreakStream, DecompressedByteStream}, {});
    pxDriver.LinkFunction(*scanMatchK, "accumulate_match_wrapper", &accumulate_match_wrapper);
    pxDriver.LinkFunction(*scanMatchK, "finalize_match_wrapper", &finalize_match_wrapper);

    pxDriver.generatePipelineIR();
    pxDriver.deallocateBuffers();

    iBuilder->CreateRetVoid();

    pxDriver.finalizeObject();
}

void LZ4GrepGenerator::generateCountOnlyGrepPipeline(re::RE* regex) {
    auto & iBuilder = pxDriver.getBuilder();
    this->generateMainFunc(iBuilder);


    // GeneratePipeline
    this->generateLoadByteStreamAndBitStream(iBuilder);
    this->generateExtractAndDepositMarkers(iBuilder);

    auto swizzle = this->generateSwizzleExtractData(iBuilder);

    StreamSetBuffer * depositedSwizzle0 = pxDriver.addBuffer<CircularCopybackBuffer>(iBuilder, iBuilder->getStreamSetTy(4), this->getInputBufferBlocks(), 1);
    StreamSetBuffer * depositedSwizzle1 = pxDriver.addBuffer<CircularCopybackBuffer>(iBuilder, iBuilder->getStreamSetTy(4), this->getInputBufferBlocks(), 1);

    Kernel * multiplePdepK = pxDriver.addKernelInstance<LZ4MultiplePDEPkernel>(iBuilder, 4, 2, 4);
    pxDriver.makeKernelCall(multiplePdepK, {DepositMarker, swizzle.first, swizzle.second}, {depositedSwizzle0, depositedSwizzle1});


    StreamSetBuffer * matchCopiedSwizzle0 = pxDriver.addBuffer<CircularCopybackBuffer>(iBuilder, iBuilder->getStreamSetTy(4), this->getInputBufferBlocks(), 1);
    StreamSetBuffer * matchCopiedSwizzle1 = pxDriver.addBuffer<CircularCopybackBuffer>(iBuilder, iBuilder->getStreamSetTy(4), this->getInputBufferBlocks(), 1);

    Kernel * swizzledMatchCopyK = pxDriver.addKernelInstance<LZ4SwizzledMatchCopyKernel>(iBuilder, 4, 2, 4);
    pxDriver.makeKernelCall(swizzledMatchCopyK, {MatchOffsetMarker, M0Marker, M0CountMarker, ByteStream, depositedSwizzle0, depositedSwizzle1}, {matchCopiedSwizzle0, matchCopiedSwizzle1});


    // Produce unswizzled bit streams
    StreamSetBuffer * extractedbits = pxDriver.addBuffer<CircularBuffer>(iBuilder, iBuilder->getStreamSetTy(8), this->getInputBufferBlocks());
    Kernel * unSwizzleK = pxDriver.addKernelInstance<SwizzleGenerator>(iBuilder, 8, 1, 2);
    pxDriver.makeKernelCall(unSwizzleK, {matchCopiedSwizzle0, matchCopiedSwizzle1}, {extractedbits});

    StreamSetBuffer * LineBreakStream;
    StreamSetBuffer * Matches;
    std::vector<re::RE*> res = {regex};
    std::tie(LineBreakStream, Matches) = grepPipeline(res, extractedbits);


    kernel::Kernel * matchCountK = pxDriver.addKernelInstance<kernel::PopcountKernel>(iBuilder);
    pxDriver.makeKernelCall(matchCountK, {Matches}, {});
    pxDriver.generatePipelineIR();


    iBuilder->setKernel(matchCountK);
    Value * matchedLineCount = iBuilder->getAccumulator("countResult");
    matchedLineCount = iBuilder->CreateZExt(matchedLineCount, iBuilder->getInt64Ty());
    iBuilder->CallPrintInt("aaa", matchedLineCount);

    pxDriver.deallocateBuffers();

    // TODO return matchedLineCount
//        idb->CreateRet(matchedLineCount);


    iBuilder->CreateRetVoid();

    pxDriver.finalizeObject();
}

ScanMatchGrepMainFunctionType LZ4GrepGenerator::getScanMatchGrepMainFunction() {
    return reinterpret_cast<ScanMatchGrepMainFunctionType>(pxDriver.getMain());
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
    inputStream = &*(args++);
    inputStream->setName("input");

    headerSize = &*(args++);
    headerSize->setName("headerSize");

    fileSize = &*(args++);
    fileSize->setName("fileSize");

    hasBlockChecksum = &*(args++);
    hasBlockChecksum->setName("hasBlockChecksum");

    match_accumulator = &*(args++);
    match_accumulator->setName("match_accumulator");

    iBuilder->SetInsertPoint(BasicBlock::Create(M->getContext(), "entry", main, 0));
}